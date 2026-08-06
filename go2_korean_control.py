#!/usr/bin/env python3
"""Go2 한글 조작기 — 교육용 실험실 메인 프로세스.

역할:
- 브라우저 UI 제공
- 파라미터 / 로그 / 분석 관리
- go2_sim_worker.py 시작·종료·장면 전환

Viewer는 worker 프로세스에서만 실행한다.
macOS + mjpython에서 launch_passive를 같은 프로세스에서
재호출하지 않도록 구조를 분리했다.
"""

from __future__ import annotations

import atexit
import csv
import json
import os
import signal
import subprocess
import sys
import threading
import time
import traceback
import webbrowser
from collections import defaultdict
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
HTML_PATH = ROOT / "go2_control_panel_ko.html"
WORKER_PATH = ROOT / "go2_sim_worker.py"
LOG_PATH = ROOT / "go2_experiment_log.csv"
WORKER_LOG_PATH = ROOT / "go2_worker.log"
SCENE_FLAT = ROOT / "unitree_robots" / "go2" / "scene.xml"
SCENE_TERRAIN = ROOT / "unitree_robots" / "go2" / "scene_terrain.xml"

HOST = "127.0.0.1"
PORT = 8765

from go2_defaults import DEFAULT_PARAMS  # noqa: E402


def resolve_worker_python() -> str:
    """macOS Viewer는 mjpython이 필수. worker도 반드시 mjpython으로 띄운다."""
    env = os.environ.get("GO2_PYTHON", "").strip()
    if env and Path(env).exists():
        return env

    candidates: list[Path] = []
    exe = Path(sys.executable)
    candidates.append(exe)
    candidates.append(exe.with_name("mjpython"))
    candidates.append(ROOT / ".venv" / "bin" / "mjpython")
    candidates.append(ROOT / ".venv" / "bin" / "python")

    which = None
    try:
        import shutil

        which = shutil.which("mjpython")
    except Exception:  # noqa: BLE001
        which = None
    if which:
        candidates.append(Path(which))

    # 1순위: 이름에 mjpython 이 들어간 실행파일
    for c in candidates:
        if c.is_file() and "mjpython" in c.name:
            return str(c.resolve())

    # 2순위: 현재 인터프리터
    if exe.is_file():
        return str(exe.resolve())

    return sys.executable

MODE_LABELS = {
    "stand": "서기",
    "lift_fl": "왼앞발 들기",
    "trot_inplace": "제자리 트로트",
    "walk_forward": "앞으로 걷기 실험",
    "walk_flat": "평지 걷기",
    "motor_off": "모터 힘 끄기",
    "home": "home 초기화",
    "pause": "일시정지",
    "kp_test": "Kp 실험",
    "kd_test": "Kd 실험",
    "lift_test": "발 높이 실험",
    "swing_test": "보폭 실험",
    "slope": "경사",
    "low_step": "낮은 턱",
    "stairs": "계단",
}

WIZARD_STEPS = [
    {"id": "stand", "title": "서기", "hint": "기본 자세를 잡고 균형을 확인합니다."},
    {"id": "lift_fl", "title": "발 하나 들기", "hint": "한 다리 지지로 무게 이동을 느껴봅니다."},
    {"id": "trot_inplace", "title": "제자리 트로트", "hint": "대각선 다리 패턴을 관찰합니다."},
    {"id": "walk_flat", "title": "평지 걷기", "hint": "전진 편향과 보폭을 함께 봅니다."},
    {"id": "kp_test", "title": "Kp 실험", "hint": "자세 복원력을 바꿔 떨림/주저앉음을 비교합니다."},
    {"id": "kd_test", "title": "Kd 실험", "hint": "감쇠를 바꿔 진동과 반응성을 비교합니다."},
    {"id": "lift_test", "title": "발 높이 실험", "hint": "장애물을 넘을 때 필요한 들림을 찾습니다."},
    {"id": "swing_test", "title": "보폭 실험", "hint": "보폭이 속도와 안정도에 미치는 영향을 봅니다."},
    {"id": "flat_done", "title": "평지 완료", "hint": "평지 기록을 남기고 지형으로 넘어갑니다."},
    {"id": "slope", "title": "경사", "hint": "경사에서 Pitch 변화와 미끄러짐을 관찰합니다."},
    {"id": "low_step", "title": "낮은 턱", "hint": "발 높이가 충분한지 확인합니다."},
    {"id": "stairs", "title": "계단", "hint": "주기·보폭·토크 배율을 함께 조정합니다."},
]

CSV_HEADER = [
    "날짜",
    "모드",
    "Kp",
    "Kd",
    "Lift",
    "Swing",
    "Frequency",
    "Forward Bias",
    "Torque Scale",
    "몸통 높이",
    "Pitch",
    "Roll",
    "속도",
    "넘어짐 여부",
    "메모",
]


class WorkerManager:
    """go2_sim_worker 생명주기 관리."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.proc: subprocess.Popen[str] | None = None
        self.scene_key = "flat"  # flat | terrain
        self.ui_status = "대기 중"
        self.switching = False
        self.error: str | None = None
        self.viewer_ready = False
        self.worker_python = resolve_worker_python()
        self.params = dict(DEFAULT_PARAMS)
        self.mode = "stand"
        self.paused = False
        self.last_status: dict[str, Any] = {
            "height": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "speed": 0.0,
            "fallen": False,
            "alive": False,
        }
        self.wizard_index = -1  # -1 = 비활성
        self._reader: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self._stderr_fp = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "ui_status": self.ui_status,
                "switching": self.switching,
                "scene": self.scene_key,
                "error": self.error,
                "viewer_ready": self.viewer_ready,
                "worker_python": self.worker_python,
                "params": dict(self.params),
                "mode": self.mode,
                "mode_label": MODE_LABELS.get(self.mode, self.mode),
                "paused": self.paused,
                "worker_alive": self._alive_unlocked(),
                "status": dict(self.last_status),
                "wizard_index": self.wizard_index,
                "wizard_steps": WIZARD_STEPS,
                "defaults": dict(DEFAULT_PARAMS),
                "log_path": str(LOG_PATH),
                "worker_log_path": str(WORKER_LOG_PATH),
            }

    def _alive_unlocked(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def _scene_path(self, key: str) -> Path:
        return SCENE_FLAT if key == "flat" else SCENE_TERRAIN

    def _set_status(self, text: str) -> None:
        self.ui_status = text

    def send(self, payload: dict[str, Any]) -> None:
        with self._lock:
            if not self._alive_unlocked() or self.proc is None or self.proc.stdin is None:
                return
            try:
                self.proc.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
                self.proc.stdin.flush()
            except BrokenPipeError:
                self.error = "MuJoCo 실행 오류"
                self._set_status("MuJoCo 실행 오류")

    def _read_loop(self, proc: subprocess.Popen[str]) -> None:
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                if self._stop_reader.is_set():
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._handle_worker_msg(msg, proc)
        except Exception:  # noqa: BLE001
            pass
        finally:
            with self._lock:
                if self.proc is proc and proc.poll() is not None:
                    if not self.switching:
                        self.error = "MuJoCo 실행 오류"
                        self._set_status("MuJoCo 실행 오류")
                    self.last_status["alive"] = False

    def _handle_worker_msg(self, msg: dict[str, Any], proc: subprocess.Popen[str]) -> None:
        with self._lock:
            if self.proc is not proc:
                return
            mtype = msg.get("type")
            if mtype == "status":
                self.last_status = {
                    "height": float(msg.get("height", 0.0)),
                    "roll": float(msg.get("roll", 0.0)),
                    "pitch": float(msg.get("pitch", 0.0)),
                    "speed": float(msg.get("speed", 0.0)),
                    "fallen": bool(msg.get("fallen", False)),
                    "alive": True,
                }
                if "mode" in msg:
                    self.mode = str(msg["mode"])
                if "paused" in msg:
                    self.paused = bool(msg["paused"])
            elif mtype == "ready":
                self.error = None
                self.viewer_ready = True
                label = "평지 실행 중" if self.scene_key == "flat" else "지형 실행 완료"
                self._set_status(label)
                print(f"[main] Viewer ready: {msg.get('message', '')}", flush=True)
            elif mtype == "error":
                self.error = str(msg.get("message", "MuJoCo 실행 오류"))
                self.viewer_ready = False
                self._set_status("MuJoCo 실행 오류")
                print(f"[main] worker error: {self.error}", file=sys.stderr, flush=True)
            elif mtype == "exited":
                self.last_status["alive"] = False
                self.viewer_ready = False
                if not self.switching:
                    self._set_status("Viewer 종료됨")

    def stop_worker(self, reason: str = "Viewer 종료 중") -> None:
        with self._lock:
            self._set_status(reason)
            self.viewer_ready = False
            proc = self.proc
            self.proc = None
            fp = self._stderr_fp
            self._stderr_fp = None
        if proc is None:
            if fp:
                try:
                    fp.close()
                except Exception:  # noqa: BLE001
                    pass
            return
        try:
            if proc.poll() is None and proc.stdin is not None:
                try:
                    proc.stdin.write(json.dumps({"cmd": "stop"}) + "\n")
                    proc.stdin.flush()
                except Exception:  # noqa: BLE001
                    pass
            try:
                proc.wait(timeout=2.5)
            except subprocess.TimeoutExpired:
                self._kill_proc(proc)
                try:
                    proc.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    pass
        finally:
            self._stop_reader.set()
            if self._reader and self._reader.is_alive():
                self._reader.join(timeout=1.0)
            self._reader = None
            self._stop_reader.clear()
            try:
                if proc.poll() is None:
                    self._kill_proc(proc)
            except Exception:  # noqa: BLE001
                pass
            if fp:
                try:
                    fp.close()
                except Exception:  # noqa: BLE001
                    pass

    @staticmethod
    def _kill_proc(proc: subprocess.Popen[str]) -> None:
        """macOS GUI worker는 새 세션을 쓰지 않으므로 일반 terminate/kill 사용."""
        try:
            if proc.poll() is None:
                proc.terminate()
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.05)
        try:
            if proc.poll() is None:
                proc.kill()
        except Exception:  # noqa: BLE001
            pass
        # process group 이 있는 경우에만 추가 정리
        try:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            pass

    def start_worker(self, scene_key: str) -> None:
        scene = self._scene_path(scene_key)
        mock = os.environ.get("GO2_MOCK", "").strip() in ("1", "true", "yes")
        if not scene.exists():
            if mock:
                scene.parent.mkdir(parents=True, exist_ok=True)
                if not scene.exists():
                    scene.write_text("<!-- mock scene placeholder -->\n", encoding="utf-8")
            else:
                with self._lock:
                    self.error = f"장면 파일 없음: {scene}"
                    self.viewer_ready = False
                    self._set_status("MuJoCo 실행 오류")
                raise FileNotFoundError(scene)

        with self._lock:
            loading = "지형 불러오는 중" if scene_key == "terrain" else "평지 불러오는 중"
            self._set_status(loading)
            self.scene_key = scene_key
            self.error = None
            self.viewer_ready = False

        headless = os.environ.get("GO2_HEADLESS", "").strip() in ("1", "true", "yes")
        py = resolve_worker_python()
        self.worker_python = py
        cmd = [py, str(WORKER_PATH), "--scene", str(scene)]
        if mock:
            cmd.append("--mock")
        elif headless:
            cmd.append("--headless")

        # stderr를 PIPE로 막으면 MuJoCo 로그가 버퍼를 채워 worker가 멈출 수 있다.
        # 파일로 보내서 Viewer가 실제로 뜨게 한다.
        WORKER_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        stderr_fp = WORKER_LOG_PATH.open("a", encoding="utf-8")
        stderr_fp.write("\n" + "=" * 60 + "\n")
        stderr_fp.write(f"cmd: {' '.join(cmd)}\n")
        stderr_fp.write(f"cwd: {ROOT}\n")
        stderr_fp.flush()

        print(f"[main] worker 시작: {' '.join(cmd)}", flush=True)
        if sys.platform == "darwin" and "mjpython" not in Path(py).name and not mock:
            msg = (
                "macOS에서는 mjpython으로 실행해야 MuJoCo 창이 열립니다. "
                f"현재 worker python={py}"
            )
            print(f"[경고] {msg}", file=sys.stderr, flush=True)

        # macOS GUI: start_new_session=False (창이 안 뜨는 문제 방지)
        # Linux: True 여도 무방하나, 종료 단순화를 위해 False로 통일
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=stderr_fp,
            text=True,
            bufsize=1,
            cwd=str(ROOT),
            start_new_session=False,
            env={**os.environ, "PYTHONUNBUFFERED": "1"},
        )
        with self._lock:
            self.proc = proc
            self._stderr_fp = stderr_fp
            self._stop_reader.clear()
            self._reader = threading.Thread(
                target=self._read_loop, args=(proc,), daemon=True
            )
            self._reader.start()

        # Viewer ready 를 받을 때까지 대기 (최대 20초 — 모델 로드 시간 감안)
        deadline = time.time() + 20.0
        ready = False
        while time.time() < deadline:
            with self._lock:
                if self.viewer_ready:
                    ready = True
                    break
                if self.error:
                    break
                if proc.poll() is not None:
                    tail = ""
                    try:
                        tail = WORKER_LOG_PATH.read_text(encoding="utf-8")[-800:]
                    except Exception:  # noqa: BLE001
                        pass
                    self.error = "MuJoCo 실행 오류 (worker 종료). go2_worker.log 확인"
                    self._set_status("MuJoCo 실행 오류")
                    print(f"[main] worker 조기 종료. log tail:\n{tail}", file=sys.stderr, flush=True)
                    break
            time.sleep(0.05)

        if not ready and not self.error:
            with self._lock:
                self.error = "MuJoCo Viewer가 열리지 않았습니다. go2_worker.log 를 확인하세요."
                self._set_status("MuJoCo 실행 오류")
            print("[main] Viewer ready timeout — HTML만 열린 상태일 수 있음", file=sys.stderr, flush=True)
            return

        if not ready:
            return

        # 파라미터·home 동기화 (Viewer가 열린 뒤에만)
        self.send({"cmd": "set_params", **self.params})
        self.send({"cmd": "home"})
        with self._lock:
            self.mode = "stand"
            self.paused = False
            self._set_status(
                "평지 실행 중" if scene_key == "flat" else "지형 실행 완료"
            )

    def switch_scene(self, scene_key: str) -> dict[str, Any]:
        with self._lock:
            if self.switching:
                return self.snapshot()
            self.switching = True
            self.error = None
        try:
            self.stop_worker("Viewer 종료 중")
            # join/wait 완료 후 새 worker
            time.sleep(0.05)  # 스케줄 양보만 (viewer 재생성 대기용 sleep 아님)
            self.start_worker(scene_key)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.error = f"MuJoCo 실행 오류: {exc}"
                self._set_status("MuJoCo 실행 오류")
        finally:
            with self._lock:
                self.switching = False
        return self.snapshot()

    def set_params(self, updates: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            for k in DEFAULT_PARAMS:
                if k in updates and updates[k] is not None:
                    self.params[k] = float(updates[k])
            payload = {"cmd": "set_params", **self.params}
        self.send(payload)
        return self.snapshot()

    def set_mode(self, mode: str) -> dict[str, Any]:
        with self._lock:
            self.mode = mode
            if mode == "pause":
                self.paused = True
                self.send({"cmd": "pause", "paused": True})
            elif mode == "home":
                self.paused = False
                self.send({"cmd": "home"})
                self.mode = "stand"
            elif mode == "motor_off":
                self.paused = False
                self.send({"cmd": "motor_off"})
            else:
                self.paused = False
                self.send({"cmd": "pause", "paused": False})
                self.send({"cmd": "set_mode", "mode": mode})
        return self.snapshot()

    def restore_defaults(self) -> dict[str, Any]:
        """초기 기본값 복원 + home 자세 초기화 (v2 동작)."""
        with self._lock:
            self.params = dict(DEFAULT_PARAMS)
            payload = {"cmd": "set_params", **self.params}
        self.send(payload)
        self.send({"cmd": "home"})
        with self._lock:
            self.mode = "stand"
            self.paused = False
        return self.snapshot()

    def set_wizard(self, index: int) -> dict[str, Any]:
        with self._lock:
            if index < 0:
                self.wizard_index = -1
                return self.snapshot()
            index = max(0, min(len(WIZARD_STEPS) - 1, index))
            self.wizard_index = index
            step = WIZARD_STEPS[index]
            step_id = step["id"]

        # 평지 완료 → 지형 전환 유도
        if step_id == "flat_done":
            snap = self.switch_scene("terrain")
            with self._lock:
                self.wizard_index = index
            return snap

        # 경사/턱/계단은 지형 장면 권장
        if step_id in ("slope", "low_step", "stairs") and self.scene_key != "terrain":
            self.switch_scene("terrain")

        # 평지 단계인데 지형이면 평지로
        if step_id in (
            "stand",
            "lift_fl",
            "trot_inplace",
            "walk_flat",
            "kp_test",
            "kd_test",
            "lift_test",
            "swing_test",
        ) and self.scene_key != "flat":
            self.switch_scene("flat")

        # 실험 모드별 파라미터 살짝 안내값
        if step_id == "kp_test":
            self.set_params({"kp": 55.0})
        elif step_id == "kd_test":
            self.set_params({"kd": 2.3})
        elif step_id == "lift_test":
            self.set_params({"lift": 0.18})
        elif step_id == "swing_test":
            self.set_params({"swing": 0.12, "forward_bias": 0.08})

        mode_map = {
            "stand": "stand",
            "lift_fl": "lift_fl",
            "trot_inplace": "trot_inplace",
            "walk_flat": "walk_forward",
            "kp_test": "trot_inplace",
            "kd_test": "trot_inplace",
            "lift_test": "trot_inplace",
            "swing_test": "walk_forward",
            "flat_done": "stand",
            "slope": "walk_forward",
            "low_step": "walk_forward",
            "stairs": "walk_forward",
        }
        self.set_mode(mode_map.get(step_id, "stand"))
        with self._lock:
            self.wizard_index = index
        return self.snapshot()


MANAGER = WorkerManager()


def ensure_log() -> None:
    if not LOG_PATH.exists():
        with LOG_PATH.open("w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(CSV_HEADER)


def append_log(memo: str = "") -> dict[str, Any]:
    ensure_log()
    snap = MANAGER.snapshot()
    st = snap["status"]
    params = snap["params"]
    row = [
        datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        snap.get("mode_label", snap.get("mode", "")),
        params.get("kp"),
        params.get("kd"),
        params.get("lift"),
        params.get("swing"),
        params.get("frequency"),
        params.get("forward_bias"),
        params.get("torque_scale"),
        round(float(st.get("height", 0.0)), 4),
        round(float(st.get("pitch", 0.0)), 4),
        round(float(st.get("roll", 0.0)), 4),
        round(float(st.get("speed", 0.0)), 4),
        "예" if st.get("fallen") else "아니오",
        memo.strip(),
    ]
    with LOG_PATH.open("a", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(row)
    return {
        "ok": True,
        "row": dict(zip(CSV_HEADER, row)),
        "filename": LOG_PATH.name,
        "path": str(LOG_PATH),
        "message": f"저장 완료: {LOG_PATH.name} ({LOG_PATH})",
    }


def read_log_rows() -> list[dict[str, str]]:
    ensure_log()
    with LOG_PATH.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def analyze_logs() -> dict[str, Any]:
    rows = read_log_rows()
    insights: list[str] = []
    if len(rows) < 3:
        return {
            "count": len(rows),
            "insights": [
                "기록이 아직 부족합니다. 동작을 바꾼 뒤 ‘기록 저장’을 여러 번 눌러 보세요."
            ],
            "tips": [
                "같은 동작에서 Kp만 바꿔가며 3회 이상 기록하면 분석이 선명해집니다.",
                "넘어졌다면 메모에 ‘Kp 너무 높음’처럼 원인을 적어 두세요.",
            ],
        }

    def fnum(v: Any, default: float = 0.0) -> float:
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    # Kp vs 넘어짐
    buckets: dict[str, list[int]] = defaultdict(list)
    kd_stable: dict[str, list[int]] = defaultdict(list)
    for r in rows:
        fallen = 1 if str(r.get("넘어짐 여부", "")).strip() in ("예", "true", "1", "True") else 0
        kp = fnum(r.get("Kp"))
        kd = fnum(r.get("Kd"))
        if kp <= 30:
            key = "Kp≤30"
        elif kp <= 45:
            key = "Kp 30~45"
        elif kp <= 60:
            key = "Kp 45~60"
        else:
            key = "Kp>60"
        buckets[key].append(fallen)

        if kd < 1.5:
            kd_key = "Kd<1.5"
        elif kd <= 2.5:
            kd_key = "Kd 1.5~2.5"
        elif kd <= 3.5:
            kd_key = "Kd 2.5~3.5"
        else:
            kd_key = "Kd>3.5"
        kd_stable[kd_key].append(fallen)

    def fall_rate(vals: list[int]) -> float:
        return sum(vals) / max(1, len(vals))

    if buckets:
        ordered = sorted(buckets.items(), key=lambda kv: fall_rate(kv[1]))
        worst = max(buckets.items(), key=lambda kv: fall_rate(kv[1]))
        best = ordered[0]
        if fall_rate(worst[1]) > fall_rate(best[1]):
            if "Kp>" in worst[0] or worst[0].startswith("Kp 45"):
                insights.append(
                    f"{worst[0]} 구간에서 넘어짐이 더 잦습니다 "
                    f"({fall_rate(worst[1])*100:.0f}%). Kp를 높일수록 떨림·튐으로 불안정해질 수 있습니다."
                )
            else:
                insights.append(
                    f"{worst[0]}에서 넘어짐 비율이 높습니다. 자세가 주저앉는지 확인해 보세요."
                )
            insights.append(
                f"상대적으로 {best[0]}가 안정적입니다 (넘어짐 {fall_rate(best[1])*100:.0f}%)."
            )

    if kd_stable:
        best_kd = min(kd_stable.items(), key=lambda kv: fall_rate(kv[1]))
        insights.append(
            f"{best_kd[0]} 구간이 가장 안정적입니다 "
            f"(넘어짐 {fall_rate(best_kd[1])*100:.0f}%). 보통 Kd 2.0~2.5 근처를 권장합니다."
        )

    # 높이 평균
    heights = [fnum(r.get("몸통 높이")) for r in rows if r.get("몸통 높이")]
    if heights:
        avg_h = sum(heights) / len(heights)
        insights.append(f"기록된 평균 몸통 높이는 {avg_h:.3f} m 입니다. 0.25~0.35 m 부근이 서기에 유리합니다.")

    if not insights:
        insights.append("아직 뚜렷한 패턴이 없습니다. 파라미터를 하나씩만 바꿔 기록해 보세요.")

    return {
        "count": len(rows),
        "insights": insights,
        "tips": [
            "한 번에 변수 하나만 바꾸기 (과학 실험의 기본).",
            "평지에서 안정된 뒤 지형(경사·턱·계단)으로 확장하기.",
            "메모에 ‘왜 실패했는지’를 남기면 다음 실험이 쉬워집니다.",
        ],
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "Go2Lab/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        # 콘솔 소음 줄이기
        sys.stderr.write("[http] " + (fmt % args) + "\n")

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, obj: Any) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _text(self, code: int, text: str, content_type: str) -> None:
        body = text.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            if not HTML_PATH.exists():
                self._text(404, "go2_control_panel_ko.html 없음", "text/plain; charset=utf-8")
                return
            self._text(200, HTML_PATH.read_text(encoding="utf-8"), "text/html; charset=utf-8")
            return
        if path == "/api/status":
            self._json(200, MANAGER.snapshot())
            return
        if path == "/api/analysis":
            self._json(200, analyze_logs())
            return
        if path == "/api/log":
            self._json(200, {"rows": read_log_rows()[-50:]})
            return
        self._json(404, {"error": "not found"})

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or 0)
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
            return data if isinstance(data, dict) else {}
        except json.JSONDecodeError:
            return {}

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        data = self._read_json()
        try:
            if path == "/api/scene":
                key = str(data.get("scene", "flat"))
                if key not in ("flat", "terrain"):
                    self._json(400, {"error": "scene must be flat|terrain"})
                    return
                snap = MANAGER.switch_scene(key)
                self._json(200, snap)
                return
            if path == "/api/mode":
                mode = str(data.get("mode", "stand"))
                self._json(200, MANAGER.set_mode(mode))
                return
            if path == "/api/params":
                self._json(200, MANAGER.set_params(data))
                return
            if path == "/api/defaults":
                self._json(200, MANAGER.restore_defaults())
                return
            if path == "/api/log":
                memo = str(data.get("memo", ""))
                self._json(200, append_log(memo))
                return
            if path == "/api/wizard":
                if "index" in data:
                    self._json(200, MANAGER.set_wizard(int(data["index"])))
                elif data.get("action") == "next":
                    idx = MANAGER.wizard_index
                    if idx < 0:
                        idx = 0
                    else:
                        idx = min(len(WIZARD_STEPS) - 1, idx + 1)
                    self._json(200, MANAGER.set_wizard(idx))
                elif data.get("action") == "prev":
                    idx = max(0, MANAGER.wizard_index - 1)
                    self._json(200, MANAGER.set_wizard(idx))
                elif data.get("action") == "close":
                    self._json(200, MANAGER.set_wizard(-1))
                else:
                    self._json(200, MANAGER.set_wizard(0))
                return
            if path == "/api/pause":
                paused = bool(data.get("paused", True))
                if paused:
                    self._json(200, MANAGER.set_mode("pause"))
                else:
                    self._json(200, MANAGER.set_mode(MANAGER.mode if MANAGER.mode != "pause" else "stand"))
                return
            self._json(404, {"error": "not found"})
        except Exception as exc:  # noqa: BLE001
            self._json(500, {"error": str(exc), "trace": traceback.format_exc()})


def cleanup(*_args: Any) -> None:
    try:
        MANAGER.stop_worker("종료 중")
    except Exception:  # noqa: BLE001
        pass


def main() -> int:
    if not WORKER_PATH.exists():
        print(f"[오류] worker 파일이 없습니다: {WORKER_PATH}", file=sys.stderr)
        return 1
    if not HTML_PATH.exists():
        print(f"[오류] UI 파일이 없습니다: {HTML_PATH}", file=sys.stderr)
        return 1
    if not SCENE_FLAT.exists():
        print(
            f"[오류] 평지 장면이 없습니다: {SCENE_FLAT}\n"
            "저장소 루트에서 실행하세요. unitree_robots/go2/scene.xml 이 필요합니다.",
            file=sys.stderr,
        )
        return 1

    mock = os.environ.get("GO2_MOCK", "").strip() in ("1", "true", "yes")
    worker_py = resolve_worker_python()
    if sys.platform == "darwin" and "mjpython" not in Path(worker_py).name and not mock:
        print("=" * 60, file=sys.stderr)
        print("[오류] macOS 에서 MuJoCo 창을 열려면 mjpython 이 필요합니다.", file=sys.stderr)
        print(f"  현재 감지된 python: {worker_py}", file=sys.stderr)
        print("  아래처럼 실행하세요:", file=sys.stderr)
        print("    bash scripts/setup_env.sh", file=sys.stderr)
        print("    bash scripts/run_lab.sh", file=sys.stderr)
        print("  또는:", file=sys.stderr)
        print("    source .venv/bin/activate", file=sys.stderr)
        print("    ./.venv/bin/mjpython go2_korean_control.py", file=sys.stderr)
        print("=" * 60, file=sys.stderr)
        return 2

    ensure_log()

    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    atexit.register(cleanup)

    httpd = ThreadingHTTPServer((HOST, PORT), Handler)
    print("=" * 56)
    print("  Go2 한글 조작기 (입문 실험실)")
    print(f"  브라우저: http://{HOST}:{PORT}")
    print(f"  worker : {worker_py}")
    print(f"  장면   : {SCENE_FLAT}")
    print(f"  기록   : {LOG_PATH}")
    print(f"  worker로그: {WORKER_LOG_PATH}")
    print("  Ctrl+C 로 종료 (MuJoCo worker도 함께 종료)")
    print("=" * 56)

    # 먼저 MuJoCo worker(Viewer)를 띄운 뒤 브라우저를 연다
    try:
        MANAGER.start_worker("flat")
    except Exception as exc:  # noqa: BLE001
        MANAGER.error = str(exc)
        MANAGER.ui_status = "MuJoCo 실행 오류"
        print(f"[오류] worker 시작 실패: {exc}", file=sys.stderr)

    if not MANAGER.viewer_ready and not mock:
        print(
            "[오류] MuJoCo Viewer가 열리지 않았습니다. HTML만 보이면 실패 상태입니다.\n"
            f"  로그: {WORKER_LOG_PATH}",
            file=sys.stderr,
        )

    try:
        webbrowser.open(f"http://{HOST}:{PORT}")
    except Exception:  # noqa: BLE001
        pass

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[종료] worker 정리 중...")
    finally:
        cleanup()
        httpd.server_close()
        print("[종료] 완료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
