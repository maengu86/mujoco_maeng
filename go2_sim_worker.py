#!/usr/bin/env python3
"""Go2 MuJoCo simulation worker.

Viewer(launch_passive)는 이 프로세스에서 단 한 번만 연다.
장면 전환은 부모(go2_korean_control.py)가 이 프로세스를 종료한 뒤
새 worker를 띄우는 방식으로 수행한다.

통신: stdin/stdout newline-delimited JSON
"""

from __future__ import annotations

import argparse
import json
import math
import select
import sys
import time
import traceback
from pathlib import Path
from typing import Any

from go2_defaults import DEFAULT_PARAMS

# ---------------------------------------------------------------------------
# 기본 자세 / 관절
# ---------------------------------------------------------------------------

JOINT_NAMES = [
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
]

# standing home (rad)
HOME_Q = {
    "FL_hip_joint": 0.0,
    "FL_thigh_joint": 0.9,
    "FL_calf_joint": -1.8,
    "FR_hip_joint": 0.0,
    "FR_thigh_joint": 0.9,
    "FR_calf_joint": -1.8,
    "RL_hip_joint": 0.0,
    "RL_thigh_joint": 0.9,
    "RL_calf_joint": -1.8,
    "RR_hip_joint": 0.0,
    "RR_thigh_joint": 0.9,
    "RR_calf_joint": -1.8,
}

LEG_ORDER = ["FL", "FR", "RL", "RR"]
# 대각선 트로트: FL+RR / FR+RL
TROT_PHASE = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}


def emit(obj: dict[str, Any]) -> None:
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def try_read_commands(timeout: float = 0.0) -> list[dict[str, Any]]:
    cmds: list[dict[str, Any]] = []
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            break
        line = sys.stdin.readline()
        if not line:
            break
        line = line.strip()
        if not line:
            continue
        try:
            cmds.append(json.loads(line))
        except json.JSONDecodeError:
            emit({"type": "error", "message": f"잘못된 명령 JSON: {line[:80]}"})
        timeout = 0.0
    return cmds


def quat_to_rpy(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def resolve_actuator_ids(model) -> list[int]:
    """관절 이름 순서에 맞춰 actuator id를 찾는다."""
    import mujoco

    ids: list[int] = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"관절을 찾을 수 없음: {name}")
        # 보통 actuator 이름이 관절과 같거나, joint에 연결된 actuator를 사용
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            # 이름에 _joint 제거 시도
            alt = name.replace("_joint", "")
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, alt)
        if aid < 0:
            # joint id로 actuator 검색
            found = -1
            for a in range(model.nu):
                if model.actuator_trnid[a * 2] == jid:
                    found = a
                    break
            if found < 0:
                raise RuntimeError(f"액추에이터를 찾을 수 없음: {name}")
            aid = found
        ids.append(aid)
    return ids


def resolve_qpos_dofs(model) -> list[int]:
    import mujoco

    qpos_addrs: list[int] = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_addrs.append(model.jnt_qposadr[jid])
    return qpos_addrs


def resolve_qvel_dofs(model) -> list[int]:
    import mujoco

    qvel_addrs: list[int] = []
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qvel_addrs.append(model.jnt_dofadr[jid])
    return qvel_addrs


def reset_to_home(model, data, qpos_addrs: list[int]) -> None:
    import mujoco

    # free joint 기본값 유지 + 관절 home
    mujoco.mj_resetData(model, data)
    # 몸통을 약간 띄워 시작
    if model.nq >= 7:
        data.qpos[2] = 0.32
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for name, addr in zip(JOINT_NAMES, qpos_addrs):
        data.qpos[addr] = HOME_Q[name]
    mujoco.mj_forward(model, data)


def target_pose(mode: str, t: float, params: dict[str, float]) -> dict[str, float]:
    q = dict(HOME_Q)
    freq = max(0.1, float(params["frequency"]))
    lift = float(params["lift"])
    swing = float(params["swing"])
    forward = float(params["forward_bias"])
    phase = (t * freq) % 1.0

    if mode in ("stand", "home", "pause"):
        return q

    if mode == "motor_off":
        return q

    if mode == "lift_fl":
        # 왼앞발 들기: thigh 올리고 calf 접기
        q["FL_thigh_joint"] = HOME_Q["FL_thigh_joint"] - lift * 2.5
        q["FL_calf_joint"] = HOME_Q["FL_calf_joint"] + lift * 1.5
        return q

    if mode in ("trot_inplace", "walk_forward", "walk_flat"):
        walk = mode in ("walk_forward", "walk_flat")
        for leg in LEG_ORDER:
            ph = (phase + TROT_PHASE[leg]) % 1.0
            # 0~0.5 스윙, 0.5~1.0 스탠스
            if ph < 0.5:
                s = math.sin(ph * math.pi * 2.0)  # 0..1..0 over swing half? use sin(pi*2*ph) over 0-0.5 -> 0..1..0
                s = math.sin(ph / 0.5 * math.pi)
                thigh_delta = -lift * s
                calf_delta = lift * 0.6 * s
                hip_delta = 0.0
                if walk:
                    # 전진: 스윙 중 다리를 앞으로
                    hip_delta = swing * math.sin(ph / 0.5 * math.pi) + forward
                q[f"{leg}_thigh_joint"] = HOME_Q[f"{leg}_thigh_joint"] + thigh_delta
                q[f"{leg}_calf_joint"] = HOME_Q[f"{leg}_calf_joint"] + calf_delta
                q[f"{leg}_hip_joint"] = HOME_Q[f"{leg}_hip_joint"]  # abduction 유지
                # thigh에 전후 스윙을 섞음
                if walk:
                    q[f"{leg}_thigh_joint"] += swing * 0.8 * math.sin(ph / 0.5 * math.pi)
                    q[f"{leg}_hip_joint"] += hip_delta * 0.15
            else:
                if walk:
                    # 스탠스에서 뒤로 밀기
                    st = (ph - 0.5) / 0.5
                    q[f"{leg}_thigh_joint"] = HOME_Q[f"{leg}_thigh_joint"] - swing * 0.5 * st
                    q[f"{leg}_hip_joint"] = HOME_Q[f"{leg}_hip_joint"] - forward * 0.1
        return q

    # 실험/지형 모드는 걷기 또는 서기 기반으로 처리
    if mode in ("kp_test", "kd_test", "lift_test", "swing_test"):
        return target_pose("trot_inplace", t, params)
    if mode in ("slope", "low_step", "stairs"):
        return target_pose("walk_forward", t, params)

    return q


def apply_pd(
    model,
    data,
    actuator_ids: list[int],
    qpos_addrs: list[int],
    qvel_addrs: list[int],
    q_des: dict[str, float],
    params: dict[str, float],
    motor_off: bool,
) -> None:
    kp = float(params["kp"])
    kd = float(params["kd"])
    scale = float(params["torque_scale"])
    for i, name in enumerate(JOINT_NAMES):
        if motor_off:
            data.ctrl[actuator_ids[i]] = 0.0
            continue
        q = data.qpos[qpos_addrs[i]]
        dq = data.qvel[qvel_addrs[i]]
        tau = (kp * (q_des[name] - q) - kd * dq) * scale
        # actuator ctrlrange 클램프
        lo, hi = model.actuator_ctrlrange[actuator_ids[i]]
        if hi > lo:
            tau = max(lo, min(hi, tau))
        data.ctrl[actuator_ids[i]] = tau


def run_mock_worker(scene_path: str) -> int:
    """MuJoCo 없이 IPC/생명주기만 검증하는 mock worker."""
    params = dict(DEFAULT_PARAMS)
    mode = "stand"
    paused = False
    emit({"type": "ready", "scene": scene_path, "message": "mock worker 준비 완료"})
    t0 = time.time()
    while True:
        for cmd in try_read_commands(0.05):
            name = cmd.get("cmd")
            if name == "stop":
                emit({"type": "exited", "reason": "stop"})
                return 0
            if name == "set_params":
                for k in DEFAULT_PARAMS:
                    if k in cmd:
                        params[k] = float(cmd[k])
                emit({"type": "ack", "cmd": "set_params", "params": dict(params)})
            elif name == "set_mode":
                mode = str(cmd.get("mode", "stand"))
                emit({"type": "ack", "cmd": "set_mode", "mode": mode})
            elif name == "home":
                mode = "stand"
                paused = False
                emit({"type": "ack", "cmd": "home"})
            elif name == "pause":
                paused = bool(cmd.get("paused", True))
                emit({"type": "ack", "cmd": "pause", "paused": paused})
            elif name == "motor_off":
                mode = "motor_off"
                emit({"type": "ack", "cmd": "motor_off"})
            elif name == "ping":
                emit({"type": "pong"})
        height = 0.30 + 0.01 * math.sin(time.time() - t0)
        emit(
            {
                "type": "status",
                "height": height,
                "roll": 0.01,
                "pitch": -0.02,
                "speed": 0.05 if mode in ("walk_forward", "walk_flat", "trot_inplace") else 0.0,
                "fallen": False,
                "mode": mode,
                "paused": paused,
                "params": dict(params),
                "alive": True,
            }
        )


def run_worker(scene_path: str, headless: bool = False, mock: bool = False) -> int:
    if mock:
        return run_mock_worker(scene_path)

    import mujoco

    scene = Path(scene_path).resolve()
    if not scene.exists():
        emit({"type": "error", "message": f"장면 파일이 없습니다: {scene}"})
        return 2

    try:
        model = mujoco.MjModel.from_xml_path(str(scene))
        data = mujoco.MjData(model)
    except Exception as exc:  # noqa: BLE001
        emit({"type": "error", "message": f"장면 로드 실패: {exc}"})
        return 3

    actuator_ids = resolve_actuator_ids(model)
    qpos_addrs = resolve_qpos_dofs(model)
    qvel_addrs = resolve_qvel_dofs(model)

    params = dict(DEFAULT_PARAMS)
    mode = "stand"
    paused = False
    sim_t0 = time.time()
    last_status = 0.0

    reset_to_home(model, data, qpos_addrs)
    # ready 는 Viewer가 실제로 열린 뒤에만 보낸다.
    # (이전에 모델 로드 직후 ready를 보내 HTML만 성공처럼 보이던 문제를 방지)

    def status_payload() -> dict[str, Any]:
        # free joint: qpos[0:3] pos, qpos[3:7] quat
        height = float(data.qpos[2]) if model.nq >= 3 else 0.0
        roll = pitch = 0.0
        if model.nq >= 7:
            qw, qx, qy, qz = [float(x) for x in data.qpos[3:7]]
            roll, pitch, _ = quat_to_rpy(qw, qx, qy, qz)
        vx = float(data.qvel[0]) if model.nv >= 1 else 0.0
        vy = float(data.qvel[1]) if model.nv >= 2 else 0.0
        speed = math.hypot(vx, vy)
        fallen = height < 0.12 or abs(roll) > 0.85 or abs(pitch) > 0.85
        return {
            "type": "status",
            "height": height,
            "roll": roll,
            "pitch": pitch,
            "speed": speed,
            "fallen": fallen,
            "mode": mode,
            "paused": paused,
            "params": dict(params),
            "alive": True,
        }

    def handle_cmd(cmd: dict[str, Any]) -> bool:
        """False면 종료."""
        nonlocal mode, paused, params, sim_t0
        name = cmd.get("cmd")
        if name == "stop":
            return False
        if name == "ping":
            emit({"type": "pong"})
            return True
        if name == "set_params":
            for k in DEFAULT_PARAMS:
                if k in cmd:
                    params[k] = float(cmd[k])
            emit({"type": "ack", "cmd": "set_params", "params": dict(params)})
            return True
        if name == "set_mode":
            mode = str(cmd.get("mode", "stand"))
            if mode == "home":
                reset_to_home(model, data, qpos_addrs)
                mode = "stand"
                sim_t0 = time.time()
            emit({"type": "ack", "cmd": "set_mode", "mode": mode})
            return True
        if name == "home":
            reset_to_home(model, data, qpos_addrs)
            mode = "stand"
            paused = False
            sim_t0 = time.time()
            emit({"type": "ack", "cmd": "home"})
            return True
        if name == "pause":
            paused = bool(cmd.get("paused", True))
            emit({"type": "ack", "cmd": "pause", "paused": paused})
            return True
        if name == "motor_off":
            mode = "motor_off"
            emit({"type": "ack", "cmd": "motor_off"})
            return True
        return True

    def control_step() -> None:
        t = time.time() - sim_t0
        if paused:
            data.ctrl[:] = 0.0
            return
        q_des = target_pose(mode, t, params)
        apply_pd(
            model,
            data,
            actuator_ids,
            qpos_addrs,
            qvel_addrs,
            q_des,
            params,
            motor_off=(mode == "motor_off"),
        )

    try:
        if headless:
            # CI/컴파일·연동 테스트용 (Viewer 없음)
            emit(
                {
                    "type": "ready",
                    "scene": str(scene),
                    "viewer": False,
                    "message": "headless worker 준비 완료",
                }
            )
            while True:
                for cmd in try_read_commands(0.0):
                    if not handle_cmd(cmd):
                        emit({"type": "exited", "reason": "stop"})
                        return 0
                control_step()
                mujoco.mj_step(model, data)
                now = time.time()
                if now - last_status >= 0.1:
                    emit(status_payload())
                    last_status = now
                time.sleep(model.opt.timestep * 0.25)
        else:
            import mujoco.viewer

            print(f"[worker] Viewer 여는 중: {scene}", file=sys.stderr, flush=True)
            with mujoco.viewer.launch_passive(model, data) as viewer:
                emit(
                    {
                        "type": "ready",
                        "scene": str(scene),
                        "viewer": True,
                        "message": "MuJoCo Viewer 실행 완료",
                    }
                )
                print("[worker] Viewer 실행 완료", file=sys.stderr, flush=True)
                while viewer.is_running():
                    for cmd in try_read_commands(0.0):
                        if not handle_cmd(cmd):
                            emit({"type": "exited", "reason": "stop"})
                            return 0
                    step_start = time.time()
                    control_step()
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    now = time.time()
                    if now - last_status >= 0.1:
                        emit(status_payload())
                        last_status = now
                    # 실시간 맞추기
                    elapsed = time.time() - step_start
                    sleep_t = model.opt.timestep - elapsed
                    if sleep_t > 0:
                        time.sleep(sleep_t)
            emit({"type": "exited", "reason": "viewer_closed"})
            return 0
    except Exception as exc:  # noqa: BLE001
        emit(
            {
                "type": "error",
                "message": f"MuJoCo 실행 오류: {exc}",
                "trace": traceback.format_exc(),
            }
        )
        print(traceback.format_exc(), file=sys.stderr, flush=True)
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Go2 MuJoCo sim worker")
    parser.add_argument("--scene", required=True, help="scene.xml 경로")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Viewer 없이 실행 (테스트용)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="MuJoCo 없이 IPC만 검증 (테스트용)",
    )
    args = parser.parse_args()
    try:
        return run_worker(args.scene, headless=args.headless, mock=args.mock)
    except Exception as exc:  # noqa: BLE001
        emit({"type": "error", "message": f"MuJoCo 실행 오류: {exc}"})
        return 1


if __name__ == "__main__":
    sys.exit(main())
