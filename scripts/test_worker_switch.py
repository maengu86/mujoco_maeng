#!/usr/bin/env python3
"""worker 장면 전환을 10회 이상 반복하고, 종료 후 잔여 프로세스가 없는지 확인."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

os.environ["GO2_MOCK"] = "1"

from go2_korean_control import MANAGER, cleanup  # noqa: E402


def count_workers() -> int:
    out = subprocess.check_output(["ps", "aux"], text=True)
    n = 0
    for line in out.splitlines():
        if "go2_sim_worker.py" in line and "grep" not in line:
            n += 1
    return n


def main() -> int:
    print("[test] mock worker 시작 (평지)")
    MANAGER.start_worker("flat")
    time.sleep(0.3)
    snap = MANAGER.snapshot()
    assert snap.get("viewer_ready") is True, snap
    assert "평지" in snap["ui_status"] or "실행" in snap["ui_status"], snap

    print("[test] 모드/파라미터")
    MANAGER.set_params({"kp": 42, "kd": 2.2})
    MANAGER.set_mode("trot_inplace")
    time.sleep(0.2)

    print("[test] 장면 전환 12회")
    for i in range(12):
        key = "terrain" if i % 2 == 0 else "flat"
        print(f"  round {i+1}: -> {key}")
        snap = MANAGER.switch_scene(key)
        assert not snap["switching"]
        assert snap["scene"] == key
        assert snap.get("viewer_ready") is True, snap
        assert "오류" not in (snap["ui_status"] or ""), snap
        # 종료 반영 대기 후 잔여 확인
        time.sleep(0.2)
        living = count_workers()
        assert living <= 1, f"worker too many: {living}"

    print("[test] Ctrl+C 유사 정리")
    cleanup()
    time.sleep(0.4)
    left = count_workers()
    print(f"[test] 남은 go2_sim_worker: {left}")
    if left != 0:
        print("[FAIL] worker 잔여 프로세스 존재")
        return 1
    print("[OK] worker 전환/종료 검증 통과")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
