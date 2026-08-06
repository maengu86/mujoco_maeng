"""Go2 한글 조작기 v2 공통 기본값.

메인/worker/UI가 같은 초기값을 쓰도록 한곳에 둔다.
"""

from __future__ import annotations

DEFAULT_PARAMS: dict[str, float] = {
    "kp": 40.0,
    "kd": 1.5,
    "frequency": 1.2,  # 보행 주기 (Hz)
    "lift": 0.28,  # 발 들기 크기
    "swing": 0.18,  # 앞뒤 스윙 크기
    "forward_bias": 0.05,  # 전진 편향
    "torque_scale": 1.0,  # 전체 토크 배율
}

DEFAULT_MODE = "stand"
