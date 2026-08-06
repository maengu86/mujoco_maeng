#!/usr/bin/env bash
# 어느 컴퓨터에서든 가상환경 + MuJoCo 의존성을 한 번에 설치합니다.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  else
    echo "[오류] python3 가 없습니다. Python 3.10+ 를 설치하세요."
    exit 1
  fi
fi

echo "[1/3] Python: $($PYTHON_BIN --version)"
echo "[2/3] 가상환경 .venv 생성/갱신"
"$PYTHON_BIN" -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo "[3/3] 장면 파일 확인"
test -f unitree_robots/go2/scene.xml
test -f unitree_robots/go2/scene_terrain.xml
test -f unitree_robots/go2/go2.xml

echo
echo "설치 완료."
echo "실행:"
echo "  source .venv/bin/activate"
if [[ -x .venv/bin/mjpython ]]; then
  echo "  ./.venv/bin/mjpython go2_korean_control.py"
else
  echo "  python go2_korean_control.py"
  echo "  (macOS 에서는 mjpython 이 권장됩니다. mujoco 설치 후 .venv/bin/mjpython 확인)"
fi
