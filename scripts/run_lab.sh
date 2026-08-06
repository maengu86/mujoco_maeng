#!/usr/bin/env bash
# Go2 입문 실험실 실행 (MuJoCo Viewer + 브라우저 HTML)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ ! -d .venv ]]; then
  echo "[안내] .venv 가 없습니다. 먼저 설치합니다."
  bash scripts/setup_env.sh
fi

# shellcheck disable=SC1091
source .venv/bin/activate

if [[ ! -f unitree_robots/go2/scene.xml ]]; then
  echo "[오류] unitree_robots/go2/scene.xml 이 없습니다. git pull 로 전체 저장소를 받으세요."
  exit 1
fi

echo "브라우저 UI: http://127.0.0.1:8765"
echo "종료: Ctrl+C (worker도 함께 종료)"

if [[ -x .venv/bin/mjpython ]]; then
  exec .venv/bin/mjpython go2_korean_control.py
else
  # Linux/Windows 또는 mjpython 미포함 환경
  exec python go2_korean_control.py
fi
