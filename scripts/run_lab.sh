#!/usr/bin/env bash
# Go2 한글 조작기 실행 — MuJoCo Viewer + 브라우저 HTML 동시 기동
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

if [[ ! -f go2_sim_worker.py || ! -f go2_korean_control.py || ! -f go2_control_panel_ko.html ]]; then
  echo "[오류] 필수 파일이 없습니다. 저장소 루트에서 실행하세요."
  exit 1
fi

echo "========================================================"
echo " Go2 한글 조작기"
echo " 브라우저: http://127.0.0.1:8765"
echo " 기록파일: $ROOT/go2_experiment_log.csv"
echo " worker로그: $ROOT/go2_worker.log"
echo " 종료: Ctrl+C"
echo "========================================================"

# macOS: mjpython 필수 (일반 python 이면 Viewer 창이 안 뜸)
if [[ "$(uname -s)" == "Darwin" ]]; then
  if [[ ! -x .venv/bin/mjpython ]]; then
    echo "[오류] .venv/bin/mjpython 이 없습니다."
    echo "  bash scripts/setup_env.sh 를 다시 실행하세요."
    exit 1
  fi
  exec .venv/bin/mjpython go2_korean_control.py
fi

if [[ -x .venv/bin/mjpython ]]; then
  exec .venv/bin/mjpython go2_korean_control.py
fi

exec python go2_korean_control.py
