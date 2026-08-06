#!/usr/bin/env bash
# MuJoCo / 파일 / mjpython 상태 점검
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "== 경로 =="
echo "ROOT=$ROOT"
echo

echo "== 필수 파일 =="
for f in go2_korean_control.py go2_sim_worker.py go2_control_panel_ko.html go2_defaults.py \
         unitree_robots/go2/scene.xml unitree_robots/go2/scene_terrain.xml unitree_robots/go2/go2.xml; do
  if [[ -f "$f" ]]; then echo "OK  $f"; else echo "MISSING  $f"; fi
done
echo

echo "== Python / mjpython =="
if [[ -d .venv ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
  echo "venv python: $(command -v python)"
  python - <<'PY'
import sys
print("version:", sys.version)
try:
    import mujoco
    print("mujoco:", mujoco.__version__)
except Exception as e:
    print("mujoco import FAIL:", e)
PY
  if [[ -x .venv/bin/mjpython ]]; then
    echo "mjpython: OK (.venv/bin/mjpython)"
  else
    echo "mjpython: MISSING — macOS 에서는 Viewer가 안 뜹니다"
  fi
else
  echo ".venv 없음 — bash scripts/setup_env.sh 실행 필요"
fi
echo

echo "== 최근 worker 로그 (있으면) =="
if [[ -f go2_worker.log ]]; then
  tail -n 40 go2_worker.log
else
  echo "(아직 go2_worker.log 없음)"
fi
