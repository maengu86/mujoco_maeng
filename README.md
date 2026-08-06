# Go2 강화학습 입문 실험실

Unitree Go2 + MuJoCo 기반 **교육용 한글 조작기**입니다.  
이 저장소만 클론하면 **Go2 모델/장면 + 실행 스크립트**까지 함께 받습니다.

브라우저에서 동작·파라미터·지형 전환·실험 마법사·기록 분석을 할 수 있습니다.

## 포함 내용

```text
go2_korean_control.py        # 브라우저 UI / 파라미터 / 로그 / worker 관리
go2_sim_worker.py            # MuJoCo Viewer (프로세스당 1개)
go2_control_panel_ko.html    # 교육용 밝은 테마 UI
unitree_robots/go2/          # Go2 모델 + scene.xml + scene_terrain.xml + meshes
requirements.txt             # mujoco, numpy
scripts/setup_env.sh         # 가상환경 설치
scripts/run_lab.sh           # 실행
```

## 새 컴퓨터에서 시작 (복붙)

```bash
# 1) 저장소 받기
git clone https://github.com/maengu86/mujoco_maeng.git
cd mujoco_maeng

# 2) (작업 브랜치가 있다면)
git checkout cursor/go2-korean-lab-worker-39ff
git pull

# 3) 환경 설치
bash scripts/setup_env.sh

# 4) 점검 (선택)
bash scripts/check_env.sh

# 5) 실행 (MuJoCo Viewer + HTML 패널 동시 오픈)
bash scripts/run_lab.sh
```

브라우저: http://127.0.0.1:8765  
종료: `Ctrl+C` (MuJoCo worker도 함께 종료)

**중요 (macOS):** 반드시 `mjpython` 으로 실행하세요.  
일반 `python go2_korean_control.py` 를 쓰면 HTML만 뜨고 MuJoCo 창이 안 열릴 수 있습니다.

### macOS Apple Silicon 수동 실행

```bash
cd mujoco_maeng
source .venv/bin/activate
./.venv/bin/mjpython go2_korean_control.py
```

### Linux

```bash
cd mujoco_maeng
source .venv/bin/activate
python go2_korean_control.py
```

### HTML만 뜨고 MuJoCo가 안 뜰 때

```bash
bash scripts/check_env.sh
tail -n 80 go2_worker.log
# 그다음 반드시 mjpython 으로 재실행
bash scripts/run_lab.sh
```

## 구조 설명

macOS + `mjpython`에서는 같은 Python 프로세스에서 `launch_passive()`를
두 번 호출하면 `another MuJoCo viewer is already open` 오류가 납니다.

그래서 Viewer는 **worker 프로세스 하나당 하나만** 실행하고, 지형 전환 시에는

1. 현재 worker 종료 요청  
2. Viewer 종료  
3. worker join/wait  
4. 새 worker 시작  
5. 장면 선택 (`scene.xml` / `scene_terrain.xml`)  
6. 새 Viewer 실행  

순서로 동작합니다. 브라우저 UI와 파라미터는 유지되며, 시작 시 home 자세로 초기화됩니다.

## 기능

- 동작: 서기, 왼앞발 들기, 제자리 트로트, 앞으로 걷기, 모터 끄기, home, 일시정지
- 파라미터: Kp, Kd, 보행 주기, 발 들기, 스윙, 전진 편향, 토크 배율 (호버 설명 카드)
- 장면: 평지 / 지형 장면 (전환 중 버튼 비활성화)
- 실험 마법사: 서기 → … → 계단 순서 가이드
- 기록: `go2_experiment_log.csv` (메모 포함)
- 자동 분석: Kp/Kd와 넘어짐 패턴 요약

## 개발/테스트 (MuJoCo 없이)

```bash
GO2_MOCK=1 python3 go2_korean_control.py
# 또는
python3 scripts/test_worker_switch.py
```

## 라이선스 고지

Go2 모델/메시는 Unitree `unitree_mujoco` 원본을 포함합니다. 자세한 내용은 `THIRD_PARTY.md` 참고.
