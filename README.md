# Go2 강화학습 입문 실험실

Unitree Go2 + MuJoCo 기반의 **교육용 한글 조작기**입니다.  
브라우저에서 동작·파라미터·지형 전환·실험 마법사·기록 분석을 할 수 있습니다.

## 구조

```
go2_korean_control.py      # 브라우저 UI / 파라미터 / 로그 / worker 관리
go2_sim_worker.py          # MuJoCo Viewer (프로세스당 1개)
go2_control_panel_ko.html  # 교육용 밝은 테마 UI
unitree_robots/go2/
  scene.xml
  scene_terrain.xml
go2_experiment_log.csv     # 실험 기록 (실행 시 생성)
```

**중요:** macOS + `mjpython`에서는 같은 Python 프로세스에서 `launch_passive()`를
두 번 호출하면 `another MuJoCo viewer is already open` 오류가 납니다.  
그래서 Viewer는 **worker 프로세스 하나당 하나만** 실행하고, 지형 전환 시에는

1. 현재 worker 종료 요청  
2. Viewer 종료  
3. worker join/wait  
4. 새 worker 시작  
5. 장면 선택 (`scene.xml` / `scene_terrain.xml`)  
6. 새 Viewer 실행  

순서로 동작합니다. 브라우저 UI와 파라미터는 유지되며, 시작 시 home 자세로 초기화됩니다.

## 실행 (macOS Apple Silicon)

`unitree_mujoco` 루트에 이 파일들을 두고:

```bash
cd ~/go2-study/unitree_mujoco
source .venv/bin/activate
./.venv/bin/mjpython go2_korean_control.py
```

브라우저: http://127.0.0.1:8765

## 기능

- 동작: 서기, 왼앞발 들기, 제자리 트로트, 앞으로 걷기, 모터 끄기, home, 일시정지
- 파라미터: Kp, Kd, 보행 주기, 발 들기, 스윙, 전진 편향, 토크 배율 (호버 설명 카드)
- 장면: 평지 / 지형 장면 (전환 중 버튼 비활성화)
- 실험 마법사: 서기 → … → 계단 순서 가이드
- 기록: `go2_experiment_log.csv` (메모 포함)
- 자동 분석: Kp/Kd와 넘어짐 패턴 요약
- Ctrl+C 시 worker까지 정리 (좀비 방지)

## 개발/테스트용

MuJoCo 없이 worker 생명주기만 검증:

```bash
GO2_MOCK=1 python3 go2_korean_control.py
# 또는
python3 scripts/test_worker_switch.py
```
