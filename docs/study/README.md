# 학습 가이드 — 이 저장소로 무엇을 공부할 수 있는가

구현하면서 놓친 내용을 나중에 스스로 복기하기 위한 자료입니다. 코드를 다시 읽는 순서, 각 부분이 다루는 개념, 왜 그렇게 설계했는지, 면접에서 어떻게 설명할지를 정리했습니다.

| 문서 | 내용 |
|---|---|
| [01-architecture.md](01-architecture.md) | 전체 구조와 데이터 흐름. 코드를 읽는 순서 |
| [02-dynamixel-and-motor-control.md](02-dynamixel-and-motor-control.md) | DYNAMIXEL Protocol 2.0, 컨트롤 테이블, 위치 제어 모드, watchdog, 프로파일 |
| [03-teleop-control-loop.md](03-teleop-control-loop.md) | 리더→팔로워 추종 루프 설계: 보정, 속도 제한, 걸림 보호, 상태 기계 |
| [04-isaac-sim-digital-twin.md](04-isaac-sim-digital-twin.md) | URDF→USD, Isaac Sim 스탠드얼론 스크립트, UDP 연동, 장면 구성, 렌더 |
| [05-debugging-log.md](05-debugging-log.md) | 실제로 겪은 문제 10가지와 원인을 찾은 과정 (면접용 스토리) |
| [06-interview-notes.md](06-interview-notes.md) | 1분·3분 설명 스크립트, 예상 질문, 스스로 답해볼 문제 |

## 추천 순서

1. `01`을 읽고 `teleoperation/sim_teleop/common.py`(121줄)와 `config.json`을 연다. 설정이 어디로 흘러가는지만 본다.
2. `02`를 읽고 `teleoperation/teleop/hardware.py`의 `Rig.sample()`과 `Rig.write()`만 읽는다. 나머지는 나중에.
3. `03`을 읽고 `teleoperation/sim_teleop/teleop.py`의 `run()`을 위에서 아래로 한 번 읽는다. 300줄이지만 루프 하나다.
4. `04`를 읽고 `isaac_view.py`를 읽는다. Isaac Sim이 없어도 구조는 이해된다.
5. `05`, `06`은 취업 준비 직전에 다시 본다.

## 이 프로젝트에서 얻어 갈 수 있는 키워드

로봇 원격조작(leader-follower teleoperation), 서보 통신 프로토콜(Protocol 2.0, sync read/write), 안전 설계(watchdog, 속도 제한, stall 감지, 과부하 복구), 보정(zero pose, 방향 부호, tick↔각도 변환), 프로세스 간 통신(UDP JSON), 디지털 트윈(URDF, USD, PhysX articulation), 시뮬레이션 기반 자세 탐색(샘플링 IK), 문제 해결 기록.
