# 키친모션 (Kitchen Motion) — 캡스톤 기록

소상공인의 반복 주방 작업을 위한 4자유도 양팔 로봇. 사람의 시연(리더-팔로워 텔레오퍼레이션)을 데이터로 모아 작업별 모방학습으로 이어가는 것이 목표입니다.
2026 YU 창의적 종합설계 경진대회 / 영남대학교 / 전성빈 · 이규홍 · 박종진

## 저장소 구조

| 폴더 | 내용 |
|---|---|
| `teleoperation/` | 리더-팔로워 텔레오퍼레이션 코드. 현재 사용하는 것은 `teleoperation/sim_teleop/` (리더 26~30 → 팔로워 11~15, Isaac Sim 연동). `teleoperation/teleop/`는 이전 세대 A/B 양팔 코드로, 모터 접근 계층(`hardware.py`)을 재사용합니다. |
| `isaac_sim/robot_model/` | 팔로워 URDF(Onshape 내보내기), Isaac Sim용 수정본 `robot_isaac.urdf`, 변환된 USD(`usd_isaac/`), STL 메시 |
| `media/renders/` | Isaac Sim 렌더 이미지 14장과 발표자료 제작용 프롬프트 |
| `docs/progress/` | 날짜별 진행 기록 |
| `docs/hardware-notes.md` | 모터 ID, 보드, 영구 설정 변경 이력 |
| `docs/study/` | 학습 가이드: 구조, 모터 통신, 제어 루프, Isaac Sim, 디버깅 기록, 면접 노트 |

## 빠른 실행 (이 PC 기준)

```bash
cd teleoperation
./run_sim_teleop.sh            # Isaac Sim 뷰어 + 텔레오퍼레이션
./stop_teleop.sh               # 추종 정지 (팔로워 자세 유지)
python3 sim_teleop/teleop.py --release   # 팔로워 토크 OFF (팔을 받친 뒤)
```

자세한 절차와 안전 동작은 [teleoperation/sim_teleop/README.md](teleoperation/sim_teleop/README.md).

## 다른 PC에서 이어서 시작하기

1. 클론 후 `teleoperation/` 폴더에서 작업합니다. 파이썬은 시스템 `python3`(3.10 이상)이면 되고, `dynamixel_sdk`·`pyserial`은 `teleoperation/vendor/`에 동봉되어 있어 설치가 필요 없습니다.
2. USB 시리얼 권한: `sudo usermod -aG dialout $USER` 후 재로그인. 로그아웃해도 반영이 안 되면 `sudo loginctl terminate-user $USER`(이 PC에서 겪은 문제, docs/progress 참고).
3. 포트: `sim_teleop/config.json`의 `buses`는 `/dev/serial/by-id/...` 경로라 같은 OpenCR·OpenRB-150 보드를 쓰면 그대로 동작합니다. 보드가 다르면 `ls /dev/serial/by-id/`로 바꿔 넣으세요. 읽기 전용 확인: `python3 sim_teleop/scan_motors.py`, `python3 sim_teleop/teleop.py --status`.
4. Isaac Sim: 6.0.1 기준. 설치 경로가 `~/isaacsim-6.0.1`이 아니면 `ISAAC=/path/to/isaacsim ./run_sim_teleop.sh`. 로봇 USD는 저장소의 `isaac_sim/robot_model/usd_isaac/`를 자동으로 찾고, 다른 위치면 환경변수 `ROBOT_USD`로 지정합니다.
5. 모터 EEPROM은 이 저장소의 팔로워 팔 기준으로 이미 변경되어 있습니다(`docs/hardware-notes.md`). 다른 팔을 쓰면 `scan_motors.py`로 모드·제한을 먼저 확인하세요.

## 현재 상태 (2026-09-28)

- 리더 → 팔로워 실시간 추종 동작 (그리퍼 포함), 180°/s 추종, 걸림·과부하 보호.
- Isaac Sim 6.0.1에서 팔로워 URDF 모델이 실물과 동시에 움직이는 디지털 트윈 동작.
- Isaac Sim 주방 환경(작업대, 인덕션, 미니 프라이팬 등)과 발표용 연출 렌더.
- 다음: 영상 동기 수집, 모방학습, 양팔 장면.

진행 기록: [docs/progress/](docs/progress/)
