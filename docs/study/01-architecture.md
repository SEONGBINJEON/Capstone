# 01. 전체 구조와 데이터 흐름

## 한 장 그림

```
 사람 손 → 리더 암(XL330 ×4 + XC430, 토크 OFF, 위치만 읽음)
              │  USB (OpenRB-150, 1 Mbps, Protocol 2.0)
              ▼
     sim_teleop/teleop.py  ── 20 Hz 제어 루프 ──►  팔로워 암(XL430/XM430/XC430, 토크 ON)
              │            (목표 위치 sync write)     USB (OpenCR)
              │
              │ UDP 127.0.0.1:5005  {"joints": {"joint_11": {"rad": ...}, ...}}
              ▼
     sim_teleop/isaac_view.py  (Isaac Sim 6.0.1 스탠드얼론)
        └ robot_isaac.usda 로드 → Articulation 관절 각도 설정 → 렌더
```

두 개의 프로세스가 완전히 분리되어 있습니다. **로봇을 움직이는 프로세스는 절대 시뮬레이터에 의존하지 않습니다.** 시뮬레이터가 죽거나 느려도 추종은 계속됩니다. UDP는 상대가 없어도 보내는 쪽이 막히지 않는 프로토콜이라 이 용도에 맞습니다.

## 파일 지도 (`teleoperation/`)

| 파일 | 역할 | 크기 |
|---|---|---|
| `sim_teleop/config.json` | 유일한 설정: 포트, 관절 매핑(26→11 …), 기준점(zero), 방향, 그리퍼 개폐값, 안전 파라미터, 시뮬 매핑 | |
| `sim_teleop/common.py` | 설정 로딩, Rig 생성, tick↔각도 변환, 목표 계산(`joint_target`), 건강 검사(`health`) | 121줄 |
| `sim_teleop/teleop.py` | 제어 루프(`run`), 상태 조회(`--status`), 토크 해제(`--release`), UDP 발행 | 300줄 |
| `teleop/hardware.py` | 이전 세대에서 가져온 모터 접근 계층 `Rig`. 포트 열기, 모델 확인, sync read, 안전한 write, 확장 위치 모드 처리 | 421줄 |
| `teleop/config.py` | `SafetyError`, `TICKS_PER_DEG`(4096/360), 모델 번호표 | |
| `sim_teleop/isaac_view.py` | Isaac Sim 뷰어. USD 로드, 장면, UDP 수신, 관절 적용, 스크린샷 | 206줄 |
| `sim_teleop/scene_lib.py` | 주방 장면 빌더(재질, 상자, 원기둥, 미니 프라이팬, 주방 전체) | 196줄 |
| `sim_teleop/render_gallery.py` | 발표용 렌더: 100대 병렬, 3×3 주방, 자세 자동 탐색, 파지 시퀀스 | 335줄 |
| `sim_teleop/calibrate.py` | 읽기 전용 보정(방향, 기준 자세, 그리퍼 개폐) | |
| `sim_teleop/scan_motors.py`, `reboot_motor.py`, `set_position_limits.py` | 진단·복구·EEPROM 유지보수 도구 | |
| `vendor/` | dynamixel_sdk 4.0.5, pyserial 3.5 (pip 없는 PC용 동봉) | |

## 왜 이렇게 나눴나

- **설정과 코드 분리**: 모터 ID, 기준점, 방향 부호처럼 "이 로봇에만 해당하는 숫자"는 전부 `config.json`. 다른 팔에 붙이려면 JSON만 바꿉니다.
- **하드웨어 접근 계층(`Rig`) 재사용**: 시리얼 포트 배타적 열기, 모델 번호 검증, 확장 위치 모드의 다회전 좌표 처리, "허용된 주소만 쓰기"처럼 실수하면 로봇이 망가지는 부분은 이미 검증된 코드를 그대로 씁니다. 새로 쓴 것은 그 위의 정책(어떤 속도로, 언제 멈추고, 언제 재개하나)입니다.
- **제어 루프는 한 함수**: `run()` 하나에 준비 → 토크 ON → 루프 → 종료 처리가 순서대로 있습니다. 상태 기계가 작을 때는 클래스로 쪼개는 것보다 위에서 아래로 읽히는 게 디버깅에 유리합니다.
- **시뮬은 소비자**: 뷰어는 각도를 받아 그리기만 합니다. 뷰어가 로봇에 명령을 보내는 경로는 없습니다(사고 방지).

## 데이터가 변환되는 지점 (여기서 버그가 납니다)

1. **모터 tick(0~4095, 1 tick = 360/4096°)** ← 리더에서 읽음
2. `delta_ticks(pos, zero)`: 기준점 대비 편차, 4096에서 감기는 것을 고려해 ±2048 범위로 정규화
3. `joint_target()`: 팔로워 목표 tick = follower_zero + 편차 × direction × scale, 범위로 클램프
4. 슬루 제한: 한 주기에 speed×dt 이상 못 움직이게 목표를 잘라냄(`commanded`)
5. `Rig.stream_goals()`: 프로파일(가속·속도)과 목표를 sync write
6. `follower_to_sim()`: 팔로워 tick → (tick − zero)/TICKS_PER_DEG × sign + offset → 라디안 → UDP
7. 뷰어: 라디안 × view_sign → `Articulation.set_joint_positions`

각 단계의 부호와 기준점을 맞추는 일이 "보정"이고, 이번에 시간을 가장 많이 쓴 곳입니다.
