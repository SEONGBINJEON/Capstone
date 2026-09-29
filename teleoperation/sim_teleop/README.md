# 리더 26~30 → 팔로워 11~15 텔레오퍼레이션 + Isaac Sim 디지털 트윈

작성: 2026-09-28. 이 PC 기준.

| 역할 | 보드 | 모터 ID | 포트 |
|---|---|---|---|
| 팔로워 | OpenCR | 11 12 13 14 15(그리퍼) | `/dev/serial/by-id/usb-ROBOTIS_OpenCR_…` |
| 리더 | OpenRB-150 | 26 27 28 29 30(그리퍼) | `/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_…` |

매핑: 26→11, 27→12, 28→13, 29→14, 30→15 (그리퍼 포함). 설정은 `sim_teleop/config.json`.

## 파일

| 파일 | 역할 |
|---|---|
| `teleop.py` | 하드웨어 추종 루프. 팔로워 관절 각도를 UDP(127.0.0.1:5005)로 발행 |
| `isaac_view.py` | Isaac Sim 6.0.1 뷰어. 저장소의 `isaac_sim/robot_model/usd_isaac/robot_isaac/robot_isaac.usda`(자동 탐색, 환경변수 `ROBOT_USD`로 변경 가능) (원본 `robot.urdf`의 메시 경로 `package://assets\...`를 `assets/...`로 고친 `robot_isaac.urdf`를 Isaac 임포터로 변환한 것)를 열고 UDP 각도로 관절을 움직임 |
| `calibrate.py` | 읽기 전용 보정 도구(방향, 기준 자세, 그리퍼 개폐). 모터에는 쓰지 않음 |
| `scan_motors.py` | 읽기 전용 버스 스캔 |
| `common.py` | 설정/Rig/각도 변환 공용 코드 (`teleop/hardware.py`의 Rig 재사용) |
| `scene_lib.py` | 주방 장면·미니 프라이팬·재질 생성 (뷰어와 렌더러 공용) |
| `render_gallery.py` | 발표용 연출 렌더 (100대 병렬, 3×3 주방, 파지 시퀀스) |
| `reboot_motor.py` | 과부하 등 래치된 하드웨어 오류가 난 모터 재부팅 |
| `set_position_limits.py` | EEPROM 위치 제한 변경 (토크 OFF, 이전 값 자동 백업) |
| `wait_in_range.py` | 팔로워를 손으로 옮길 때 모터 범위 안에 들어올 때까지 읽기 전용 대기 |
| `probe_usd.py` | USD를 헤드리스로 열어 메시 수·바운딩 박스 확인, 스크린샷 |
| `offline_loop_test.py` | 모터 없이 가짜 Rig로 제어 루프를 6초 돌려보는 오프라인 테스트 |
| `../run_sim_teleop.sh` | 뷰어 + 추종 동시 실행 |

파이썬은 시스템 `python3`(3.10)를 쓰고, `dynamixel_sdk`/`pyserial`은 `vendor/`에 동봉되어 있습니다. 뷰어는 Isaac Sim의 `python.sh`로 실행합니다.

## 처음 한 번: 보정 (약 5분, 모터에 쓰지 않음)

모두 저장소의 `teleoperation/` 폴더에서 실행. 팔로워 토크는 꺼져 있어야 손으로 움직일 수 있습니다.

1. 회전 방향. 관절마다 리더와 팔로워를 손으로 **같은 물리적 방향**으로 20° 이상 돌리면 부호를 자동 판정해 저장합니다.
   ```bash
   python3 sim_teleop/calibrate.py direction
   ```
2. 그리퍼 개폐 위치. 리더·팔로워 그리퍼를 닫힘/열림으로 두고 Enter.
   ```bash
   python3 sim_teleop/calibrate.py gripper
   ```
3. (선택) 기준 자세. 두 팔을 같은 자세(예: 일자로 세움)로 두고 저장하면 `--match` 없이도 시작할 수 있습니다.
   ```bash
   python3 sim_teleop/calibrate.py zero
   ```

기존 `config/robot.json`의 리더 B 기준점과 팔로워 A 기준점을 그대로 조합하면 시작 정렬량이 200° 이상 나와 사용할 수 없었습니다. `--match`는 두 팔을 같은 자세로 맞춘 뒤 상대 추종할 때만 쓰고, 기본은 절대 기준(`./run_sim_teleop.sh`)입니다. 리더 26 기준점을 2060으로 고친 뒤로는 절대 기준이 맞습니다.

## 실행

1. (해결됨) 팔로워 3번 관절(ID 13)의 EEPROM 범위는 0~4095로 넓혔습니다. 다른 관절이 범위 밖이면 시작 시 안내가 나옵니다.
2. 리더를 팔로워와 **같은 자세**로 맞춘 뒤:
   ```bash
   ./run_sim_teleop.sh --match
   ```
   Isaac Sim 창이 30~60초 뒤에 뜨고, 터미널에서는 팔로워 토크 ON → 추종이 시작됩니다.
3. 종료는 **Ctrl+C**. 팔로워는 현재 자세를 토크 ON으로 유지합니다.
4. 토크를 끌 때는 팔을 받친 상태에서:
   ```bash
   python3 sim_teleop/teleop.py --release
   ```

읽기만 하려면 `python3 sim_teleop/teleop.py --status`, 모터에 쓰지 않고 시작 정렬량만 보려면 `--dry-run --match`.

뷰어만 따로 띄우려면:
```bash
~/isaacsim-6.0.1/python.sh sim_teleop/isaac_view.py
```

## 안전 동작

- 리더 토크가 켜져 있으면 시작하지 않음.
- 시작 정렬 15°/s, 이후 추종 180°/s (`config.json`의 `align_speed_deg_s`, `follow_speed_deg_s`).
- 추종 오차 12° 초과(장애물 등) → 현재 자세 유지로 전환. 리더를 팔로워 자세로 되돌리면(차이 3° 이내) 저속 정렬 후 자동 재개.
- 리더 한 주기 30° 이상 급변, 온도 60°C, 통신 지연 150 ms, 하드웨어 오류 → 안전 정지(자세 유지).
- 팔로워 통신 watchdog 300 ms: 프로세스가 죽으면 모터가 스스로 정지.
- 리더 그리퍼 ID 30은 OpenRB 4.5 V 전원 때문에 전압 경고 비트가 항상 켜져 있어 예외 처리했습니다(읽기 전용이므로 문제 없음).
- 팔로워 베이스 ID 11은 기존 설정대로 확장 위치 모드의 소프트웨어 범위 548~1572(기준 1060, ±45°)로 제한됩니다. 더 넓히려면 `config.json`의 `extended_position`을 수정하세요.

## Isaac Sim 장면·스크린샷

- 기본 장면은 주방(`--scene kitchen`): 작업대, 하부 수납장, 타일 벽, 선반, 인덕션, 도마, 수건, 머그·향신료·접시·칼 블록, 그리고 **미니 프라이팬**(지름 90 mm, 손잡이 85 mm, 강체) 2개. 로봇 앞의 팬은 물리 강체라 밀거나 잡을 수 있습니다.
- `--scene plain`은 격자 바닥만.
- `--props`를 붙이면 Isaac 에셋 서버에서 머그·그릇 등을 추가로 내려받지만, 텍스처 로딩 문제로 화면이 검게 나올 수 있어 기본은 꺼져 있습니다.
- 뷰어가 실행 중일 때 스크린샷: 다른 터미널에서 `touch work/capture_request` → `work/shots/shot_<시각>.png` (1920×1080).
- 특정 자세를 헤드리스로 렌더: `python.sh sim_teleop/isaac_view.py --headless --scene kitchen --pose '{"joint_22":23,"joint_33":-117,"joint_44":-85}' --capture out.png`
- 핀레이 그리퍼 핑거(follower_07)는 파란색(USD 재질 수정 + 뷰어에서 재바인딩).
- URDF 0° 자세가 바닥에 누운 방향이어서 뷰어가 루트를 X축 90° 회전시켜 세웁니다(`--root-rotate-x`).

## 발표용 연출 이미지 (render_gallery.py)

실제 학습이 아닌 **연출 렌더**입니다. 결과는 `work/shots/gallery/`에 생성됩니다. 발표에 쓴 최종 14장은 저장소 `media/renders/`에 있습니다.
```bash
~/isaacsim-6.0.1/python.sh sim_teleop/render_gallery.py            # 전체
~/isaacsim-6.0.1/python.sh sim_teleop/render_gallery.py --only hero # 일부: hero, parallel_arms, parallel_kitchens
```
- `parallel_arms_100(.png / _low.png)`: 팔로워 100대 10×10 격자, 자연스러운 자세 무작위.
- `parallel_kitchens_3x3(.png / _low.png)`: 주방 환경 9개 병렬.
- `hero_*`, `gripper_closeup_pan`, `grasp_sequence_01~03`, `top_view_workspace`: 단일 주방.
- 파지 자세(접근·파지·들기)는 시뮬레이션에서 그리퍼 위치를 읽어 자동 탐색하며 `poses.json`에 저장됩니다.

## Isaac Sim 매핑 조정

`config.json`의 각 관절 `sim` 항목:

- `joint`: USD 관절 이름 (`joint_11`, `joint_22`, `joint_33`, `joint_44`, 그리퍼는 `joint_55`)
- `sign`: 화면 회전 방향이 실물과 반대면 -1 (teleop.py 쪽에서 적용, 재시작 필요)
- `view_sign`: 같은 역할이지만 뷰어 쪽에서 적용되어 뷰어만 재시작하면 됨 (joint_1은 -1로 설정됨)
- `offset_deg`: 화면 자세가 실물과 일정 각도 어긋나면 보정
- `zero_ticks`: URDF 0° 자세일 때 팔로워 tick (기본값은 `follower_zero`, 즉 일자로 세운 자세 기준)
- 그리퍼 `closed_deg`/`open_deg`: joint_55 각도 범위. URDF의 프리즘 관절(joint_66/77)은 상하한이 같아 움직이지 않으므로 회전 관절 joint_55로 표시합니다.

`sim_source`를 `target`으로 바꾸면 실제 팔로워 위치 대신 명령 목표를 화면에 표시합니다(지연 감소).

## 작업 기록 2026-09-28 (첫 세션)

- 환경: 이 PC. 팔로워 11~15 OpenCR(`/dev/ttyACM0`), 리더 26~30 OpenRB-150(`/dev/ttyACM1`). `dialout` 그룹 추가 후 `systemd --user`가 로그아웃에도 살아 있어 반영이 안 됐고 `loginctl terminate-user`로 해결.
- SDK: 시스템 python3(3.10)에 pip 없음 → `vendor/`에 dynamixel_sdk 4.0.5, pyserial 3.5 동봉.
- 모터 설정 변경(영구):
  - ID 13 EEPROM Min/Max Position Limit 1024/3140 → 0/4095. 백업 `work/eeprom_limit_backup_13_*.json`. 이유: 접힌 자세(약 600)가 범위 밖. 같은 설계의 B 팔로워 23은 원래 0~4095.
- 설정(`config.json`):
  - 리더 26 기준점 12 → 2060 (리더 베이스 기준이 180° 틀어져 있었음).
  - ID 11 확장 위치 소프트웨어 범위 548~1572(±45°) → 기준 2048, raw_reference 1060, 1~4095(±179°). follower_zero 1060 → 2048. 베이스 케이블 감김 여부는 미확인.
  - 시뮬레이션: joint_1 `view_sign=-1` (화면 회전 방향 반대였음).
  - 안전: max_tracking_error 12→20°, max_lead 10°, 걸림 보호(5° 이상 차이 + 1.4초 정지 → 자세 유지, 리더 되돌리면 재개), 대기 후 재개는 즉시 180°/s.
- 오늘 발생한 문제: ID 13 과부하(0x20)로 모터 토크 자동 OFF → 안전 정지. 3번 관절이 기구 한계(리더 목표 ~456 부근)에 걸린 채 밀었기 때문. `reboot_motor.py 13`으로 복구. 이후 걸림 보호 추가.
- Isaac Sim: 원본 URDF 메시 경로 `package://assets\...`(윈도우 역슬래시) 때문에 USD에 메시가 없었음 → `robot_isaac.urdf`로 고쳐 재변환(`usd_isaac/robot_isaac/`). 로봇이 z<0에 있어 바닥판에 가렸던 것도 수정. 핀레이 핑거 파란색. 주방 장면·미니 프라이팬·연출 렌더 14장(`work/shots/final/`, zip과 PPT 프롬프트 포함).
- 종료 상태: 추종 정지 후 `--release`로 팔로워 토크 OFF, 뷰어 종료.
- 다음에 할 만한 것: 리더 3번 관절 가동 범위를 팔로워 기구 한계에 맞춰 제한, 그리퍼 `calibrate.py gripper`로 리더 열림 위치 실측(현재 1625 추정), 베이스 큰 회전 시 케이블 확인, 양팔 장면 렌더.
