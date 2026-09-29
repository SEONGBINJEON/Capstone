> **이전 세대 기록 (2026-09-13, 다른 PC, A/B 두 쌍 18개 모터).** 아래 경로 `/home/jeonseongbin/Teleoperation`과 `run_teleop.py` 명령은 당시 환경 기준이며 지금은 쓰지 않습니다. 현재 텔레오퍼레이션(리더 26~30 → 팔로워 11~15 + Isaac Sim)은 [sim_teleop/README.md](sim_teleop/README.md)를 보세요. 저장소 최상위 [README](../README.md)에 다른 PC에서 시작하는 절차가 있습니다.

# 텔레오퍼레이션 실행 안내

최종 정리: 2026-09-13. 프로젝트: `/home/jeonseongbin/Teleoperation`.

오늘 종료 상태: 활성 모터 18개 토크 OFF 확인 완료, 추종 및 통신 배경 서비스 종료. 다음 실행은 아래 명령으로 새 서비스를 시작합니다. 종료 시점 최종 백업은 `docs/checkpoints/2026-09-13-final/`입니다.

## A/B 함께 실행

리더 A/B를 편한 시작 자세에 두고 아래 두 줄을 터미널에 붙여 넣습니다.

```bash
cd /home/jeonseongbin/Teleoperation
.venv/bin/python run_teleop.py
```

어느 폴더에서든 한 줄로 실행하려면:

```bash
/home/jeonseongbin/Teleoperation/.venv/bin/python /home/jeonseongbin/Teleoperation/run_teleop.py
```

팔로워가 현재 리더 자세까지 최대 15°/s로 정렬한 뒤, 최대 180°/s로 따라갑니다. 리더를 일자로 세우고 실행할 필요는 없습니다. A는 그리퍼를 포함하고, B 그리퍼 25·30번은 제외합니다.

기존 배경 서비스가 있으면 재사용합니다. 실행 명령은 실제 정렬·추종 요청이므로 상태만 보고 싶을 때는 아래 `--status`를 사용합니다.

## 자주 쓰는 명령

아래 명령은 먼저 `cd /home/jeonseongbin/Teleoperation`을 실행한 터미널에서 사용합니다.

| 용도 | 명령 |
|---|---|
| A/B 동시 시작 | `.venv/bin/python run_teleop.py` |
| 상태만 확인 | `.venv/bin/python run_teleop.py --status` |
| 추종 정지, 자세 유지 | `.venv/bin/python run_teleop.py --hold` |
| A만 추종 | `.venv/bin/python run_teleop.py --pair A` |
| B만 추종 | `.venv/bin/python run_teleop.py --pair B` |
| 장애물 제거 후 B 수동 재개 | `.venv/bin/python run_teleop.py --resume B` |
| 장애물 제거 후 A 수동 재개 | `.venv/bin/python run_teleop.py --resume A` |
| 대기 중인 A/B 수동 재개 | `.venv/bin/python run_teleop.py --resume both` |

실행 화면에서 **Ctrl+C**를 누르면 추종을 멈추고 자세를 유지합니다. 토크와 USB 통신을 유지하는 배경 서비스는 남습니다. 터미널 창을 닫는 것만으로 추종 정지가 보장되지는 않습니다. 토크 해제나 전원 차단 전에는 팔을 받쳐야 합니다.

`--pair A/B`는 추종할 쌍을 고릅니다. 현재 서비스는 활성 모터 18개를 함께 읽으므로 나머지 보드나 모터를 뽑아도 되는 옵션은 아닙니다.

## 장애물에 막혔을 때

막힌 팔만 자세 유지로 전환하며 다른 팔의 추종과 리더 읽기는 계속됩니다.

- 막힐 때 밀던 관절의 리더를 팔로워 정지 각도보다 반대쪽으로 조금 되돌리면 자동 재개합니다. 여러 관절이 함께 밀고 있었다면 해당 관절들을 모두 되돌려야 합니다.
- 자동 재개는 바로 일반 추종 속도를 사용합니다. 처음 시작할 때의 느린 정렬을 반복하지 않습니다.
- 리더 목표가 여전히 장애물 너머에 있으면 대기를 유지합니다.
- 리더를 그대로 두고 장애물만 제거했다면 `--resume B` 또는 `--resume A`를 사용합니다. 수동 재개는 최대 15°/s로 다시 맞춘 뒤 추종합니다.
- 통신 장애·과열·모터 하드웨어 오류는 장애물 대기와 별도로 전체 추종을 중지시킵니다.

## 실행이 안 될 때

`bash: .venv/bin/python: No such file or directory`가 뜨면 현재 폴더가 프로젝트가 아닌지 확인합니다. 위의 폴더 이동 두 줄 또는 절대 경로 한 줄 명령을 사용합니다.

USB 포트나 허브 구멍을 바꾸면 `config/robot.json`의 장치 경로가 달라질 수 있습니다. 보드 전원 LED와 USB 인식, 모터 응답은 별개입니다. 오류 문구는 `--status`로 확인합니다.

## 상세 기록과 백업

- 상세 구현·보정·문제 해결 기록: `docs/implementation_record_2026-09-13.md`
- 실제 사용 설정: `config/robot.json`
- 사용자가 선택한 편한 리더 시작 자세 기록: `config/start_pose.json`
- 종료 시점의 코드·설정·상태 백업: `docs/checkpoints/2026-09-13-final/`
- 추종 중에 남겼던 이전 백업: `docs/checkpoints/2026-09-13/`

예전 시험 스크립트 대신 이 문서의 `run_teleop.py`를 사용합니다.
