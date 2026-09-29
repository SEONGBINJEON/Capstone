# Isaac Sim 연결 계약

첫 버전은 실제 로봇의 목표 위치와 관측 위치를 구분해서 출력합니다.
목표 생산 → 실물 전송 → 같은 목표 벡터를 출력하는 순서이며,
UDP 수신 데이터가 실물 로봇에 명령을 넣는 경로는 없습니다.

`--jsonl`은 줄마다 JSON 하나를 저장합니다.
`--udp-port 8765`는 동일 데이터를 127.0.0.1:8765로 보내며, 별도 수신자 없어도 동작합니다.
UDP는 유실될 수 있고 실시간/동기화 보장은 없습니다. 실물 모터와 시뮬레이터의 실제 응답도 다를 수 있습니다.

## 목표 메시지: teleop.targets.v1

```json
{
  "schema": "teleop.targets.v1",
  "sequence": 12,
  "unix_time": 1789218000.0,
  "mode": "dry_run",
  "hardware_command_sent": false,
  "joints": [
    {
      "pair": "A",
      "joint": "joint_1",
      "leader_id": 16,
      "follower_id": 11,
      "target_ticks": 2048,
      "target_motor_offset_rad": 0.0,
      "actual_ticks": 2048,
      "saturated": false
    }
  ]
}
```

`joints`는 선택된 쌍당 5개입니다. 위는 한 항목만 보인 예시입니다.
`target_ticks`는 속도/범위 제한 후 실물 팔로워에 보내는 모터 위치값입니다.
`actual_ticks`는 같은 루프에서 전송 **전에** 관측한 실제 팔로워 위치입니다.
`target_motor_offset_rad`는 팔로워 기준점에서의 모터 축 각도 변화량이며 **URDF 관절 좌표가 아닙니다**.
회전 방향, 감속/링크 비율, URDF 영점과 축, 그리퍼의 회전→직선 변환을 확인해야 합니다.
`hardware_command_sent=true`는 SDK 송신이 성공했다는 뜻으로, 목표 도달을 뜻하지 않습니다.

중지 시 `teleop.stopped.v1`에 reason, frames, hold_errors, torque_left_on을 보냅니다.
마지막 목표를 무한정 재생하면 안 됩니다. 시뮬레이터 수신기는 마지막 메시지 수신 뒤
300 ms가 지나면 새 목표 적용을 멈추고 현재 시뮬레이션 자세를 유지하도록 구현합니다.
재시작 시 sequence가 0부터 시작하므로 새로운 세션으로 취급해야 합니다.

## 이후 Isaac Sim 구현에 필요한 정보

1. 팔로워 A/B의 URDF 또는 USD 파일과 실제 관절 이름.
2. 모터 ID → articulation 관절 매핑과 joint type(회전/직선/그리퍼).
3. 기준 자세의 URDF 관절 값, 방향과 감속비.
4. 사용 중인 Isaac Sim 버전과 실행 환경.

이 정보가 확인되면 수신 스레드는 최신 메시지만 저장하고,
Isaac Sim의 물리 스텝 콜백에서 선택한 A/B articulation에 관절 목표를 적용합니다.
메시지별로 관절 개수/ID/유한한 수/범위를 확인하고, 잘못되거나 오래된 메시지는 적용하지 않습니다.
물리 객체를 수신 스레드에서 직접 수정하지 않습니다.

팔로워 A만 먼저 추가해도 양쪽 실물 팔의 통신 코드는 바꿀 필요가 없습니다.
