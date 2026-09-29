# 04. Isaac Sim 디지털 트윈

## 개념 정리

- **URDF**: 링크(형상·질량)와 관절(축·범위·부모/자식)을 XML로 기술한 로봇 모델. Onshape 내보내기로 생성됨.
- **USD**(Universal Scene Description): NVIDIA Omniverse/Isaac Sim의 장면 포맷. 프림(prim) 트리, 레이어 합성(reference/payload), 스키마(UsdGeom, UsdPhysics, UsdShade, UsdLux).
- **Articulation**: PhysX가 관절로 연결된 강체 묶음을 하나로 푸는 단위. `isaacsim.core.prims.Articulation`으로 관절 각도를 읽고 씁니다.
- **스탠드얼론 스크립트**: GUI 확장 대신 `python.sh script.py`로 `SimulationApp`을 띄우고 루프를 직접 돌리는 방식. 자동화·헤드리스 렌더에 유리.

## 파이프라인

```
robot.urdf (Onshape) ──메시 경로 수정──► robot_isaac.urdf ──Isaac URDF Importer──► usd_isaac/robot_isaac/robot_isaac.usda
                                                                                      ├ payloads/base.usda      (링크 트리)
                                                                                      ├ payloads/geometries.usd (메시, 1.7 MB)
                                                                                      ├ payloads/instances.usda (메시 인스턴스, 재질 바인딩)
                                                                                      ├ payloads/materials.usda (UsdPreviewSurface 재질)
                                                                                      └ payloads/Physics/physx.usda (관절, 드라이브, 제한)
```

변환 명령(저장소 `isaac_sim/robot_model`에서):
```bash
~/isaacsim-6.0.1/python.sh ~/isaacsim-6.0.1/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py \
  --urdf robot_isaac.urdf --usd-path usd_isaac --fix-base --no-merge-fixed-joints
```

## 뷰어(`isaac_view.py`)가 하는 일, 순서대로

1. `SimulationApp({'headless': ..., 'width': 1920, 'height': 1080})` — 이 줄 전에 `omni`/`pxr`를 import하면 안 됩니다.
2. `omni.usd.get_context().open_stage(usd)` 후 `app.update()` 몇 번 — 로딩이 비동기라서.
3. 루트 프림에 X축 90° 회전 — URDF 0° 자세가 팔이 눕고 베이스가 세워진 방향이었기 때문.
4. `UsdGeom.BBoxCache`로 로봇의 월드 바운딩 박스 계산 → 바닥판을 로봇 최하점 아래에 깔고, 카메라를 로봇 중심으로. (처음엔 로봇이 z<0에 있어 바닥 아래에 묻혀 있었음)
5. 장면 구성(`scene_lib.build_kitchen`) — 프림을 코드로 생성: `UsdGeom.Cube/Cylinder`, `UsdShade.Material`+`UsdPreviewSurface`, `UsdPhysics.CollisionAPI/RigidBodyAPI`.
6. `World().reset()` → 물리 초기화. 그 다음에야 `Articulation(...).initialize()`가 됩니다.
7. `set_gains(kp=1e6, kd=1e4)`, `switch_control_mode('position')` — 사실상 키네마틱하게 따라가게 강성 드라이브.
8. UDP 수신 스레드가 최신 패킷만 보관. 메인 루프는 매 프레임 `set_joint_positions`(순간이동) + `set_joint_position_targets` + `world.step(render=True)`.
9. `work/capture_request` 파일이 생기면 `capture_viewport_to_file`로 스크린샷.

## 실물 → 화면 각도 변환

- 실물 tick → `(tick − zero_ticks)/TICKS_PER_DEG × sign + offset_deg` → 라디안 (teleop.py 쪽)
- 뷰어에서 `view_sign` 곱함 (뷰어만 재시작해 부호를 고칠 수 있게 분리)
- URDF 관절 이름 매핑: joint_11(베이스) … joint_44(손목), 그리퍼는 joint_55. URDF의 프리즘 관절(joint_66/77)은 상하한이 같아 움직이지 않으므로 회전 관절 하나로 개폐를 표시.

## 렌더 파이프라인(`render_gallery.py`)에서 배울 것

- **자세 자동 탐색(샘플링 IK)**: 해석적 IK 없이, 관절 4개의 격자 조합(약 3만 개)을 물리 스텝마다 넣고 그리퍼 링크의 월드 위치·방향(`RigidPrim.get_world_poses`)을 읽어 목표(프라이팬 손잡이 위 x cm, 아래를 향함)에 가장 가까운 조합을 고릅니다. 1분 정도 걸리지만 로봇 기구를 몰라도 되고, 부호 실수를 자동으로 피해 갑니다.
  - 처음 실패한 이유: 3번 관절 탐색 범위를 음수로만 잡았는데 앞으로 접히는 방향이 양수였음. FK 디버그 스크립트로 각 관절을 하나씩 돌려 링크 위치를 찍어 보고 알아냈습니다.
  - 두 번째 실패: 탐색 중 팔이 프라이팬(강체)을 쳐서 밀어냈음 → 렌더용 팬은 고정체로.
- **N개 로봇 인스턴스**: 같은 USD를 `add_reference_to_stage`로 100번 참조하고 `Articulation('/World/Arms/Arm_.*')` 정규식 뷰 하나로 (100, 7) 배열을 한 번에 씁니다. 병렬 강화학습 환경이 실제로 이렇게 구성됩니다(Isaac Lab의 env cloning).
- **재질**: `UsdPreviewSurface`의 diffuseColor/roughness/metallic/opacity만으로 도자기·유리·금속 느낌을 냄. 네트워크 에셋(텍스처 MDL)은 헤드리스 캡처에서 검게 나오는 문제가 있어 기본 비활성.

## 스스로 답해볼 문제

1. `open_stage` 직후 바운딩 박스가 0으로 나오면 무엇을 의심해야 하나? (비동기 로딩, 인스턴스 프록시 순회)
2. 뷰어가 죽어도 로봇이 계속 움직이는 이유를 프로세스/프로토콜 관점에서 설명해 보라.
3. 시뮬레이션 관절을 `set_joint_positions`로 매 프레임 순간이동시키는 방식과 드라이브 목표만 주는 방식의 차이는? 어느 쪽이 "디지털 트윈 표시"에 맞는가?
4. URDF의 `package://` 경로가 문제였던 이유와, ROS 환경에서는 왜 문제가 되지 않는지.
