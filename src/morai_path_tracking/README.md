# morai_path_tracking

MORAI 경로 추종의 단일 프로젝트 제어 패키지다. 운영 횡방향 MPC 연결, 곡률·지도
속도 계획, 종방향 MPC, 안전 제한과 비교용 기존 제어기를 한 패키지에서 관리한다.

## 운영 데이터 흐름

```text
/local_path + /localization/odometry + /vehicle/competition_status
  -> scripts/control/morai_control_adapter.py
     -> /control/internal/waypoints, pose, twist, vehicle_status_estimated
  -> vendor mpc_follower                     -> lateral command
  -> path_tracking_controller_node
     (longitudinal_only=true)                -> longitudinal command
  -> adapter freshness/rate/angle checks     -> /control/actuator_command
```

`morai_control`이라는 별도 패키지는 없다. Autoware AI `mpc_follower`만 외부 코드의
ROS 패키지명과 라이선스를 보존하기 위해 `src/vendor/autoware_ai`에 둔다.

## 디렉터리

```text
config/controllers/
  autoware_mpc.yaml                 # 현재 횡방향 MPC 기본값
  molit_2026_path_tracking.yaml     # 속도 계획·종방향·legacy 설정
launch/control/autoware_mpc.launch  # 운영 진입점
scripts/control/                    # Autoware/MORAI 어댑터
src/controllers/longitudinal/       # 현재 종방향 MPC
src/controllers/lateral/            # 비교용 기존 자체 제어기
launch/legacy/                       # 기존 전체 스택 재현
```

기존 자체 Pure Pursuit, Stanley, IMM Hybrid와 custom MPC는 삭제하지 않았다. 초기
제어기와 현재 결과의 정량 기록을 재현하는 기준 코드이기 때문이다. 다만 운영 launch와
문서에서는 `legacy`로 명시해 현재 Autoware MPC와 혼동하지 않게 했다.

## 실행

```bash
source /opt/ros/noetic/setup.bash
source ~/molit-2026-stier1-suhyeon/install/setup.bash
roslaunch morai_path_tracking autoware_mpc.launch send_control:=false
```

`send_control:=true`는 `/control/actuator_command`를 MORAI로 실제 송신한다. 다른
sender가 없고 GPS/IMU/Competition Status가 fresh하며 차량이 초기 위치에 있는지
확인한 뒤 사용한다.

주요 인자:

| 인자 | 기본값 | 의미 |
| --- | ---: | --- |
| `send_control` | `false` | 실제 UDP 제어 송신 |
| `rviz` | `true` | HD map·경로·차량·LiDAR 시각화 |
| `speed_limit_kph` | `60` | 실행 중 속도 상한 |
| `maximum_input_speed_kph` | `62` | 어댑터 입력 안전 검사 상한 |
| `use_map_guard` | `false` | 검증 전 지도 경계 guard |
| `route_candidate_file` | map 패키지 기본 후보 | 보정된 RDDF/Lanelet2 경로 |

## 현재/기존 코드 구분

- 현재: `launch/control/autoware_mpc.launch`, `config/controllers/autoware_mpc.yaml`,
  `scripts/control`, 종방향 controller와 speed planner
- 기존 비교용: `launch/controllers/legacy_controller.launch`, `launch/legacy`,
  `src/controllers/lateral`

기존 자체 제어기 수식은 [legacy 이론 문서](docs/CONTROLLER_THEORY_AND_TUNING_KO.md)와
[custom MPC 기록](docs/LEGACY_CUSTOM_MPC_KO.md)에 남겼다. 현재 정량 결과는 워크스페이스
[검증 요약](../../docs/validation/README_KO.md)을 기준으로 한다.

## 테스트

```bash
cd ~/molit-2026-stier1-suhyeon
./build.sh
source install/setup.bash
catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results
```

빌드·단위시험은 차량 주행 성능을 증명하지 않는다. 주행 성능 주장은 동일 초기화,
동일 속도 상한과 완주 rosbag을 사용한 시뮬레이터 A/B 결과로만 한다.
