# MORAI UDP wire 브리지

`morai_udp_bridge`는 MORAI가 UDP로 보내는 카메라, GPS와 IMU 패킷을 ROS1
센서 메시지로 변환하고, 자율 주행 제어 명령을 MORAI UDP 제어 packet으로
전송한다. 이 패키지는 MORAI wire packet 규격에 직접 종속된다.

차량 제어, GPS map 투영, 경로 생성, 센서 장착 TF와 LiDAR packet decoding은
담당하지 않는다. MORAI native VLP-16 데이터는 `morai_bringup`이 공식 ROS
`velodyne` driver와 연결한다.

## 처리하는 스트림

| 센서 | UDP 입력 | ROS 출력 | 타입 |
| --- | --- | --- | --- |
| 전방 카메라 | `9291` | `/image/front/compressed` | `sensor_msgs/CompressedImage` |
| 전방 카메라 정보 | 위와 동일 | `/image/front/camera_info` | `sensor_msgs/CameraInfo` |
| 좌측 카메라 | `9293` | `/image/left/compressed` | `sensor_msgs/CompressedImage` |
| 좌측 카메라 정보 | 위와 동일 | `/image/left/camera_info` | `sensor_msgs/CameraInfo` |
| 우측 카메라 | `9295` | `/image/right/compressed` | `sensor_msgs/CompressedImage` |
| 우측 카메라 정보 | 위와 동일 | `/image/right/camera_info` | `sensor_msgs/CameraInfo` |
| GPS | `9301` | `/sensors/gps/fix` | `sensor_msgs/NavSatFix` |
| IMU | `9303` | `/sensors/imu/data` | `sensor_msgs/Imu` |
| Competition Vehicle Status | `9094` | `/vehicle/competition_status` | `morai_udp_bridge/CompetitionVehicleStatus` |
| 상태 진단 | - | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

GPS quality가 0이면 `STATUS_NO_FIX`와 NaN 좌표를 발행한다. 패킷이 완전히
끊겼을 때는 마지막 센서 값을 재발행하지 않고 `/diagnostics`에서 stale 상태로
알린다.

MORAI는 GGA와 그 외 NMEA 문장을 별도 UDP datagram으로 보낼 수 있다. 브리지는
`NavSatFix`에 필요한 GGA만 발행하며, GGA가 없는 정상 부가 문장은 파싱 오류로
집계하지 않는다. GGA 체크섬 오류, 비 ASCII 데이터 등 실제 손상은 계속
`parse_errors`에 포함된다.

LiDAR 관련 토픽은 bringup과 Velodyne driver가 발행한다.

| 토픽 | 타입 | 역할 |
| --- | --- | --- |
| `/sensors/lidar/packets` | `velodyne_msgs/VelodyneScan` | VLP-16 원시 패킷 |
| `/lidar3D` | `sensor_msgs/PointCloud2` | 변환된 point cloud |

## Competition Vehicle Status 수신

`competition_vehicle_status_receiver_node`는 대회용
`CompetitioninfoPublisher`의 `#MoraiInfo$` 152-byte payload만 수신한다.
다른 길이의 full Ego Vehicle Status를 같은 패킷으로 오인하지 않는다.

| ROS 필드 | 원본 | 변환 |
| --- | --- | --- |
| `control_mode` | payload +8, `uint8` | 그대로 |
| `gear` | payload +9, `uint8` | 그대로 |
| `velocity_x_mps` | payload +74, `float32` km/h | m/s로 나누기 3.6 |

MORAI Network Settings에서 `CompetitioninfoPublisher` 또는 표시 이름
`Competition Vehicle Status`를 추가하고 Destination IP를 ROS PC 주소,
Destination Port를 기본 `9094`로 설정한다. 자율 제어기는 이 토픽의
`velocity_x_mps`를 PID feedback으로 사용하며 localization 추정 속도로
자동 fallback하지 않는다.

## 자율 제어 UDP 송신

센서 수신 책임은 기존과 같다. 별도 `control_sender_node`는
`/control/actuator_command` (`morai_udp_bridge/ActuatorCommand`)만 구독하여
MORAI 제어 datagram을 전송한다.

| 필드 | 단위/범위 | 의미 |
| --- | --- | --- |
| `header` | ROS header | controller가 발행하는 명령 header |
| `accel` | `[0, 1]` | 가속 pedal 명령 |
| `brake` | `[0, 1]` | 제동 pedal 명령 |
| `steering_angle_rad` | rad, `±maximum_steering_angle_deg` | 물리 조향각 |

`accel`과 `brake`는 동시에 0보다 클 수 없다. sender는 기본 50 Hz의 wall timer로
datagram을 보내며, 유효한 명령을 0.25초 동안 받지 못하면 accel과 조향을 0으로
바꾸고 `safe_brake_command`(기본 `0.50`)를 보낸다. 유효하지 않은 명령은 watchdog
수신 시각을 갱신하지 않는다.

기본 설정에서 물리 조향 `±40 deg`는 MORAI normalized steering `±1`로 변환된다.
`steering_sign`은 차종 좌표계와 MORAI 좌표계의 부호가 반대일 때 `-1.0`으로
설정한다. MORAI Control Destination Port는 `9093`이어야 한다.

기어는 control packet에 포함되며 MORAI 번호 `1=P, 2=R, 3=N, 4=D, 5=L`을
사용한다. 기본 `gear_command_enabled: false`에서는 `drive_gear: 4`를 유지한다.
향후 활성화하면 `/control/gear_command`를 구독하지만, fresh Competition
Status, 정지 속도, 충분한 brake, 낮은 accel 조건을 만족할 때만 변경한다.

자율 `control_sender_node`와 수동 `vehicle_control` UDP sender를 동시에 실행하면
안 된다. 두 sender가 같은 MORAI 제어 대상에 서로 다른 datagram을 보내므로, 한
번에 하나만 실행한다.

## 설정 파일

| 파일 | 용도 |
| --- | --- |
| `config/sensors/molit_2026.yaml` | 카메라 3대, GPS, IMU 전체 수신 |
| `config/localization/molit_2026_localization.yaml` | localization/path 시험용 GPS+IMU 수신 |
| `config/sensors/molit_2026_gps_only.yaml` | GPS projector 단독 시험용 |
| `config/control/molit_2026_control.yaml` | 자율 ActuatorCommand UDP sender |
| `config/status/molit_2026_vehicle_status.yaml` | Competition Vehicle Status UDP receiver |

현재 `0725demo.json`의 GPS는 30 Hz, Destination Port `9301`이고 IMU는
50 Hz, Destination Port `9303`이며
`config/sensors/molit_2026.yaml`, `config/localization/molit_2026_localization.yaml`과 일치한다.
GPS-only 설정에는 IMU worker가 없으므로 IMU 토픽을 확인할 때는 위 두 설정 중
하나를 사용한다.

IMU UDP의 12-byte auxiliary header는 MORAI 버전에 따라 값이 채워질 수 있다.
공식 SensorExample과 동일하게 이 영역은 건너뛰고 quaternion, 각속도와
선가속도 payload만 해석한다. 파싱 실패 시 node 로그에 패킷 크기와 실패 이유가
5초 주기로 출력된다.

IMU header의 data-length 필드도 MORAI 버전마다 의미가 달라 공식 예제처럼
고정값을 강제하지 않는다. 대신 지원 패킷 전체 길이, header, tail, timestamp와
10개 측정값의 유한성을 검증한다.

공통 파라미터:

| 파라미터 | 설명 |
| --- | --- |
| `bind_ip` | ROS PC에서 UDP socket을 bind할 주소. 보통 `0.0.0.0` |
| `allowed_source_ip` | 허용할 MORAI PC IP. 빈 문자열이면 제한 없음 |
| `receive_buffer_bytes` | 각 UDP socket이 요청하는 수신 버퍼 |
| `diagnostics_period` | `/diagnostics` 발행 주기(초) |

센서별 파라미터:

| 파라미터 | 설명 |
| --- | --- |
| `enabled` | GPS/IMU worker 활성화 여부 |
| `port` | MORAI Sensor Edit의 Destination Port와 동일해야 함 |
| `topic` | ROS 출력 토픽 |
| `camera_info_topic` | 카메라 정보 출력 토픽 |
| `packet_layout` | `auto` 또는 명시적 MORAI packet layout |
| `use_sensor_time` | 패킷 sensor time 사용 여부. 기본은 ROS 수신 시각 |
| `stale_timeout` | 이 시간 동안 수신이 없으면 stale 진단 |
| `max_hz` | 허용 주기 상한 감시값. `0`이면 비활성 |

Competition Status 수신 파라미터:

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `bind_ip` | `0.0.0.0` | UDP bind 주소 |
| `listen_port` | `9094` | MORAI Destination Port |
| `allowed_source_ip` | 빈 문자열 | 허용할 MORAI 송신 IP |
| `receive_buffer_bytes` | `1048576` | UDP 수신 버퍼 |
| `status_topic` | `/vehicle/competition_status` | ROS 출력 |
| `stale_timeout_sec` | `0.25` | diagnostics stale 기준 |
| `maximum_publish_hz` | `50.0` | diagnostics 주기 상한 |
| `diagnostics_period_sec` | `1.0` | diagnostics 발행 주기 |

제어 sender에서 수정 가능한 값은 `destination_ip/port`, `send_rate_hz`,
`command_timeout_sec`, `safe_brake_command`, `maximum_steering_angle_deg`,
`steering_sign`, `drive_gear`다. 선택적 기어 변경은
`gear_command_enabled`, `gear_command_topic`, `vehicle_status_topic`,
`gear_change_maximum_abs_speed_mps`, `gear_change_status_timeout_sec`,
`gear_change_minimum_brake_command`,
`gear_change_maximum_accel_command`로 설정한다.

센서 `frame_id`, 카메라 해상도와 FOV는
`ioniq5_description/config/sensors/molit_2026_sensor_mounts.yaml`이 단일 원본이다.
bringup이 이 YAML을 `sensor_setup` 파라미터로 bridge에 주입한다.

## 실행

일반적으로 bridge를 단독 실행하지 않고 bringup으로 description metadata와
함께 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_bringup molit_2026_sensors.launch
```

GPS+IMU localization/path와 기본 LiDAR까지 함께 실행할 때:

```bash
roslaunch morai_bringup molit_2026_stack.launch
```

GPS `9301`과 IMU `9303`만 받으려면 센서 bringup의 `bridge_config`에
`molit_2026_localization.yaml`을 지정한다. GPS projector만 따로 시험할 때는
`molit_2026_gps_only.yaml`을 지정할 수 있다.

자율 제어 sender만 실행할 때:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch morai_udp_bridge control_sender.launch
```

Competition Status receiver만 실행할 때:

```bash
roslaunch morai_udp_bridge competition_vehicle_status_receiver.launch
rostopic echo /vehicle/competition_status
```

별도 terminal에서 명령 interface와 50 Hz 발행을 확인한다.

```bash
rostopic info /control/actuator_command
rostopic hz /control/actuator_command
rostopic pub -r 50 /control/actuator_command morai_udp_bridge/ActuatorCommand \
  '{accel: 0.20, brake: 0.0, steering_angle_rad: 0.0}'
```

## MORAI Sensor Edit와 맞출 값

MORAI의 각 센서 설정에서 다음을 확인한다.

- Destination IP: ROS PC의 실제 네트워크 IP
- Destination Port: 해당 YAML의 `port`
- GPS 현재 포트: `9301`
- IMU 현재 포트: `9303`
- 센서 장착 위치: `ioniq5_description` YAML 및 `0725demo.json`과 동일

포트 변경 시 MORAI preset과 bridge YAML을 함께 바꾼다. 같은 IP/port에 두
프로세스를 동시에 실행하면 `Address already in use`가 발생하므로 기존 bridge가
실행 중인지 먼저 확인한다.

## UDP 수신 버퍼

고해상도 카메라 burst를 받으려면 Linux `net.core.rmem_max`가 YAML의
`receive_buffer_bytes` 이상이어야 한다.

```bash
sysctl net.core.rmem_max
```

현재 운영 설정 설치:

```bash
sudo install -m 0644 \
  "$(rospack find morai_bringup)/config/99-morai-udp.conf" \
  /etc/sysctl.d/99-morai-udp.conf
sudo sysctl --system
```

## 내부 구조

```text
include/morai_udp_bridge/  C++ 공개 헤더
src/sensors/protocol.cpp           순수 Camera/GPS/IMU parser와 JPEG 조립
src/network/transport.cpp          UDP worker와 diagnostics
src/sensors/streams.cpp            ROS publisher
src/nodes/bridge_node.cpp        ROS 파라미터 조립과 실행 진입점
src/sensors/competition_status_protocol.cpp  대회 Status packet parser
src/nodes/competition_vehicle_status_receiver_node.cpp  Status UDP/ROS 경계
src/nodes/control_sender_node.cpp  actuator/optional gear command UDP 송신
```

패킷 규격 수정은 `protocol.*`, socket/진단 수정은 `transport.*`, ROS 메시지
변환은 `streams.*`에서 한다. 파서 변경 시 실제 MORAI packet fixture와 단위
테스트를 함께 추가한다.

## 확인과 문제 진단

```bash
rostopic hz /image/front/compressed
rostopic echo -n 1 /sensors/gps/fix
rostopic hz /sensors/imu/data
rostopic hz /lidar3D
rostopic echo /vehicle/competition_status
rostopic echo /diagnostics
ss -lunp | grep -E ':(9094|9291|9293|9295|9301|9303|2368) '
```

- 토픽이 없으면 worker 활성화와 launch에서 선택한 config를 확인한다.
- 토픽은 있으나 값이 갱신되지 않으면 MORAI Destination IP/Port와 방화벽을
  확인한다.
- 특정 송신자 패킷만 버려지면 `allowed_source_ip`를 확인한다.
- 카메라 drop이 많으면 수신 버퍼와 `/diagnostics`의 packet/error 카운터를
  확인한다.

대회 규정과 네트워크 관련 근거는
[`docs/network/2026_RULES_AND_NETWORK.md`](docs/network/2026_RULES_AND_NETWORK.md)에 정리되어
있다.
