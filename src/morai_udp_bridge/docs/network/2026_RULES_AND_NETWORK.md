# 2026 대회 센서 통신 기준

확인 기준은 `규정집 v1.0 (2026-07-11)`과 MORAI 24.R2 공식 센서 UDP 문서다.
대회용 시뮬레이터 버전은 규정상 `25.S4.MolitComp03`이므로 최종 판단은 해당
버전의 실제 datagram으로 재검증해야 한다.

## 이 패키지에 적용되는 규정

- 센서와 참가팀 PC 사이의 통신은 UDP만 사용한다.
- GPS: 최대 1대, 최대 30 Hz
- IMU: 최대 1대, 최대 50 Hz
- 3D LiDAR: 최대 1대, VLP-16, 최대 15 Hz, 10 Hz 이하 권장, intensity 사용 가능
- Camera: 최대 4대, 최대 30 Hz
- GPS 음영 구역에서는 GPS 정보가 모두 blackout 된다.

규정의 허용 네트워크에는 `Ego Ctrl Cmd`, `CollisionData`,
`Competition Vehicle Status`가 포함된다. 이 패키지는 센서 수신 외에
Competition Vehicle Status 수신과 Ego Ctrl Cmd 송신의 wire 경계도 담당한다.
경로 추종과 PID 계산은 `morai_path_tracking`에 남긴다.

고정 카메라의 위치·자세·해상도·FOV는 통신 설정에 넣지 않는다. 규정값은
`ioniq5_description/config/sensors/molit_2026_sensor_mounts.yaml`에서 단일 원본으로
관리하며, 같은 패키지가 장착 transform과 camera optical-frame 축 변환을 맡는다.

## 네트워크 포트

다음 값은 현재 팀 패키지의 포트 계약이며 규정이 포트 번호를 강제하는 것은
아니다. SIM UI와 YAML이 반드시 같아야 한다.

| 센서 | ROS PC destination port |
|---|---:|
| Front / Left / Right camera | 9291 / 9293 / 9295 |
| GPS | 9301 |
| IMU | 9303 |
| VLP-16 | 2368 |
| Competition Vehicle Status | 9094 |

대회 LAN에서는 SIM의 Destination IP를 ROS PC 주소로 설정한다. `Host Sensor
Port`는 SIM의 송신 측 설정이며 ROS 수신 socket이 bind하는 Destination Port와
혼동하지 않는다.

## 패킷 호환성

- Camera: `MOR` header, sec/nsec, index, size, 분할 JPEG, `AI`/`EI` tail을
  검증한다. 구버전 11-byte header도 자동 감지한다.
- GPS: 한 UDP datagram의 NMEA0183 RMC/GGA에서 checksum이 유효한 GGA를 사용한다.
- IMU: 공식 문서의 107-byte 형식과 MORAI 공식 예제에서 사용하는 sec/nsec 포함
  115-byte 형식을 모두 지원한다.
- VLP-16: `morai_bringup`이 MORAI native UDP를 공식 ROS `velodyne` 드라이버에
  연결한다. 이 브리지 노드는 중복 packet decoder를 구현하지 않는다.
- Competition Vehicle Status: 대회용 Simulator의 `CompetitioninfoPublisher`
  152-byte payload만 허용하고 x축 velocity를 km/h에서 m/s로 변환한다.

기본 ROS header timestamp는 PC 수신 시각이다. 실제 UDP sec/nsec와 ROS clock의
동기화가 검증된 경우에만 `use_sensor_time: true`로 변경한다.

## GPS map projection

WGS84 `NavSatFix` 발행은 이 패키지의 책임이지만 UTM 좌표에서 map offset을 빼는
과정은 통신이 아니라 시나리오 localization에 해당한다. 관련 코드와 설정은
`morai_localization` 패키지로 분리했다.

최종 시나리오 JSON에서 아래를 확인한 뒤 localization 패키지를 활성화한다.

- global coordinate system 및 UTM zone
- scenario coordinate system
- east/north offset
- map 이름과 배포 버전

## 원문

- 규정집: <https://morai.atlassian.net/wiki/external/ZThjYjQwNDNiM2ZkNDBjODk2MDk4MmM5MjA3Y2ZmODM>
- 센서 프로토콜: <https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-35>
- 공식 센서 예제: <https://github.com/MORAI-Autonomous/MORAI-SensorExample>
