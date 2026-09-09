# molit-2026-stier1-suhyeon

MOLIT 2026 K-City용 ROS1 Noetic 워크스페이스입니다. 현재 단계에는 다음만
포함합니다.

- MORAI K-City 원본 MGeo 및 Lanelet2 HD map
- GPS/IMU UDP 수신
- noise-free GPS/IMU 직접 로컬라이제이션
- IONIQ 5 차량·센서 TF 모델
- HD map, 차량 pose, odometry, TF의 RViz 시각화

경로 계획과 차량 제어는 아직 실행하지 않습니다.

## 빌드

```bash
cd ~/molit-2026-stier1-suhyeon
./build.sh
```

## 실행

MORAI GPS/IMU UDP 포트를 각각 `9301`, `9303`으로 맞춘 뒤:

```bash
cd ~/molit-2026-stier1-suhyeon
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch morai_kcity_hd_map kcity_localization_visualization.launch
```

MORAI의 다른 브리지가 이미 `/sensors/gps/fix`와 `/sensors/imu/data`를
발행 중이라면 포트 충돌을 피하도록 다음처럼 실행합니다.

```bash
roslaunch morai_kcity_hd_map kcity_localization_visualization.launch use_udp_bridge:=false
```

RViz 없이 데이터 노드만 실행하려면 `rviz:=false`를 추가합니다.

## 주요 토픽

| 구분 | 토픽 | 타입 |
| --- | --- | --- |
| GPS 입력 | `/sensors/gps/fix` | `sensor_msgs/NavSatFix` |
| IMU 입력 | `/sensors/imu/data` | `sensor_msgs/Imu` |
| 위치·방향 | `/localization/pose` | `geometry_msgs/PoseStamped` |
| 위치·속도 | `/localization/odometry` | `nav_msgs/Odometry` |
| HD map 시각화 | `/kcity_hd_map/markers` | `visualization_msgs/MarkerArray` |
| TF | `map -> base_footprint -> base_link` | dynamic + fixed TF |

K-City 좌표계는 `EPSG:32652`, 로컬 원점은 UTM
`(302595.0, 4124145.0)`입니다. 대회 공지의 GPS/IMU 무노이즈 조건에 맞춰
pose 필터를 사용하지 않고, 속도 저역통과 필터도 꺼 두었습니다.
