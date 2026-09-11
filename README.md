# MOLIT 2026 K-City 자율주행 스택

ROS1 Noetic과 MORAI `25.S4.MolitComp03`용 자율주행 워크스페이스다. RDDF를
K-City MGeo/Lanelet2 중심선으로 보정해 경로를 만들고, GPS·IMU·Competition
Vehicle Status로 상태를 추정해 Autoware AI 기반 MPC로 추종한다.

## 구조

```text
MORAI GPS/IMU/Competition Status
  -> morai_udp_bridge + morai_localization
  -> morai_kcity_hd_map: MGeo/Lanelet2, 보정 경로, 속도 제한
  -> morai_path_tracking: 경로·속도 계획, Autoware MPC 연결, 종방향 제어
  -> morai_udp_bridge: MORAI 제어 송신
```

- `src/morai_path_tracking`: 프로젝트의 단일 제어 패키지
  - `launch/control`: 현재 운영 Autoware MPC 실행
  - `scripts/control`: ROS 메시지·안전 제한 어댑터
  - `src/controllers/longitudinal`: 현재 종방향 MPC
  - `src/controllers/lateral`: 정량 비교용 기존 자체 제어기
  - `launch/legacy`: 기존 제어기 재현 전용
- `src/vendor/autoware_ai`: 출처와 라이선스를 보존한 외부 ROS1 Autoware 패키지
- `src/morai_kcity_hd_map`: HD map, Lanelet2 경로, 속도 제한, RViz
- `src/morai_localization`: GPS/IMU pose·odometry와 TF
- `src/ioniq5_description`: 차량 치수와 센서 장착 TF

`morai_control` 같은 두 번째 프로젝트 제어 패키지는 두지 않는다. 외부 Autoware
패키지만 ROS 패키지명·헤더·라이선스 호환 때문에 `vendor` 아래에 분리한다.

## 빌드

```bash
cd ~/molit-2026-stier1-suhyeon
./build.sh
source install/setup.bash
```

## 실행

먼저 제어 송신 없이 센서·경로·MPC·RViz를 확인한다.

```bash
roslaunch morai_path_tracking autoware_mpc.launch send_control:=false
```

입력 토픽과 초기 위치를 확인한 뒤에만 실제 송신을 켠다.

```bash
roslaunch morai_path_tracking autoware_mpc.launch send_control:=true
```

자동 초기화와 한 바퀴 기록까지 수행하는 시연 스크립트는 MORAI를
`Manual -> I -> P`로 둘 수 있을 때만 실행한다.

```bash
bash ~/molit-2026-stier1-suhyeon/run_team_demo.sh
```

현재 기본 속도는 일반·고주로 모두 최대 `60 km/h`다. `run_map_speed_demo.sh 100`은
고주로 100 km/h 입력 시험용일 뿐, 안전성이나 추종 성능이 검증된 설정이 아니다.

## 검증 자료

정량 결과, 개선율, 원본 실험 경로와 해석상의 제한은
[검증 요약](docs/validation/README_KO.md)에 정리한다. 원본 rosbag과 전체 시계열은
용량 때문에 `artifacts/`에 로컬 보존하며 Git에는 요약 CSV와 그림만 포함한다.

K-City 원본 MGeo는 재배포 권한이 확인되지 않아 저장소에서 제외될 수 있다.
외부 코드 출처와 라이선스는 [vendor 안내](src/vendor/autoware_ai/README.md)를 따른다.
