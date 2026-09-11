# MOLIT 2026 K-City 자율주행 스택

ROS1 Noetic과 MORAI `25.S4.MolitComp03`용 자율주행 워크스페이스다. RDDF를
K-City MGeo/Lanelet2 중심선으로 보정해 경로를 만들고, GPS·IMU·Competition
Vehicle Status로 상태를 추정해 Autoware AI 기반 MPC로 추종한다.

## 나의 역할

- **담당:** 전체 시스템 구조 정리, UDP–ROS 연결, GPS·IMU 로컬라이제이션, 경로 추종 제어기 비교 및 튜닝
- **직접 수행:** MORAI 네트워크 입력부터 위치 추정, 경로, 제어 명령까지 이어지는 데이터 흐름과 노드 인터페이스 구성
- **제어:** Pure Pursuit·Stanley·MPC를 구현하고 시뮬레이션 결과를 비교했으며, 반복 주행 데이터를 바탕으로 파라미터 조정
- **현재 구성:** Autoware AI 기반 MPC와 프로젝트용 안전 제한·종방향 제어를 연결해 사용
- **사용 기술:** ROS1 Noetic, Python, C++, MORAI, GPS/IMU, Lanelet2, RViz, MPC

이 저장소는 팀 프로젝트 코드와 라이선스를 보존한 외부 Autoware 패키지를 함께 포함합니다. 외부 패키지와 팀원이 맡은 기능은 개인 구현 범위에 포함하지 않습니다.

## 전체 개발 전후 정량 성과

MORAI K-City 60 km/h급 폐루프 완주에서 초기 미튜닝 Autoware MPC 3회 평균과
최종 채택 상태 1회를 같은 분석기로 비교했다. `초기 최악`은 하나의 주행이 아니라
각 지표별 초기 3회 중 가장 나쁜 값이다.

| 지표 | 초기 3회 평균 | 초기 최악 | 최종 | 평균 대비 개선 |
| --- | ---: | ---: | ---: | ---: |
| raw-GPS CTE RMS | 0.08534 m | 0.08632 m | 0.05872 m | **31.19%** |
| raw-GPS CTE p95 | 0.16871 m | 0.17098 m | 0.11417 m | **32.33%** |
| localized-pose CTE RMS | 0.08534 m | 0.08631 m | 0.03195 m | **62.56%** |
| 고속 직선 CTE RMS | 0.05799 m | 0.05985 m | 0.01360 m | **76.55%** |
| raw-GPS heading RMS | 1.0079° | 1.0190° | 0.6871° | **31.83%** |
| 고속 직선 조향 변화율 RMS | 2.1173°/s | 2.1534°/s | 1.5591°/s | **26.37%** |
| 고속 직선 1초 조향 진폭 p95 | 0.9834° | 1.0532° | 0.6311° | **35.82%** |
| 고속 직선 CTE 2 cm 교차 횟수 | 130.3회 | 138회 | 6회 | **95.40%** |
| 가속-제동 명령 전환 빈도 | 28.52회/min | 29.22회/min | 19.58회/min | **31.35%** |
| 추정 jerk RMS | 3.7285 m/s³ | 3.8681 m/s³ | 1.6352 m/s³ | **56.14%** |
| 평균 속도 | 42.478 km/h | 42.356 km/h | 43.894 km/h | **3.33%** |
| 기록 완주 시간 | 186.069 s | 186.572 s | 178.658 s | **3.98%** |

raw-GPS 기준의 CTE 개선율 `31.19%`를 보수적인 대표값으로 사용한다. localized-pose
CTE `62.56%` 개선에는 로컬라이제이션 보완 효과도 포함된다. 이 표는 제어기·상태추정·
경로가 함께 바뀐 **전체 개발 전후 비교**이므로 특정 파라미터 하나의 인과 효과로
해석하지 않는다. 최종 경로도 초기보다 `0.78%` 짧아 완주 시간과 평균 속도를 함께
제시했다.

### 변경 사항별 효과

| 변경 사항 | 비교 조건 | 개선된 지표 | 악화·한계 | 판단 |
| --- | --- | --- | --- | --- |
| MPC 기준 yaw 범위 `±0.25 → ±2.5 m` | 동일 경로 3회 대 3회 | 곡선 조향 변화율 RMS **19.75~34.92% 감소**, 추정 jerk **20.75% 감소** | 직선 조향 변화율 4.86%, 급곡선 raw-GPS CTE 19.30% 증가 | 설계 근거로 채택 |
| 분기·합류 3개 지점 경로 형상 보정 | 동일 제어기·로컬라이제이션 1회 대 1회 | 두 지점 국소 CTE **54.83%, 52.61% 감소**, heading 오차 최대 **85.95% 감소**, 최저속도 최대 **80.03% 증가** | 전 구간 CTE 0.65%, 한 지점 국소 CTE 156.03% 증가 | 채택 |
| 앞축 오차 비용 `q=0 → 0.5` | 동일 경로 3회 대 2회 | 유의한 여유 개선 없음 | 앞축 근사 여유 6.67%, 차체 근사 여유 79.49% 감소 | 폐기 |
| GPS·IMU·Competition 속도 융합 | 독립 A/B 없음 | 전체 누적 결과에 포함 | 로컬라이제이션만의 개선율은 분리 불가 | 운영 반영, 독립 수치 미주장 |

조향 실험은 이후 설계 선택의 근거이며 현재 최종 파라미터 하나만의 독립 효과는
아니다. 차체·앞축 여유는 차량 제원과 추정 pose로 계산한 근사 감사값이므로 안전
보장으로 사용하지 않는다.

계산 가능한 전체 지표는 [개발 전후 비교 CSV](docs/validation/development_progress.csv),
[4개 실행 원자료](docs/validation/development_runs.csv), 변경별 통제 A/B와 폐기 후보는
[정량 검증 요약](docs/validation/README_KO.md)과
[35개 지표 CSV](docs/validation/portfolio_metrics.csv)에 공개한다.

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
