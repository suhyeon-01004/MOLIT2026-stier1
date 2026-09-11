# MOLIT 2026 K-City 자율주행 스택

ROS1 Noetic과 MORAI `25.S4.MolitComp03`용 자율주행 워크스페이스다. RDDF를
K-City MGeo/Lanelet2 중심선으로 보정해 경로를 만들고, GPS·IMU·Competition
Vehicle Status로 상태를 추정해 Autoware AI 기반 MPC로 추종한다.

## 정량 성과

MORAI K-City, 속도 상한 60 km/h에서 동일 제어기·로컬라이제이션을 유지하고
분기·합류 경로만 바꾼 1회 대 1회 폐루프 A/B 결과다. 개선율은 좋은 방향을 `+`,
악화를 `-`로 표시한다.

| 경로 추종 지표 | 보정 전 | 보정 후 | 개선율 |
| --- | ---: | ---: | ---: |
| 전체 CTE RMS | 0.03256 m | 0.03278 m | **-0.65%** |
| 최대 절대 CTE | 0.16773 m | 0.16648 m | **+0.74%** |
| 문제 지점 315.8 m 국소 CTE RMS | 0.01587 m | 0.00717 m | **+54.83%** |
| 문제 지점 1285.7 m 국소 CTE RMS | 0.00993 m | 0.02542 m | **-156.03%** |
| 문제 지점 1593.2 m 국소 CTE RMS | 0.03987 m | 0.01890 m | **+52.61%** |

| 주행 품질 지표 | 결과 |
| --- | ---: |
| 문제 교차로 최대 heading 오차 | 최대 **85.95% 감소** |
| 문제 교차로 최저속도 | 최대 **80.03% 증가** |
| 문제 교차로 최대 brake 명령 | 최대 **100% 감소** |
| 기록 완주 시간 | 182.014 → 178.658 s, **1.84% 감소** |
| 급곡선 / 중곡률 조향 명령 변화율 RMS | **19.75% / 34.92% 감소** |
| 추정 jerk RMS | 2.7013 → 2.1407 m/s³, **20.75% 감소** |
| 첫 우회전 앞축 바깥점 근사 여유 평균 | 0.247 → 0.273 m, **10.48% 증가** |

CTE는 모든 구간에서 좋아진 것이 아니라 두 문제 지점에서 약 53~55% 감소했고,
전 구간 RMS는 0.65% 악화됐다. 조향 안정성은 별도의 동일 경로 3회 대 3회 A/B,
여유는 차량 제원·추정 pose·MGeo 도색선에 기반한 근사 감사값이다. 따라서 위 결과는
시험 구간의 변동성 감소 근거이지 실차 안전 여유 보장은 아니다. 세부 전후값과
폐기 후보까지 [정량 검증 요약](docs/validation/README_KO.md)과
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
