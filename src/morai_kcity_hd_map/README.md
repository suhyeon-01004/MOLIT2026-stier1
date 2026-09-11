# MORAI K-City HD Map

MORAI `r_kr_pr_k-city_2025.scene`에 포함된 원본 MGeo를 Lanelet2와 ROS1 RViz용으로 변환한 지도입니다.

## 포함 데이터

- 실제 주행 링크 634개 (`lazy_init` 차선변경 보조 링크 261개 제외)
- 차선 경계 1,245개: 백색/황색, 실선/점선/혼합선
- 정지선 97개, 신호등 126개
- 횡단보도 폴리곤 77개(일반 60개·고원식 17개)와 기타 노면표시 23개
- 고속주회로 53개 Lanelet: 대회 규정상 제한 없음
- 나머지 581개 Lanelet: 대회 규정상 60 km/h

원본 MGeo 좌표계는 UTM 52N, 원점 `[302595, 4124145, 0]`입니다. Lanelet2 파일은 MORAI와 같은 로컬 XYZ를 보존하고 Autoware의 `Local` projector를 사용합니다.

## RViz

새 워크스페이스를 빌드하고 다음 통합 launch를 실행합니다.

```bash
cd ~/molit-2026-stier1-suhyeon
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch morai_kcity_hd_map kcity_localization_visualization.launch
```

RViz의 `K-City HD Map` 아래 namespace에서 차선, 정지선, 횡단보도,
신호등, 고속주회로, 차선변경 화살표를 각각 켜고 끌 수 있습니다.
`Localized Vehicle Pose`, `Localization Odometry`, `Localization TF`와
`IONIQ 5`에는 현재 차량 위치, 방향, TF와 차량 형상이 표시됩니다.

실제 RViz 구동 검증 화면은 [`preview/kcity_rviz.png`](preview/kcity_rviz.png)에 있습니다.

RViz의 `LiDAR PointCloud (toggle)` 체크로 `/lidar3D` 표시를 켜고 끌 수 있습니다.
기존 VLP-16 수신 구성(UDP 2368, 10 Hz, `lidar_link`)을 재사용합니다.
표시 체크 해제는 수신기 종료가 아닙니다. 수신까지 끄려면
`kcity_localization_visualization.launch use_lidar:=false`로 실행합니다.
라이다만 확인할 때는 `roslaunch morai_kcity_hd_map lidar.launch`를 사용하며,
이미 전체 스택의 라이다가 켜져 있을 때 중복 실행하지 않습니다.

## RDDF 기반 Lanelet2 경로

통합 launch는 `rddf_lanelet_route_node.py`도 실행합니다. 노드는 대회 RDDF의
각 점을 Lanelet2 Lanelet의 원본 중심선에 대응시키고, 교차로의 공통 접점은
RDDF 순서로 판별한 뒤 MGeo `to_node -> from_node` 연결성을 검사합니다. 전역
경로는 별도의 피팅 곡선이 아니라 선택된 Lanelet 중심선을 순서대로 이어서
재생성합니다.

폐루프 전역 경로는 `/global_path`, `/localization/pose`에서 시작하는 전방
100 m 경로는 `/local_path`로 발행합니다. 교차로 중첩부에서 최근접점이 다른
도로로 튀지 않도록 이전 인덱스 주변을 우선 검색합니다. RViz에서는 전역
경로가 청록색, 로컬 경로가 주황색으로 표시됩니다.

`route_candidate_file`의 기본값은 빈 문자열이며 기존 경로를 유지합니다.
승인된 국소 보정 비교에만 후보 JSON을 지정할 수 있습니다. 원래 경로의 XY 해시,
구간당 최대 50 m·35 cm 이동 상한을 검사하고 Z·순서·지도 연결은 유지합니다.
`route_window_m`은 한 구간, `route_windows_m`은 서로 겹치지 않는 최대 여섯 구간을
지정합니다. 승인된 첫 우회전·S자·분기/합류부 보정용이며 구간 밖 경로는 변경하지 않습니다.
일반 launch에서 후보 파일을 지정하지 않으면 원래 경로입니다.
2026-09-10 `run_team_demo.sh`의 기본 후보는 `route_candidate_default.json`입니다.
첫 우회전/S자는 60 km/h 상한의 100초 비교 후 선택한 좌표를 유지합니다.
이후 승인된 직진 합류부(원래 경로 거리 331~379 m, `A1256W000563`)의
짧은 굴곡을 보정했고, 전체 경로 분기/합류부 24곳의 주행 기록을 검사했습니다.
추가 감속이 확인된 300~330 m, 1280~1295 m, 1585~1602 m만 국소 보정했습니다.
직진의 연속성을 유지하도록 연결부 위치·방향·곡률을 이어 주되, 실제 도로의 굽음은 유지합니다.
지도·RDDF 원본과 제어기 설정은 그대로이며 다른 좌/우회전 링크까지 수정한 것은 아닙니다.
60 km/h 상한으로 수정 전/후 각 한 바퀴 실제 시뮬 완주 및 Manual/P 종료를 확인했습니다.
세 지점 최저 속도는 각각 45.6→56.8, 56.4→58.8, 25.3→45.6 km/h입니다.
전체 횡오차 RMS는 3.26→3.28 cm로 거의 같으며, 1593 m 차체 여유 추정은 53→34 cm로
줄었습니다. 일부 경계는 가상이므로 이 수치가 양쪽 차선/충돌 안전을 보장하지 않습니다.
종방향 PID fallback 1샘플도 기록했습니다. 반복 주행/100 km/h 검증은 하지 않았습니다.
상세 결과는 `artifacts/controller_validation/2026-09-10/merge_correction/junction_fix_live_comparison.json`,
직전 기본값은 `backups/junction_fix_before_20260910/`에 보존했습니다.

오프라인 검증 명령:

```bash
rosrun morai_kcity_hd_map rddf_lanelet_route_node.py --validate-only \\
  --rddf $(rospack find morai_kcity_hd_map)/map/2026_molit_comp_global_path.txt \\
  --lanelet-map $(rospack find morai_kcity_hd_map)/map/lanelet2_map.osm \\
  --raw-mgeo $(rospack find morai_kcity_hd_map)/raw_mgeo
```

## 재생성 및 검사

```bash
/usr/bin/python3 scripts/build_lanelet2.py
/usr/bin/python3 scripts/validate_map.py
```

생성 파일은 `map/lanelet2_map.osm`이며 Autoware용 투영 설정은 `map/map_projector_info.yaml`입니다.

## 변환 기준

- MGeo `lane_shape=solid/broken`을 Lanelet2 `solid/dashed`로 변환합니다.
- MGeo `lane_type=530`을 정지선으로 변환합니다.
- 링크의 좌·우 차선변경 플래그를 각 Lanelet에 보존합니다.
- 링크 길이의 60% 이상을 덮는 실제 경계를 Lanelet 면에 사용하고, 부분 경계는 원본 표시를 보존하면서 MGeo 폭으로 면 경계를 보완합니다.
- 신호등은 연결 링크의 끝점에서 25 m 이내인 가장 가까운 정지선과 연결합니다.
- `singlecrosswalk sign_type=5321`(일반 60개)과 `533`(고원식 17개)을
  횡단보도 Area 및 RViz 줄무늬로 처리합니다. `534`(자전거횡단도)와
  `544`(오르막경사면)는 보행자 횡단보도와 구분합니다.
- 대각선 횡단보도 두 영역이 겹치는 중앙은 RViz 줄무늬만 잘라 격자 중첩을 방지합니다.
  원본 영역·경로는 유지하며, 줄무늬는 영역에서 생성한 표시이지 MORAI 페인트 텍스처의 복제가 아닙니다.
