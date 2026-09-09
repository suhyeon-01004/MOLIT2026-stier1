# MORAI K-City HD Map

MORAI `r_kr_pr_k-city_2025.scene`에 포함된 원본 MGeo를 Lanelet2와 ROS1 RViz용으로 변환한 지도입니다.

## 포함 데이터

- 실제 주행 링크 634개 (`lazy_init` 차선변경 보조 링크 261개 제외)
- 차선 경계 1,245개: 백색/황색, 실선/점선/혼합선
- 정지선 97개, 신호등 126개
- 횡단보도 폴리곤 60개와 기타 노면표시 40개
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
- `singlecrosswalk sign_type=5321`을 횡단보도 Area로 변환합니다.
