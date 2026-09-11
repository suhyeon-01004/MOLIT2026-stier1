# Vendored Autoware AI ROS1 MPC

이 디렉터리는 현재 ROS2 Autoware Universe가 아니라, ROS1 Autoware AI의
`mpc_follower`와 필요한 의존성만 고정해 둔 vendor 영역이다. 프로젝트 실행과
MORAI 변환 코드는 `morai_path_tracking`에 있으며 여기에는 두지 않는다.

## 고정한 upstream

| 구성 | 기준 commit |
| --- | --- |
| `mpc_follower` (Autoware.AI core_perception) | `48c67999823d66d297f9fd01dda5584604282135` |
| `amathutils_lib` (Autoware common) | `202e492b88a89b58ca83f66cec089eaa27b440d4` |
| `autoware_msgs` | `0a210c604176b0a77098507f6691d627a3249631` |
| `qpoases_vendor` | `ca4e3e1e1c796f4ce4e17e46ba4372607d87f87b` |
| qpOASES | `51d3fbea30142d3acbf40cf7a1c519efc27ea67b` |

## 프로젝트 변경 범위

upstream 원본 그대로라고 주장하지 않는다. MORAI 시뮬레이터 실측 A/B를 위해 다음을
추가·수정했다.

- 명령 대비 유효 조향 이득과 steering 상태 모델
- qpOASES hot-start 및 조향률 제약
- 곡률별 비용 가중치와 속도에 따른 prediction 길이
- 경로 yaw 계산 범위, 첫 조향 변화 비용, 앞축/overhang 비용 후보
- 선택형 pose·yaw·조향 시간 정렬과 stale/frame 불일치 거부

현재 채택 파라미터는 `morai_path_tracking/config/controllers/autoware_mpc.yaml`에
있다. 시간 정렬과 앞축 비용처럼 시험 후 미채택된 기능도 기본 OFF로 남겨 재현과
회귀 검사를 지원한다. 사용하지 않는 `mpc_waypoints_converter`는 빌드하지 않는다.

## 의존성과 라이선스

- Autoware AI 패키지: 각 소스 헤더와 `package.xml`에 명시된 Apache License 2.0
- qpOASES: `qpoases_vendor/vendor/LICENSE`의 LGPL 조건

`autoware_msgs` 원본 정의는 출처 추적을 위해 보존하되 MPC에 필요한 8개 메시지만
CMake에서 생성한다. 이 디렉터리는 완전한 Autoware 배포판이 아니다.
