# Component architecture / 구성요소 구조

## Current / 현재

The package depends only on NumPy. Its core contains measured IMU integration,
initialization and DVL fusion primitives. The bounded-coast supervisor and planar
GNSS alignment are experimental. There is no ROS node, geodetic projection,
automatic surfacing, FCU adapter or installed-app integration in this package.

현재 의존성은 NumPy뿐입니다. 코어는 측정 IMU 적분, 초기화와 DVL 융합 함수를
포함합니다. 제한된 IMU 추정 유지와 GNSS 수평 정렬은 실험 단계입니다. ROS 노드,
지리 좌표 투영, 자동 부상, FCU 어댑터, 설치형 앱 통합은 포함하지 않습니다.

## Intended ownership / 목표 책임

```text
Application ────────> Underwater-Navigation ──> NumPy
      └──> SLAM ────> Underwater-Navigation
```

SLAM remains responsible for feature association, landmarks/maps and joint
visual/acoustic optimization. Navigation owns sensor-based motion estimation,
frames/time/covariance contracts and surface/submerged navigation lifecycle.
The application owns orchestration and FCU/mission acceptance.

SLAM은 특징 대응, landmark/지도와 영상·소나 결합 최적화를 담당합니다.
Navigation은 센서 기반 운동 추정, 좌표계·시간·공분산 계약과 수면/수중 항법
상태전이를 담당합니다. 앱은 실행 조율 및 FCU/임무 수용 여부를 담당합니다.

The arrow from SLAM is a migration target, not a claim that existing SLAM releases
already depend on this package. No active SLAM branch is modified here. Public
core extraction preserves provenance and source behavior. Temporary duplication
must be resolved with a coordinated API migration, not parallel algorithm edits.

SLAM 의존 화살표는 이관 목표이며 기존 SLAM 배포판이 이미 이 패키지를 사용한다는
뜻이 아닙니다. 진행 중인 SLAM 브랜치는 수정하지 않습니다. 공개 코어 추출은 출처와
기존 동작을 보존합니다. 일시적 중복은 API 이관으로 해소하며 알고리즘을 양쪽에서
따로 변경하지 않습니다.

## Planned acceptance / 예정 검증

1. Fresh measured surface GNSS initialization, geographic projection and heading
   observability; no simulator pose or hard-coded geographic anchor.
2. Submerged measured IMU/DVL navigation with actual drift and covariance checks.
3. Separate short/long DVL loss, stale/malformed sensors, clock reset and rejected
   observations; validate estimator and real FCU control state together.
4. Fresh surface GNSS reacquisition, acknowledged source switching and a new
   initialization after an invalid segment. No unconditional mission resumption.
5. Version-pinned integration, provider provenance and final installed-user tests.

1. 실제 새 수면 GNSS 초기화, 지리 좌표 투영 및 방위 관측가능성 검증.
   시뮬레이터 pose나 고정 지리 anchor를 사용하지 않습니다.
2. 실제 IMU/DVL 수중 항법의 drift와 공분산 검증.
3. 짧은/긴 DVL 상실, stale·잘못된 센서, 시간 reset 및 관측 거부를 분리하고
   추정기와 실제 FCU 제어 상태를 함께 검증.
4. 새 수면 GNSS 재획득, 입력 전환 응답 및 무효 segment 이후 새 초기화.
   임무를 무조건 재개하지 않습니다.
5. 버전 고정 통합, provider 출처와 최종 설치형 앱 사용자 흐름 검증.

A SLAM correction derived from the same IMU/DVL samples is not an independent
measurement. Future feedback must preserve measurement ownership/correlation to
avoid double-counting information and understating uncertainty.

같은 IMU/DVL에서 유도된 SLAM 보정은 독립 관측이 아닙니다. 향후 피드백은 관측
소유권과 상관관계를 보존하여 정보 중복 계산과 불확실도 과소평가를 방지해야 합니다.
