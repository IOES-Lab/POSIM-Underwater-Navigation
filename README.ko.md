# POSIM Underwater Navigation

POSIM의 측정 센서 기반 수중 항법 컴포넌트입니다.

**개발 상태:** 실험용 라이브러리이며 차량 항법이 검증된 완제품이 아닙니다.
현재 테스트는 실제 센서 전달, FCU 수용, 설치형 앱 또는 임무 안전을 보증하지
않습니다. 물리 차량의 제어기에 이 후보를 연결하지 마세요.

## 구현 범위

- raw IMU 사전적분과 DVL 관측을 사용하는 error-state filter
- 명시적 보정 사전정보를 사용하는 이동/정지 초기화 함수
- 짧은 DVL bottom-lock 상실에 대한 제한된 IMU 추정 유지
- scale을 임의로 조절하지 않는 GNSS/지역 궤적의 수평 정렬 후보

측정값과 공분산을 사용합니다. 시뮬레이터의 정답 pose나 생성 위치를 절대 좌표로
사용하지 않으며 원래 GNSS fix 상태를 바꾸지 않습니다.

## 설치와 시험

Python 3.10 이상과 NumPy가 필요합니다. ROS와 POSIM-SLAM은 필요하지 않습니다.

```sh
python -m pip install .
python -m unittest discover -s tests
```

## 동작과 제한

`BoundedInertialDvlSession`은 유효한 짧은 bottom-lock 상실 보고에서 IMU 전파를
유지합니다. 가짜 0 속도 DVL 관측을 만들지 않습니다. 실제 전파된 위치·속도·자세
공분산을 검사한 뒤 후보 출력을 반환합니다. 개발 기본값은 다음과 같습니다.

- 수락한 DVL 보조 이후 최대 유지 시간: 1.5초
- 위치 불확실도: 3σ 기준 5 m
- 속도 불확실도: 3σ 기준 0.5 m/s
- 자세 불확실도: 3σ 기준 5도

이 값은 보수적인 개발 파라미터이지 측정된 정확도 보장이 아닙니다. IMU 누락,
잘못된 입력, 시간 역전 또는 한계 초과는 segment를 무효화합니다. DVL 재획득은
실제 관측의 innovation 검사를 통과해야 합니다. 라이브러리는 차량의
`navigation_ready`를 true로 선언하지 않습니다.

`align_planar_tracks`는 측정시간이 맞는 미터 단위 GNSS/지역 궤적 쌍을 받습니다.
GNSS 좌표 투영, 안테나 위치 보정, 최신성 및 frame 확인은 호출자 책임입니다.
기본 후보 기준은 8개 표본과 15 m 이동 범위입니다. 정렬은 FCU 입력 전환이나
자동 임무 재시작을 의미하지 않습니다.

## 구성요소 경계

목표 의존 방향은 **POSIM-SLAM → Underwater-Navigation**입니다. Navigation은
SLAM에 의존하지 않습니다. 이 저장소는 기존 POSIM-SLAM 코드를 수정하지 않습니다.
초기 수치 코어의 출처는 [NOTICE](NOTICE)에 기록되어 있습니다. 중복 구현이
서로 달라지지 않도록 실제 소유권 이관은 별도로 조율합니다.

앱은 센서 전달, 세션 격리, 승인 provider 등록, FCU 입력 전환 응답, 임무 판단과
화면 표시를 담당합니다. [구조 및 계획](docs/architecture.md)은 현재 코드와 예정된
통합을 구분합니다. [English](README.md).

Apache-2.0. [LICENSE](LICENSE), [NOTICE](NOTICE).
