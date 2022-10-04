# Left-invariant EKF Localization with GPS and IMU

외국 대학의 term project에서 가져온 것입니다.



해당 프로젝트의 목적은 다음과 같습니다.

1. SLAMbook에서 학습한 Rigid body transformation과 Lie group/Lie alegra를 이용한 추정이론 로직을 실습
2. 특히, Lie group/Lie algebra 상에서 최적화를 하는데 필요한 이론을 학습하고 필요에 의해 유도할 수 있는 능력을 습득



해야할 일은 다음과 같습니다.

1. `docs\proposal.pdf` 와 `docs\report.pdf`를 보고 이론에 대해서 간단히 학습을 한다.
2. 핵심 코드 (EKF, LIEKF)의 코드를 구현해보고 시뮬레이션을 해본다.
   - `matlab/filters`의 매트랩 스크립트를 구현하고 결과 분석 (default)
     - 관심이 있다면 `Right Invariant EKF'도 수식 유도 후 구현
   - `cpp/src`에 있는 `IEKF.cpp`와 `util.cpp` 완성, 코드가 빌드 될 수 있도록 CMakeLists.txt 수정 
     - `tests` 폴더를 보니 유닛 테스트를 하기 위해 `Boost`라이브러리를 사용했습니다. 제가 경험한 바로는 회사 PWS에서는 해당 라이브러리가 빌드되지 않습니다. 따라서 tests는 제외하고 빌드하거나 능력자 분은 `gtests`나 `Catch2`등의 라이브러리로 바꿔주세요.


