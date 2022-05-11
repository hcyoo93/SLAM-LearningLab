# SLAM 스터디 자료

## SLAM이란?

SLAM이란 Simulationous Localization And Mapping의 약자로 자신의 위치를 추정하는 로컬라이제이션과 이를 기반으로 한 맵 빌딩을 동시에 수행하는 기법을 말합니다. 여기서 맵 빌딩을 또다른 실용적인 영역이므로 많은 논문들과 연구들은 로컬라이제이션 문제를 해결하는 것에 집중하고 맵 빌딩을 유명한 오픈소스 SW를 사용하는 경향이 강합니다.
하지만 센서퓨전2파트는 맵 빌딩 기술에 대해서도 관심을 가지고 해당 기술 중 우리가 활용할 수 있는 유용한 부분이 있는지 파악하는 것도 도움이 된다고 생각합니다.

다시 돌아와서, 많은 논문과 연구들은 로컬라이제이션에 집중하고 있고 이를 수학적으로 정의하면 결국 주변환경 (environment)과 물리적 특성 (위치, 속도, 자세 등) 혹은 의미론적 특성 (분류 등)을 추정(estimate)하는 과정입니다. 이 과정은 추정하고자 하는 값들을 최적 추정을 하는 것으로 귀결됩니다. 이런 최적 추정을 하는 방법들은 1960년대 개발되어 유도항법/항공 분야에서 발달하여 여전히 많은 영역에서 활용되고 있는 Kalman-filtering 기반의 기법과 2000년 대 초반 컴퓨터 비전 및 로봇 영역에서 활발히 연구된 비선형 최적화 (non-linear optimization) 기반의 기법들로 크게 구분됩니다.

SLAM은 거의 40년의 역사를 가지고 있습니다. 1986년 발표된 이래로 2D SLAM은 filtering 기법을 기반으로 발달하였습니다. extended Kalman-filter 기반의 기법들과 MCMC (Markov Chain Monte Carlo) 기반의 particle-filter 등의 연구가 쏟아졌습니다. 여러분이 많이 가지고 있는 Probabilistic Robotics (By Sebastian Thrun, Wolfram Burgard and Dieter Fox)가 해당 연구자들이 미국 스미소니언 박물관에서 안내를 하는 로봇을 이 2D SLAM 기반으로 개발한 후 해당 기술을 집대성하여 발표한 것이 이 책입니다.

하지만 컴퓨터 비전 영역에서 많은 영상들에서 특징을 추출 및 정합하여 비선형 최적화를 통해 대규모의 3D 재구성을 연구 이후, 이를 이용하여 3D localization을 시도하는 연구들이 활발해지기 시작했습니다. 또한 3D localization은 드론과 같은 3D로 움직일 수 있는 로봇들에 대한 관심이 증가하며 로보틱스 영역에서도 활발히 연구하기 시작했고 현재는 자율주행 영역에서도 정확하게 SLAM은 아니지만 이를 응용한 기술을 많이 적용하고 있는 것으로 알고 있습니다.
(2D SLAM/3D SLAM은 학계에서 구분하는 용어가 아닌 개인적인 구분입니다. 요즘 modern C++을 공부하다보니 이를 modern SLAM이라고 혼자 부르기도 합니다.)

![bundle adjustment in the large](./figures/bundle%20adjustment%20in%20the%20large.png)

## SLAM 이해

위에서 말씀드린 것처럼 이런 modern SLAM을 이해하기 위해서는 어떻게 해야할까?
우선 현재 SLAM 기술을 이끌어나가고 있는 연구 그룹들의 논문을 읽어보면 좋지만, 대부분 수학 및 해당 분야에 대한 어느 정도의 기반 지식이 있다고 가정하기 때문에 따라가기 어렵습니다. 그렇다고 open-source만 돌려보자니 뭔가 남는 것이 없는 것 같고, 실시간 운용을 위해서 C++로 구현을 해놓았는데 코드를 분석하는 것도 딥러닝 쪽 대비해서도 어려운 편입니다.
이번 스터디에서는 SLAM 기술의 큰 줄기와 기술동향 분석을 통해 우리가 활용 혹은 개발해야할 기술들을 파악하고 이를 이해하기 위한 기본적은 수학적 기반지식을 획득하는 것입니다.

현재의 modern SLAM을 이해하기 위해서는 다음과 같은 지식이 요구됩니다.

1. Pose (position + orientation) paramtrization
    * pose 중 적어도 orientation은 비선형 공간에서 다뤄져야 하며, 이를 위해서는 SO3에 대응하는 Lie group과 Lie algebra에 대한 이해가 필요합니다.
    * pose를 하나로 다루기 위해서는 SE3에 대응하는 Lie group과 Lie algebra에 대한 이해가 필요합니다.
    * 많은 논문들이 서로 다른 convention을 사용하는 경우가 많기 때문에 해당 논문에서 이를 어떻게 다루었는지 이해하지 못한 채 공식만 가져다쓸 경우 이슈가 발생합니다.
1. Bayesian Estimation
    * SLAM 문제는 결국 Gaussian noise를 기반으로 probabilistic formulation을 하고 이를 추정하는 것이므로 이를 결국 Bayesian estimation에 대한 기본적인 이해가 필요합니다.
    * 필터링 및 비선형 최적화는 결국 이 Baysian estimation을 수행하기 위한 수학적 도구이며, 어떤 문제 해결에 집중하는가에 따라 필요한 기법을 선택하면 됩니다. 예를 들어 드론처럼 리소스가 제한되며 빠른 연산이 필요한 경우는 일반적으로 필터링 기법을 선호하며, 그렇지 않은 경우 비선형 최적화가 선호됩니다.
1. Non-linear optimization (특히, least-square)
    * Factor-graph optimization
        * 비선형 최적화를 적용하여 SLAM 문제를 풀 경우, 그래프 기반으로 추정하려는 상태 변수들 사이의 관계를 정의한 후, 그 사이에서 최소화할 오차를 모델링합니다.. 이 때, 이 상태 변수들 사이의 관계를 factor, 상태 변수들을 node로 정의하여 모델링한 것이 factor-graph optimization입니다.
        * 이 때, 비선형으로 정의된 측정치 오차 모델을 gaussian 기반의 Mahalanobis distance로 변경하기 위해서는 Jacobian, Hessian 등의 계산이 필요하며 이를 위한 수학적 지식이 필요합니다.
    * Spasity 기반 problem reformulation
      * SLAM은 많은 pose 상태 변수들과 map 상태 변수들을 추정하지만 이 상태 변수들 사이의 관계는 sparse하게 연결되어 있습니다.
      * 이 sparsity를 이용하면 비선형 최적화 문제의 연산 속도를 획기적으로 개선할 수 있으며, 이것이 SLAM을 real-time (10 Hz 이상)으로 운용할 수 있는 핵심이 됩니다.

추가로 현재의 SLAM에서 어떤 센서들을 어떻게 활용하는지도 중요합니다.

1. Camera
    * 주변환경에 대한 풍부한 정보를 제공합니다.
    * 깊이 정보가 데이터에서 제외됩니다.
    * 환경 (날씨 등)에 의해 성능 열화가 발생합니다.
1. IMU
    * 자신의 움직임에 대한 가속도, 각속도 정보를 빠른 속도로 제공합니다.
    * 단독으로 사용할 경우 오차가 누적되기 때문에 이를 보조할 센서가 필요합니다.
    * 환경 (날씨 등)에 의해 성능 열화가 발생하지 않습니다.
1. Lidar
    * 깊이 정보를 포함한 주변환경에 대한 3D geometry를 제공합니다.
    * 비교적 다른 센서 대비 고가이며, 환경 (날씨 등)에 의해 성능 열화가 발생할 수 있습니다.
1. (Imaging) Radar
    * 현재로는 Lidar 대비 적은 편이지만 주변 환경에 대한 3D geometry와 속도 정조를 제공합니다.
    * 환경 (날씨 등)에 의해 성능 열화가 발생하지 않습니다.

또한 SLAM은 매우 실용적인 학문인 만큼 이를 실제로 구현하는 것이 중요하며, 이를 위해 다음의 툴들에 익숙해져야 합니다.

1. Linux 시스템 및 연관한 C++ 개발 툴
    * 기본적인 linux 시스템 사용법 (ex. bash)
    * cmake
1. C++ (특히, modern)
    * 참고가능한 소스코드의 대부분이 modern C++로 구현됨
1. C++ 연산 라이브러리
    * eigen: 행렬 연산 라이브러리
    * ceres, g2o, gtsam: 비선형 최적화 solver

## 자료 추천

1. Pose (position + orientation) paramtrization
    * [Quaternion kinematics for the error-state Kalman filter by Jona Sola (2017)](/reference/papers/Quaternion%20kinematics%20for%20the%20error-state%20Kalman%20filter%20[Sola%2017].pdf)
      * Rotation parametrization에 대해서 최신 로보틱스에 사용하는 notation 및 SO3 상에서의 error state 설계에 대해서 가장 정리가 잘 되어있음.
    * [Navigation using inertail sensors by Paul D. Groves (2014)](reference/papers/Navigation%20using%20Inertial%20Sensors%20[Groves%2014].pdf)
      * Aerospace 분야에서 어떻게 rotation parametrization을 하는지 이해할 수 있고, 기본적인 large scale navigation에 대한 기본지식을 얻을 수 있어서 위의 자료와 병행해서 보는 것은 추천함.
      * 좀 더 Inertial navigation에 대해서 알고 싶다면, 저자의 책도 추천함.
1. Bayesian Estimation
    * [Bayesian Filtering and Smoothing by Simo Särkkä](reference/papers/Bayesian%20Filtering%20and%20Smoothing%20[Sarkka%2013].pdf)
      * 여러 Filtering 관련 책을 보았지만 이 책이 가장 깔끔하게 모든 필터링 기법에 대해서 설명한다고 생각함.
      * [Basics of Sensor Fusion](reference/papers/Basics%20of%20Sensor%20Fusion%20[sarkka%2020].pdf) 이라는 자료도 있음.  
1. Non-linear optimization (특히, least-square)
    * [Least Squares Optimization: from Theory to Practice by Giorgio Grisetti (2020)](reference/papers/From%20Least%20Squares%20to%20ICP%20%5Bgrisetti%2016%5D.pdf)
      * 최신 Robotica에서 사용하는 비선형 최적화에 대해서 정리한 논문으로 저자의 tutorial slide나 강의 등을 참고하면 좋음. 하지만 하나로만 공부한다면 이 자료를 추천함.
    * [Factor Graphs for Robot Perception by Frank Dellart](reference/papers/Factor%20Graphs%20for%20Robot%20Perception%20[Dellaert%2017].pdf)
        * SLAM-backend에서 많이 사용되는 Gtsam이라는 툴을 만든 교수님의 Technical Report
        * Factor Graph의 기본과 Sparsity, 선형대수 연산으리 빠르게 하기 위한 Parameter ordering 등에 대한 자료
        * SLAM-backend는 이것으로 정리가능하지만 ... 책 한권 분량이며 내용이 어려움... (같이 별도로 공부해볼 사람은 저랑 같이 심화로 ...)
    * [Course on SLAM by Joan Solà (2017)](reference/papers/Course%20on%20SLAM%20%5BSola%2017%5D.pdf)
      * 위의 자료 대신 좀 더 핵심만 깔끔하게 정리한 자료를 본다면 이 자료를 추천함.