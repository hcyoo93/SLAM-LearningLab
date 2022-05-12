# lidarFactor.hpp

## Lidar Edge Factor

`LidarEdgeFactor` 구조체, laserOdometry와 laserMapping에서 모두 사용하는 factor.

### Member variables
```c++
Eigen::Vector3d curr_point, last_point_a, last_point_b;
double s;
```
* `curr_point`: 현재 스캔의 corner 특징점 (`cornerPointsSharp`중 한 점)
* `last_point_a`: 이전 스캔인`cornerPointLessSharp` 중 현재 점과 가장 가까운 점
* `last_point_b`: `last_point_a`와 다른 scan line 중 가장 가까운 점
* `s`: 왜곡(distortion)을 고려한 시간 스케일, 왜곡이 없는 경우는 1.0임.

### Constructor

```c++
LidarEdgeFactor(
    Eigen::Vector3d curr_point_,
    Eigen::Vector3d last_point_a_,
    Eigen::Vector3d last_point_b_, double s_)
    : curr_point(curr_point_), 
    last_point_a(last_point_a_),
    last_point_b(last_point_b_), s(s_) {}
```
* 구조체의 멤버변수의 초기화를 수행함.

### Operator ()

```c++
bool operator()(const T *q, const T *t, T *residual) const;
```
* () 연산자를 오버로딩하여 현재 스캔과 이전 스캔 사이의 상대자세(`q`)와 상대 위치변화(`t`)를 입력받아 잔차를 계산한다.
    * 시간 스케일 s를 고려해서 `q_last_curr`($^{k-1}q_{k}$)과 `t_last_curr`($^{k-1}t_{k}$)을 보간한다.
    * 현재 포인트 `curr_point`를 이전 스캔의 마지막 시점으로 좌표변환한 후, 이 점과 이전 스캔의 포인트 `last_point_a`, `last_point_b'와 다음의 metric으로 잔차 벡터를 계산한다.
    $
    \boldsymbol{r}_{\mathcal{E}} = \frac{{\left(\mathbb{X}^L_{(k,i)} - \bar{\mathbb{X}}^L_{(k-1,j)}\right)\times \left(\mathbb{X}^L_{(k,i)} - \bar{\mathbb{X}}^L_{(k-1,l)}\right)}}{\left\lVert{\bar{\mathbb{X}}^L_{(k-1,j)} - \bar{\mathbb{X}}^L_{(k-1,l)}}\right\rVert}
    $

### Cost Function

```c++
static ceres::CostFunction *Create(
    const Eigen::Vector3d curr_point_,
    const Eigen::Vector3d last_point_a_, 
    const Eigen::Vector3d last_point_b_,
    const double s_);
```
* ceres의 비용함수를 설정한다.
    * 위의 ()연산자에서 정의한 잔차에 기반한 `AutoDiffCostFunction`을 설정한다.
    * 자동으로 비용함수의 차이를 이용하여 그래디언트를 계산하기 때문에 따로 설정할 것은 없다.
    * 자코비안 계산은 따로 서술한다.


## Lidar Plane Factor

Lidar Plane Factor 구조체, laserOdometry에서만 사용하는 factor.

### Member variables
```c++
Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
Eigen::Vector3d ljm_norm;
double s;
```
  * `curr_point`: 현재 스캔의 plane 특징점 (`surfPointsFlat`중 한 점)
  * `last_point_j`: 이전 스캔인 `surfPointLessFlat` 중 현재 점과 가장 가까운 점
  * `last_point_l`: `last_point_j_`와 같은 scan line 중 가장 가까운 점
  * `last_point_m`: `last_point_j_`와 다른 scan line 중 가장 가까운 점
  * `s`: 왜곡(distortion)을 고려한 시간 스케일, 왜곡이 없는 경우는 1.0임
  * `ljm_norm`: 아래 수식의 내적의 우항에 해당하는 normalized 벡터를 계산함.

    $$
     d_{\mathcal{H}} = \left(\mathbb{X}^L_{(k,i)} - \bar{\mathbb{X}}^L_{(k-1,j)}\right)\cdot\frac{{{\left(\mathbb{X}^L_{(k-1,j)} - \bar{\mathbb{X}}^L_{(k-1,l)}\right)\times \left(\mathbb{X}^L_{(k-1,j)} - \bar{\mathbb{X}}^L_{(k-1,m)}\right)}}}{\left\lVert{\left(\mathbb{X}^L_{(k-1,j)} - \bar{\mathbb{X}}^L_{(k-1,l)}\right)\times \left(\mathbb{X}^L_{(k-1,j)} - \bar{\mathbb{X}}^L_{(k-1,m)}\right)}\right\rVert}
     $$

### Constructor
```c++
LidarPlaneFactor(
    Eigen::Vector3d curr_point_, 
    Eigen::Vector3d last_point_j_, 
    Eigen::Vector3d last_point_l_, 
    Eigen::Vector3d last_point_m_, double s_)
    : curr_point(curr_point_), 
    last_point_j(last_point_j_), 
    last_point_l(last_point_l_), 
    last_point_m(last_point_m_), s(s_)
```
* 구조체의 멤버변수의 초기화를 수행하고 `ljm_norm`을 계산함.

### Operator ()
```c++
bool operator()(const T *q, const T *t, T *residual) const;
```
* () 연산자를 오버로딩하여 현재 스캔과 이전 스캔 사이의 상대자세(`q`)와 상대 위치변화(`t`)를 입력받아 잔차를 계산한다.
    * 시간 스케일 s를 고려해서 `q_last_curr`($^{k-1}q_{k}$)과 `t_last_curr`($^{k-1}t_{k}$)을 보간한다.
    * 현재 포인트 `curr_point`를 이전 스캔의 마지막 시점으로 좌표변환한 후, 이 점과 `ljm_norm`을 내적하여 잔차를 구함.

### Cost Function

```c++
static ceres::CostFunction *Create(
    const Eigen::Vector3d curr_point_, 
    const Eigen::Vector3d last_point_j_, 
    const Eigen::Vector3d last_point_l_, 
    const Eigen::Vector3d last_point_m_, const double s_);
```
* ceres의 비용함수를 설정한다.
    * 위의 ()연산자에서 정의한 잔차에 기반한 `AutoDiffCostFunction`을 설정한다.
    * 자동으로 비용함수의 차이를 이용하여 그래디언트를 계산하기 때문에 따로 설정할 것은 없다.
    * 자코비안 계산은 따로 서술한다.


## Lidar Plane Norm Factor

Lidar Plane Norm Factor 구조체, laserMapping에서만 사용하는 factor.
맵 상의 기준 좌표계에서 평면 특징점의 법선 벡터를 추정하고 이를 이용하여 신규로 들어온 데이터가 평면의 방정식에서 얼마나 벗어나는지를 잔차로 사용함.

### Member variables
```c++
Eigen::Vector3d curr_point;
Eigen::Vector3d plane_unit_norm;
double negative_OA_dot_norm;
```
* `curr_point`: 현재 스캔의 plane 특징점 (`surfPointsFlat`중 한 점)
* `plane_unit_norm`: 맵 상에서 연관된 평면의 정규화된 법선 벡터
* `negative_OA_dot_norm`: 정규화된 법선벡터와 평면상의 한 점을 내적 값에 -1을 곱한 것. 평면 방정식의 결과 값이 0이 되되록 한다.
    $ a\cdot x + b\cdot y + c\cdot z + d = 0$

### Constructor
```c++
LidarPlaneNormFactor(
    Eigen::Vector3d curr_point_,
    Eigen::Vector3d plane_unit_norm_,
    double negative_OA_dot_norm_) 
    : curr_point(curr_point_), 
    plane_unit_norm(plane_unit_norm_),
    negative_OA_dot_norm(negative_OA_dot_norm_) {}
```

### Operator ()
```c++
bool operator()(const T *q, const T *t, T *residual) const;
```
* () 연산자를 오버로딩하여 현재 스캔과 이전 스캔 사이의 상대자세(`q`)와 상대 위치변화(`t`)를 입력받아 잔차를 계산한다.
    * 시간 스케일 s를 고려해서 `q_last_curr`($^{k-1}q_{k}$)과 `t_last_curr`($^{k-1}t_{k}$)을 보간한다.
    * 현재 포인트 `curr_point`를 plane 특징점의 평면에 대한 normal 벡터인`plane_unit_norm`과 내적하고 이를 `negative_OA_dot_norm`와 더하여 잔차를 구함.

### Cost Function

```c++
static ceres::CostFunction *Create(
    const Eigen::Vector3d curr_point_, 
    const Eigen::Vector3d plane_unit_norm_,
    const double negative_OA_dot_norm_);
```
* ceres의 비용함수를 설정한다.
    * 위의 ()연산자에서 정의한 잔차에 기반한 `AutoDiffCostFunction`을 설정한다.
    * 자동으로 비용함수의 차이를 이용하여 그래디언트를 계산하기 때문에 따로 설정할 것은 없다.
    * 자코비안 계산은 따로 서술한다.

## Lidar Distance Factor

Lidar Distance Factor 구조체, laserMapping에서만 사용하는 factor.
실제로는 주석처리하여 사용하고 있지 않음. (추후 분석 예정임.)

### Member variables
```c++
Eigen::Vector3d curr_point;
Eigen::Vector3d closed_point;
```
### Constructor
```c++
LidarDistanceFactor(
    Eigen::Vector3d curr_point_,
    Eigen::Vector3d closed_point_)
    : curr_point(curr_point_), 
    closed_point(closed_point_);
```

### Operator ()
```c++
bool operator()(const T *q, const T *t, T *residual) const;
```
* () 연산자를 오버로딩하여 현재 스캔과 이전 스캔 사이의 상대자세(`q`)와 상대 위치변화(`t`)를 입력받아 잔차를 계산한다.
    * 시간 스케일 s를 고려해서 `q_last_curr`($^{k-1}q_{k}$)과 `t_last_curr`($^{k-1}t_{k}$)을 보간한다.
    * 현재 포인트 `curr_point`를 맵 상의 가장 가까운 점인 `closed_point`까지의 거리로 잔차를 구함.

### Cost Function
```c++
static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_);
```
* ceres의 비용함수를 설정한다.
    * 위의 ()연산자에서 정의한 잔차에 기반한 `AutoDiffCostFunction`을 설정한다.
    * 자동으로 비용함수의 차이를 이용하여 그래디언트를 계산하기 때문에 따로 설정할 것은 없다.
    * 자코비안 계산은 따로 서술한다.

