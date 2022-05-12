# laserMapping.cpp
## Logic
### Input
* $${\mathcal{Q}}_{k-1}$$: k-1번째 sweep까지 누적된 맵 상의 point cloud
* $$\!^{w}{\mathcal{T}}_{k-1}$$: k-1번째 sweep이 끝난 시점에서의 라이다 포즈 ($$@t_{k}$$)
* $$\bar{\mathcal{P}}_{k}$$: 라이다에서 k번째 sweep 동안 누적된 point cloud를 k번째 sweep이 끝나는 시점으로 투영한 점들
* $$\!^{L}{\mathcal{T}}_{k}$$: odometry를 통해 만들어진 상대 포즈, 정확히는 $$\!^{L_{k-1}}{\mathcal{T}}_{L_{k}}$$ 를 의미함.

### Output

*   $$\!^{w}{\mathcal{T}}_{k}$$: k번째 sweep이 끝난 시점의 라이다 포즈 ($$@t_{k+1}$$)
*   $${\mathcal{Q}}_{k}$$: 라이다에서 k번째 sweep에서 들어온 point cloud $$\bar{\mathcal{P}}_k$$ 를 world 좌표계로 투영한 점들


### Algorithm

1.  특징점 추출 (odometry 보다 10배 많이 사용)
2.  맵 상의 특징점과 라이다에서 들어온 특징점 사이의 연관관계 찾기
    1.  $${\mathcal{Q}}_{k-1}$$ 을 10m 단위의 큐브로 저장한다.
    2.  $$\bar{\mathcal{P}}_{k}$$ 를 world 좌표계로 변환하여 $$\bar{\mathcal{Q}}_{k}$$ 를 구한다.
    3.  $${\mathcal{Q}}_{k-1}$$ 과 $$\bar{\mathcal{Q}}_{k}$$ 가 교차하는 지점을 찾고 그 영역에 해당하는 특징점들을 $$\mathcal{S}^{\prime}$$ 에 저장하고 3D KD-tree를 만든다. 해당 특징점은 edge line과  plane patch를 유지한다.
    4.  $$\mathcal{S}^{\prime}$$ 에서 특징점을 하나 씩 추출하여 공분산 $$\boldsymbol{M}$$ 과  그 것의 eigien value $$V$$ 와 eigen vector $$E$$ 를 계산한다.
        *   $$V_1 \gg V_2, V_3$$ 의 특성을 가지면 edge point로 생각하고 2개의 점을 선택하고 거리 $$d_{\mathcal{E}} $$ 를 계산한다.
        *   $$V_1, V_2 \gg V_3$$ 의 특성을 가지면 plane point로 생각하고 3개의 점을 선택하고 거리 $$d_{\mathcal{H}}$$ 를 계산한다.
3.  최적화 문제를 풀어서 $$\!^{w}{\mathcal{T}}_{k}$$ 를 구한다.
4.  $$\bar{\mathcal{Q}}_{k}$$ 를 맵에 등록하여 $${\mathcal{Q}}_{k}$$를 구한다. 맵에 등록하기 전, 사이즈를 줄이기 위해 voxel grid filter를 돌린다.

## Global Variables
* `q_w_curr`: $$^{w}q_{L_{k}}$$, map 기준 좌표계 대비 현재 로컬 좌표계의 rotation
* `t_w_curr`: $$^{w}t_{w,L_{k}}$$, map 기준 좌표계 대비 현재 로컬 좌표계의 translation
* `q_wmap_wodom`: $$^{w}q_{o}$$, map의 기준 좌표계와 odometry의 기준좌표계 사이의 rotation
* `t_wmap_wodom`: $$^{w}t_{w,o}$$, map의 기준 좌표계와 odometry의 기준좌표계 사이의 translation
* `q_wodom_curr`: $$^{o}q_{L_{k}}$$, odometry 기준 좌표계 대비 현재 로컬 좌표계의 rotation
* `t_wodom_curr`: $$^{o}t_{o,L_{k}}$$, odometry 기준 좌표계 대비 현재 로컬 좌표계의 translation
`laserCloudWidth`, `laserCloudHeight`, `laserCloudDepth`
### Input
```c++
// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
```
* $${\mathcal{Q}}_{k-1}$$: k-1번째 sweep까지 누적된 맵 상의 점들
* $$\bar{\mathcal{Q}}_{k}$$: k번째 sweep에서 라이다에서 얻어진 점들을 odometry에서 구한 포즈 ($$^{L}T_{k}$$)를 이용하여 맵 상에 투영한 점들
* `laserCloudCornerLast`: 지난 sweep의 corner 특징점들, $$\bar{\mathcal{E}}_{k-1} \subset \bar{\mathcal{P}}_{k-1}$$
* `laserCloudSurfLast`: 지난 sweep의 plane 특징점들, $$\bar{\mathcal{H}}_{k-1} \subset \bar{\mathcal{P}}_{k-1}$$
* `laserCloudCornerFromMap`: 센서가 아니라 맵에 있는 corner 특징점들, $$\tilde{\mathcal{E}}_{k-1} \subset {\mathcal{Q}}_{k-1}$$
* `laserCloudSurfFromMap`: 센서가 아니라 맵에 있는 plane 특징점들, $$\tilde{\mathcal{H}}_{k-1} \subset {\mathcal{Q}}_{k-1}$$
* `kdtreeCornerFromMap`: Kd-Tree로 구분된 맵 상의 corner 특징점
* `kdtreeSurfFromMap`: Kd-Tree로 구분된 맵 상의 plane 특징점

### Output

```c++
// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
```
* `laserCloudCornerArray`: 라이다에서 들어온 corner 특징점들을 큐브로 구분하여 배치한 배열
* `laserCloudSurfArray`: 라이다에서 들어온 plane 특징점들을 큐브로 구분하여 배치한 배열
* cube 인덱스에서 1m 단위로 나누어 점들을 등록한다?
* (w x h)xd 로 포인터 위치가 계산된다.
### Input & Ouput
```c++
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
```
* `laserCloudFullRes`: 한 sweep에 들어있는 모든 점들 (local -> global)

```c++
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
```
* `laserCloudSurround`: all visuable cube points
## Point Association

```c++
void transformAssociateToMap();
void transformUpdate();
void pointAssociateToMap(PointType const *const pi, PointType *const po);
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);
```
* `transformAssociateToMap` 함수
    * map의 기준 좌표계 대비 현재 좌표계의 자세변환 행렬을 구한다.
    $$ \!^{w}T_{L_{k}} = \!^{w}T_{o} \cdot\ \!^{o}T_{L_{k}} $$

* `transformUpdate` 함수
    * map의 기준 좌표계 대비 odometry 기준 좌표계의 자세변환 행렬을 구한다.
    $$ \!^{w}T_{o} = \!^{w}T_{L_{k}} \cdot\ \!^{o}T_{L_{k}}^{-1} $$

* `pointAssociateToMap` 함수
    * 현재 좌표계의 점을 map의 기준 좌표계 {w}로 변환한다.
    * intensity는 그냥 전달한다.
    $$ \!^{w}p_{k} = \!^{w}R_{L_{k}}\!^{L_{k}}p_{k} + \!^{w}t_{w,L_{k}} $$
* `pointAssociateTobeMapped` 함수
    * map 기준 좌표계 {w}의 점을 현재의 로컬 좌표계로 변환한다.
    * intensity는 그냥 전달한다.
    $$ \!^{L_k}p = \!^{w}R_{L_{k}}^{-1}\left(\!^{w}p - \!^{w}t_{w,L_{k}}\right) $$


## Data Handler
```c++
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2);
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2);
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2);
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);
```

* `laserCloudCornerLastHandler` 함수
    * 입력을 `cornerLastBuf`에 넣어준다.
* `laserCloudSurfLastHandler` 함수
    * 입력을 `surfLastBuf`에 넣어준다.
* `laserCloudFullResHandler` 함수
    * 입력을 `fullResBuf`에 넣어준다.
* `laserOdometryHandler` 함수
    * 입력을 `odometryBuf`에 넣어준다.
    * `q_wodom_curr`과 `t_wodom_curr`을 입력에서 읽어들이고 `q_w_curr`과 `t_w_curr`을 계산한다.
    * `q_w_curr`과 `t_w_curr`을 publish한다.


## Process Mapping
```c++
void process();
```
* `cornerLastBuf`, `surfLastBuf`, `fullResBuf`, `odometryBuf` 버퍼가 비어있는지 조사한다.

* 비어있지 않으면, `cornerLastBuf` 버퍼와 나머지 버퍼의 타임스탬프를 비교하여 처리할 데이터를 선정한다.
    * 코드 상에서는 센서 버퍼 (`cornerLastBuf`, `surfLastBuf`, `fullResBuf`)의 타임스탬프 중 하나라도 `odometryBuf`의 타임스탬프와 일치해야만 루프에 들어간다.
    * 센서 버퍼의 데이터를 pcl 라이브러리의 point cloud 타입으로 복사한다.
    * odometry 버퍼의 데이터를 읽어들여 `q_wodom_curr`와 `t_wodom_curr`에 설정한다.

* 맵 상에서 현재 위치를 기반으로 탐색 영역인 큐브를 설정한다.
    * 맵을 큐브로 구분히여 현재 위치를 `centerCubeI`, `centerCubeJ`, `centerCubeK`로 인덱싱한다.
        * 큐브의 원점을 중심으로 구분하며, 맵의 원점을 포함하는 큐브의 원점은 맵의 원점과 일치한다.
        * 큐브의 크기는 x, y, z 방향으로 각각 50m이다.
        * 이 인덱싱은 큐브의 크기를 고려하여 동적으로 offset된다. 초기 offset 값은 해당 배열 크기의 절반으로 한다.
        * 위치가 음수이면 인덱스에서 1을 뺀다. (C언어 계열 라운딩 특성을 반영함.)

    * 큐드 인덱스에 대한 조정을 수행한다.
        * 3 < `centerCubeI` < laserCloudWidth - 3`가 정상 영역이다. 
        * lower boundary에 있으면, 맵이 negative 방향으로 확장된다는 것을 의미한다.
            * `laserCloudCornerArray`내의 width에 해당하는 데이터 단위로 데이터를 원형으로 오른쪽으로 이동시킨다. (인덱스가 작은 쪽을 큰 쪽으로 복사, 왜 하는지는 잘 모르겠다.)
            * `centerCubeI`를 1 증가시키고, centerWidth도 1 증가시킨다.
        * upper boundary에 있으면, 맵이 positive 방향으로 확장된다는 것을 의미한다.
            * `laserCloudCornerArray`내의 width에 해당하는 데이터 단위로 데이터를 원형으로 왼쪽으로 이동시킨다. (인덱스가 큰 쪽을 작은 쪽으로 복사, 왜 하는지는 잘 모르겠다.)
            * `centerCubeI`를 1 감소시키고, centerWidth도 1 감소시킨다.
        * `centerCubeJ`, `centerCubeK`에 대해서도 똑같이 수행한다.
* 큐브 인덱스를 기반으로 각 축 방향으로 5개의 큐브에 대해서 탐색을 수행한다.(총 125개)
    * 큐브 영역 안에 있으면, 그 인덱스를 ValidIndex에 저장하고 ValidNum을 증가시킨다.
    * laserCloudCornerFromMap, laserCloudSurfFromMap에 valid data를 복사한다.

    * laserCloudCornerLast를 downSizeFilterCorner를 해서 laserCloudCornerStack에 저장함. (local frame의 데이터임.)
    * laserCloudSurfLast를 downSizeFilterSurf를 해서 laserCloudSurfStack에 저장함. (local frame의 데이터임.)
        * 맵 준비 완료
    * laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50이면
        * 맵 포인트로 kdtree 생성
        * 2번 iteration 수행
            * laserCloudCornerStack에 있는 점들을 처리
                * pointAssociateToMap: world frame으로 좌표 변환
                * kdtree에서 가장 가까운 점 5개 검색
                * 가장 가까운 점까지의 거리가 1보다 작으면, 위의 5개의 평균을 구해서 center로 저장하고, 공분산을 구한다.
                * 구한 분산을 통해 line 특성을 가지고 있는지 확인한다.
                    * 공분산을 eigen decomposition하고 eigen value와 vector를 구한다.
                    * 가장 큰 eigen value가 두번째로 큰 eigen value에 비해서 3배 이상 크면, line으로 판단한다.
                * center를 기준으로 0.1m 간격의 line 특징점을 생성하고 이를 LidarEdgeFactor로 추가한다.

            * laserCloudSurfStack에 있는 점들을 처리
                * pointAssociateToMap: world frame으로 좌표 변환
                * kdtree에서 가장 가까운 점 5개 검색
                * 가장 가까운 점까지의 거리가 1보다 작으면, 5개의 점으로 평면을 추정하고 법선 벡터를 구한다.
                * 법선 벡터를 정규화하고 이를 이용해서 5개의 점 중에서 오차가 0.2보다 큰 것이 있으면 유효한 평면이 아닌 것으로 판단한다.
                   * LidarPlaneNormFactor로 추가한다.
            * data association을 마치고, ceres solver를 수행해 최적화를 수행한다.
* 최적화를 통해 구한 odometry 포즈를 업데이트한다.
* 현재 들어온 포인트를 맵에 추가한다.
* downsizeFilter를 수행한다.
* 맵을 publish한다.
    * 5 frame마다 `laserCloudSurround`을 publish한다.
    * 20 frame마다 `laserCloudMap`을 publish한다.
    * 매 frame마다 `laserCloudFullRes`을 publish한다.
* 포즈를 publish한다.
    * `odomAftMapped`을 publish한다.
    * `laserAfterMappedPath`을 publish한다.

## Main
```c++
int main(int argc, char **argv);
```