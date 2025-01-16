# Google Cartographer를 이용한 SLAM

## 1. 목적
이 프로젝트는 Google Cartographer를 이용한 SLAM(Simultaneous Localization and Mapping)을 수행한다. 사용한 센서는 imu와 2D LiDAR를 사용하였다.

## 2. 기능
### 매핑
#### rosbag 기반일 때
- 이때 rosbag 및 pbstream파일에 대한 경로를 아래의 launch.py 파일을 열어 수정한 후, 아래의 명령어를 실행한다. 
```bash
   ros2 launch cartographer_ros Damvi_rosbag_launch.py
```
#### 일반 주행일 때
- 이때 rosbag 및 pbstream파일에 대한 경로를 아래의 launch.py 파일을 열어 수정한 후, 아래의 명령어를 실행한다. 
```bash
   ros2 launch cartographer_ros Damvi_carto_launch.py
```
#### .pgm/.yaml 파일 저장
- Mapping을 할 떄에는 `f1tenth bringup`과 `imu stella`를 실행한 상태에서 주행해야 한다. 
  이 때, `Damvi_carto_launch.py`를 실행하여 맵을 생성한다. 이후 맵을 저장할 때에는 `nav2` 패키지의 `map saver`를 사용하여 저장한다. ROS2 기준 map saver 호출 명령어는 아래와 같으며, apt get install로 설치한 nav2 패키지가 있음을 가정한다.  
```bash
   ros2 run nav2_map_server map_saver_cli -f <*MAP_NAME*>
```
#### .pbstream, 파일 저장
**주의사항**
** 반드시 ros cartographer2의 `Damvi_carto_launch.py`를 실행하고, ***종료하지 않은 상태로 아래의 명령어를 입력한다. 반드시 pbstream이 저장되었는지 경로에 확인한다*** 
```bash
   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '<*저장경로/PBSTREAM_NAME*>'}"
```
- **.pbstream 저장 시 권고 사항:**
-- 가능한 많은 loop를 돌 것을 권장한다. 
-- 주행할 때 S자 주행을 하면서 여러 각도에 대한 2D scan 데이터를 취득할 경우, pure_localization 모드의 성능이 대폭 향상되니, 가능한 구불구불한 주행 및 직진 주행을 최대한 섞어가는 것을 권장한다.  

### 로컬라이제이션(pure_localization 모드)
- 실시간 `/scan` 및 `/imu/data` 토픽을 사용할 수 있고, 또는 ROS bag 파일에서 데이터를 처리할 수도 있다. 이에 대한 코드는 서로 다르니 아래 내용을 참조하면 된다.
- cartographer는 pose graph 기반으로 동작하기 때문에, initial pose를 잡는 것이 굉장히 중요하다. 시험한 결과, pbstream 파일이 "괜찮은" 상태라고 가정해도 처음 3초 이내의 주행에 대한 데이터가 수집되어야 localization에 필요한 node 데이터를 취득할 수 있기 때문에 어느 정도의 주행이 필요하다. **이러한 주행을 줄이고 싶으면 pbstream을 최대한 다양한 각도로 얻으면 된다.**
- 이외에도 `.lua` 파일에서 몇몇의 파라미터 튜닝이 필요하다. **주석으로 설명한 부분에 대한 변수를 수정하면 되며, 그 외의 변수들은 바꾸어도 크게 의미가 없거나 이미 최적화된 변수이기 때문에 수정하지 않는 것을 권장한다.**
#### rosbag에 대한 명령어
- 이때 rosbag 및 pbstream파일에 대한 경로를 아래의 launch.py 파일을 열어 수정한 후, 아래의 명령어를 실행한다. 
```bash
   ros2 launch cartographer_ros Damvi_rosbag_pure_launch.py
```
#### 실제 주행에 대한 명령어
- 앞선 rosbag와 같이, pbstream에 대한 경로를 적절히 변경해준 뒤, 명령어를 실행한다.
```bash
   ros2 launch cartographer_ros Damvi_carto_pure_launch.py
```
##### use_sim_time 관련

- TODO


## 3. 기존 cartograpehr 코드 외 추가 사항
- **odometry 처리:** Cartographer는 기본적으로 `odom` topic을 발행하지 않는다.
- 따라서 연산 처리 속도를 고려하여  C++ 노드(`trajectory_to_odom.cpp`)를 작성하여 아래 기능을 추가하였다:
  - 기존 `node_options.hpp`에 있는 어느 한 bool 값을 true로 변경하여 pose와 관련된 토픽이 발행되도록 변경해주었다.
  ```c++
     bool publish_tracked_pose = true;
  ```
  - map -> odom, odom->base_link에 대한 tf 변환관계를 처리하도록 코드를 수정하였다. 이에 대한 구체적인 서술은 아래에서 참조하면 된다. 간략히 요약하자면, /tracked_pose 토픽과 map<->odom tf 변환관계를 읽은 후에 /tracked_pose 메시지를 저 변환관계를 적용하여 /odom 토픽을 발행하도록 코드를 수정하였다. 
  
## 4. 재현을 위한 요구사항 및 주요 파일 정리
- ROS2 Humble. 

- 구성 파일:
  - `Damvi_carto_config.lua`: 매핑 때 쓰이는 lua 파일. loop closure 변수 크기가 주요함.
  - `Damvi_localization_config.lua`: pure_localization 모드에서 쓰이는 lua 파일. 구체적인 변수에 대한 설명은 아래의 표에서 자세히 서술. 
  - `Damvi_rosbag_launch.py`: 매핑. ROS bag을 사용한 데이터 재생을 위한 실행 파일.
  - `Damvi_rosbag_pure_launch.py`: 로컬라이제이션. rosbag 기반 모드.  
  - `Damvi_carto_launch.py`: 매핑. Cartographer 실행을 위한 주요 실행 파일.
  - `Damvi_carto_pure_launch.py`: 로컬라이제이션.

- 수정 파일
  - `node_options.hpp`: odometry 처리에서 쓰이는 `/tracked_pose` 토픽을 발행하도록 설정하는 헤더파일. `cartographer_ros/include`에 존재.
    

## 5. lua 파라미터 설명 및 trajectory_to_odom.cpp 정리
### lua 파라미터
- 주요 파라미터 정리

1번부터 순서대로 값을 조절해가며 파라미터를 튜닝하면 된다. 만약 초록색 markerarray가 급격한 드리프트 상황에서 제대로 된 localization을 수행하지 못할 경우에만 5번값을 키우면 되고, 나머지 7개에 대한 파라미터는 작성된대로 조절하면 된다. 주로 등장하는 constraint_builder란 Cartographer의 Pose Graph(포즈 그래프) 구성 요소 중 하나로, 로봇의 위치 추정 및 맵 정렬을 위해 **제약 조건(constraints)**을 생성한다. 
우선 이 파라미터가 절대적인 정답은 아니고, 어쩌면 몇몇 파라미터를 극단적으로 키우거나 줄였을 때 성능이 잘 나올 수도 있다. 신비한 파라미터의 세계..!
 
1. [매우 강력] loop closure
- `POSE_GRAPH.constraint_builder.translation_weight = 50000.0` 
- `POSE_GRAPH.constraint_builder.rotation_weight = 50000.0`
   - loop closure로 기존 맵 대비 현재 따여진 맵을 얼마나 강하게 정합시킬건지 결정하는 파라미터. 이 값을 작을 경우에는 loop closure가 일어나지 않아서 루프를 돌 때마다 기존 맵에 맞추려는 것이 아니라, **그 값이 작을 수록 새로운 맵을 그리려고 하는 경향이 강해진다**. 따라서 가능하면 크게 설정하면 되나, 50000.0보다 키울 경우 어느정도 수렴하여 더이상의 증가는 드라마틱한 효과를 낳지 못한다.
   - translation, rotation은 단어 뜻 그대로의 변수를 의미한다.  

2. [매우강력] loop closure 최적화
- `POSE_GRAPH.optimize_every_n_nodes = 1`
   - cartographer는 노드를 토대로 localization을 수행한다. 따라서 node개 몇개가 쌓인 후에 loop closure를 수행할 지 결정한다. 하지만 pure_localization 모드의 경우, pbstream 파일 안에 수 많은 노드가 이미 존재한다고 가정한다. 우리는 최대한 빨리 초기 위치를 잡아내는 것이 목적이기 때문에, 단 하나의 노드만 잡히더라도 바로 loop closure를 수행하도록 하였다. 값을 키우면 초기위치를 찾는데 오래 걸린다. 0으로 하면 아예 노드를 찾지 않는다. **따라서 1이 최적의 값이기에 가능하면 수정하지 않는 것을 권장한다.**
- `POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001`
   - 이 값은 loop closure를 수행할 시, lua 파일에 있는 constraint 값들을 얼마나 반영할 지를 결정한다. 실험을 통해 얻은 결과로는, 저 값이 작을 수록 initial pose를 잘 찾는 경향이 있었지만, 0.0001에서 가장 좋은 성능을 발휘하였다. 앞선 변수와 마찬가지로 고정하는 것을 권장한다.

3. [매우 강력] global_sampling_ratio
- `POSE_GRAPH.global_sampling_ratio = 0.00498`
   - **반드시 0.0001이나 그 이하 수준으로 값을 조절**. 값이 클수록 높은 pure_localization 정확도를 보임.
   - Cartographer에서 글로벌 위치 재설정(Global Localization) 또는 루프 클로저(Loop Closure) 검출을 수행할 때 사용되는 최소 매칭 점수를 설정하는 파라미터. 이 값은 글로벌 매칭 시 스캔과 서브맵 간의 정합 정확도를 결정하는 기준점수. **연산량을 매우 극단적으로 잡아먹기에 조금씩 키워야만 한다**

4. [매우 강력] min_score
- `POSE_GRAPH.constraint_builder.min_score = 0.75`
   - fast_correlative_scan_matcher 수행 시, 매칭에 필요한 최소 점수. 

5. IMU 관련
- `TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0`
   - 설정한 변수 시간동안(30초)동안의 데이터를 기준으로 중력 벡터가 계산된다. 값을 키우면 급격한 드리프트에 대해서는 정확한 pose를 잡을 수 있다. 낮추게 되면 imu의 노이즈가 민감해진다. **급격한 주행시에는 이 값을 키우면 된다. 하지만 이 정도도 적절해보인다.**

6. Ceres 기반 Scan Matcher
ceres scanmatcher란 구글의 ceres c++ 라이브러리를 기반으로 비선형 최적화를 푸는 ceres solver를 사용하여서 lidar 데이터에 대한 scan matching을 수행하는 툴이다. 
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 400.0`
   - pbstream내의 occupancy map(.pgm)에 존재하는 장애물의 배치에 대해서, 현재 장애물들의 위치와 비교해서 더 강하게 붙으려는 정도. 값은 키웠지만 최적값이라고는 보장하지 못하며, 드라미틱하게 키웠을 떄에도 효과가 미미하였음. 따라서 보통의 영향을 주는 것으로 판단함.
     
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 200.0`
- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0`
   - ceres_scan_matcher는 LiDAR데이터 기반의 이전의 서브맵들과의 비교를 해서, 현재의 LiDAR데이터값과 이전 서브맵들의 데이터에 대해서 어느범위까지 이동/회전을 진행시켜서 비교할지 정하는 weight값. 즉 ceres_scan_matcher로 구한 scan_matching값을 얼마나 강하게 적용할 지 결정한다.

8. [강력] fast_correlative_scan_matcher
- scan matching을 빠르게(fast)하는 알고리즘이며, 현재 라이더를 기존 맵(pbstream, 서브 맵)과 빠르게 정렬하여 pose 또는 **loop closure**를 검출하는데 사용됨. global/local에서 모두 사용됨. 낮은 해상도에서 높은 해상도로 차례대로 올려가며 세부적으로 scan matching을 수행하며 계산 속도를 줄이고 최적의 매칭을 빠르게 찾음. 탐색 범위는 아래 두 변수로 제한함. 
- `POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.01` 
- `POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(1.5)`
   - 두 변수에 대한 단위는 meter, radian이다. 각각 선형/회전 변환의 탐지 범위에 대해서 결정함. 4의 min_score도 이에 매칭되기 위해 필요한 최저 점수를 의미함. 

9. [강력] real_time_correlative_scan_matcher
- 로봇의 현재 위치를 추정할 때 쓰이는 파라미터 그룹. local-SLAM에서 로봇의 현재 위치 추정을 최적화하기 위해 아래의 파라미터가 쓰임. 즉 pure_localization 모드에서, 주요한 역할을 하는 파라미터임. Real_time correlative_scan_matching은 현재 lidar 스캔 데이터를 기존의 local submap(pbstream 등)과 비교하여 여러 변환(translation/rotation)을 평가하여 가장 높은 상관성을 가진 변환을 선택함. 
- `TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01`
   - **선형변환(x, y. 단위: meter). 작을 수록 더 높은 정확도를 보임**. 비교하는 영역이 적기 때문에, 좋은 pbstream이 있다면 조금만 비교해도 **빠르게**기존 맵에 대해서 어디있는지 찾을 수 있음. 즉 모든 후보군들이 정확하다면(pbstream이 정확하다면) 아주 작은 스캔 데이터로만 비교해도 기존 맵에서 어디있는지 알 수 있음. 예를 들어서, 내가 5호관 모든 장소와 사물들을 정확하게 기억한다면, 어느 한 강의실의 책상 다리만 보고도 내가 지금 몇호실에 있는지 파악할 수 있음. 
- `TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.5)`
   - **회전변환(yaw. 단위: radian). 작을 수록 더 높은 정확도를 보임** 앞선 설명과 동일함.
   
   - 아래는 각각 각 변환에 대한 가중치임. 크게 늘려도 미미한 효과를 보였음. 오히려 앞선 두 변수의 값을 줄이는 것이 더 극단적인 효과가 있었음. 
   - `TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 200.0`
   - `TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 200.0`
     ### Fast Correlative Scan Matching vs Real-Time Correlative Scan Matching
      | 특성              | Fast Correlative Scan Matching       | Real-Time Correlative Scan Matching |
      |-------------------|--------------------------------------|-------------------------------------|
      | **속도**          | 더 빠름                              | 비교적 느림                          |
      | **매칭 범위**     | 전역 및 로컬                         | 주로 로컬                            |
      | **계산 복잡도**   | 다단계 계층적 접근 사용               | 단순 탐색 기반 접근                   |
      | **용도**          | 전역 위치 추정, 루프 클로저            | 로컬 위치 추적                        |

11. 기타(아래 변수들은 초기 위치를 잡는데 별 영향을 주지 않음)
- `POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0`
   - 서브 맵 간의 최대 간격. 단위는 meter. 
- `MAP_BUILDER.num_background_threads = 4`
   - 백그라운드에서 실행한 작업의 수. 즉 쓰레드의 수. DAMVI가 최대 8thread까지 가능하나, 안정적인 성능을 위해 4를 선정함. 
- `TRAJECTORY_BUILDER_2D.min_range = 0.1`
   - 라이다 최소 탐지 범위
- `TRAJECTORY_BUILDER_2D.max_range = 20.0`
   - 라이다 최대 담지 범위
- `TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0`
   - 라이다 데이터가 탐지하지 못한 거리에 대해서, 그 거리를 임의로 몇으로 설정할지를 결정
- `TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0`
   - adaptive_voxel_filter(동적으로 lidar입력데이터를 분석하고, 필요에 따라 포인터를 동적 filtering을 수행. DAMVI는 2D lidar이기에 grid cell이긴 하지만 cartographer는 3D lidar까지 지원)가 유지하려는 포인트 간 최대 거리를 설정. 두 점 간의 거리가 max_length보다 크면 해당 점은 필터링 대상에서 제외됨. 이 값이 클수록 pointcloud의 밀도가 낮아지고(부정확함) 계산속도가 빨라짐. 
- `TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200`
   - adaptive_voxel_filter가 filtering후에 최소한으로 유지해야하는 포인트의 개수. filter가 너무 많은 데이터를 제거하지 않도록 보장하는 파라미터. 
- `TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05`
   - voxel_filter가 더 많은 데이터 포인트를 유지하도록 하여 맵에 대한 세밀한 표현이 가능함.
- `TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1`
   - 스캔 데이터를 얼마나 누적할지를 결정. pure_localization에서는 빠른 처리가 필요하기 때문에 최소 누적으로 설정.
- `TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 5,}`
   - 유지할 서브맵의 수. 너무 많으면 두 번째 주행때도 map을 따려는 성향이 강하기에 5정도가 적당. 너무 작으면 제대로 된 localization이 안됨. 
- `TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05`
   - 해상도. 한 cell은 5cm임.

이외에도 options를 비롯한 여러 파라미터가 존재한다. 문의사항은 제 메일로 보내주시길바랍니다. 

### trajectory_to_odom.cpp

