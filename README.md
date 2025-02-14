# Google Cartographer를 이용한 SLAM

이 프로젝트는 **Google Cartographer**를 활용하여 SLAM(Simultaneous Localization and Mapping)과 pure_localization을 구현.  
사용 센서: **IMU** 및 **2D LiDAR(Hokuyo)**

---

## 목차
1. [목적](#1-목적)
2. [기능](#2-기능)
   - [매핑](#21-매핑)
   - [맵 저장 (.pgm/.yaml 파일)](#22-맵-저장-pgmyaml-파일)
   - [PBStream 파일 저장](#23-pbstream-파일-저장)
   - [로컬라이제이션 (pure_localization 모드)](#24-로컬라이제이션-pure_localization-모드)
3. [추가 기능: Odometry 처리](#3-추가-기능-odometry-처리)
4. [주요 요구사항 및 파일 구성](#4-주요-요구사항-및-파일-구성)
5. [Lua 파라미터 설명 및 trajectory_to_odom.cpp 정리](#5-lua-파라미터-설명)

---
## 1. 목적

본 프로젝트는 Google Cartographer를 이용한 SLAM을 수행하여, IMU와 2D LiDAR 데이터를 기반으로 로봇의 위치 추정과 실시간 맵 생성.

---

## 2. 기능

### 2.1 매핑
   Mapping을 할 떄에는 `f1tenth bringup`과 `imu stella`를 실행한 상태에서 주행해야 한다. 

#### 2.1.1 bag file 기반 매핑

- **설정:**  

  `Damvi_rosbag_launch.py` 파일 내에서 rosbag 및 pbstream 파일 경로를 수정.

- **실행 명령어:**

```bash
   ros2 launch cartographer_ros Damvi_rosbag_launch.py
```
#### 2.1.2 실제 주행 매핑

- 이때 rosbag 및 pbstream파일에 대한 경로를 아래의 launch.py 파일을 열어 수정한 후, 아래의 명령어를 실행. 

```bash
   ros2 launch cartographer_ros Damvi_carto_launch.py
```
### 2.2 .pgm/.yaml 파일 저장

- 이 때, `Damvi_carto_launch.py`를 실행하여 맵을 생성한다. 이후 맵을 저장할 때에는 `nav2` 패키지의 `map saver`를 사용하여 저장한다. ROS2 기준 map saver 호출 명령어는 아래와 같으며, apt get install로 설치한 nav2 패키지가 있음을 가정한다.  

```bash
   ros2 run nav2_map_server map_saver_cli -f 저장할파일절대경로/저장할파일명
```

### 2.3 .pbstream, 파일 저장

**주의사항**

- 반드시 ros cartographer2의 `Damvi_carto_launch.py`를 실행하고, ***종료하지 않은 상태로 아래의 명령어를 입력한다. 반드시 pbstream이 저장되었는지 경로에 확인한다*** 

```bash
   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '저장할파일절대경로/pbstream파일명'}"
```

- **.pbstream 저장 시 권고 사항:**

- 가능한 많은 loop를 돌 것을 권장한다. 

- 주행할 때 S자 주행을 하면서 여러 각도에 대한 2D scan 데이터를 취득할 경우, pure_localization 모드의 성능이 대폭 향상되니, 가능한 구불구불한 주행 및 직진 주행을 최대한 섞어가는 것을 권장한다.  

### 2.4 로컬라이제이션(pure_localization 모드)

- 실시간 `/scan` 및 `/imu/data` 토픽을 사용할 수 있고, 또는 ROS bag 파일에서 데이터를 처리할 수도 있다. 이에 대한 코드는 서로 다르니 아래 내용을 참조하면 된다.

- cartographer는 pose graph 기반으로 동작하기 때문에, initial pose를 잡는 것이 굉장히 중요하다. 시험한 결과, pbstream 파일이 "괜찮은" 상태라고 가정해도 처음 3초 이내의 주행에 대한 데이터가 수집되어야 localization에 필요한 node 데이터를 취득할 수 있기 때문에 어느 정도의 주행이 필요하다. 이러한 주행을 줄이고 싶으면 pbstream을 최대한 다양한 각도로 얻으면 된다.

- 이외에도 `.lua` 파일에서 몇몇의 파라미터 튜닝이 필요하다. **주석으로 설명한 부분에 대한 변수를 수정하면 되며, 그 외의 변수들은 바꾸어도 크게 의미가 없거나 이미 최적화된 변수이기 때문에 수정하지 않는 것을 권장한다.**

#### 2.4.1 rosbag 기반 pure_localization 에 대한 명령어
- 이때 rosbag 및 pbstream파일에 대한 경로를 아래의 launch.py 파일을 열어 수정한 후, 아래의 명령어를 실행한다. 

```bash
   ros2 launch cartographer_ros Damvi_rosbag_pure_launch.py
```

#### 2.4.2 실제 주행에 대한 명령어
- 앞선 rosbag와 같이, pbstream에 대한 경로를 적절히 변경해준 뒤, 명령어를 실행한다.

```bash
   ros2 launch cartographer_ros Damvi_carto_pure_launch.py
```

##### use_sim_time 관련

- TODO: 추후 업데이트 예정


## 3. 추가 기능:Odometry 처리

- Cartographer는 기본적으로 `odom` topic을 발행하지 않는다. 따라서 연산 처리 속도를 고려하여  C++ 노드(`trajectory_to_odom.cpp`)를 작성하여 아래 기능을 추가하였다. 기존 `node_options.hpp`에 있는 어느 한 bool 값을 true로 변경하여 pose와 관련된 토픽이 발행되도록 변경해주었다.
  ```c++
     bool publish_tracked_pose = true;
  ```
- map → odom, odom → base_link 대한 tf 변환관계를 처리하도록 작성하였다. 간략히 요약하자면, /tracked_pose 토픽과 map<->odom tf 변환관계를 읽은 후에 /tracked_pose 메시지를 저 변환관계를 적용하여 /odom 토픽을 발행하도록 코드를 수정하였다. 
  
## 4. 주요 요구사항 및 파일 구성

- 필수 환경:
   ROS2 Humble. 

- 구성 파일:
  - `Damvi_carto_config.lua`: 매핑 때 쓰이는 lua 파일.
  
  - `Damvi_localization_config.lua`: pure_localization 모드에서 쓰이는 lua 파일. 
  
  - `Damvi_rosbag_launch.py`: 매핑. rosbag을 사용한 데이터 재생을 위한 실행 파일.
  
  - `Damvi_rosbag_pure_launch.py`: 로컬라이제이션. rosbag 기반 모드.  
  
  - `Damvi_carto_launch.py`: 매핑. Cartographer 실행을 위한 주요 실행 파일.
  
  - `Damvi_carto_pure_launch.py`: 로컬라이제이션.


## 5. lua 파라미터 설명

### 5.1 lua 주요 파라미터 정리
아래 파라미터들은 순차적으로 조정하면서 최적의 성능을 도출할 수 있다. 만약 차량이 급격한 드리프트 상황에서 제대로 된 localization을 수행하지 못할 경우에만 5번값(IMU 관련 값)을 키우면 된다. 나머지 파라미터는 기본 설정을 유지하는 것을 권장한다.
 
1. loop closure

  ```lua
      POSE_GRAPH.constraint_builder.translation_weight = 50000.0 
      POSE_GRAPH.constraint_builder.rotation_weight = 50000.0
  ```
- loop closure로 기존 맵 대비 현재 따여진 맵을 얼마나 강하게 정합시킬건지 결정하는 파라미터. 이 값을 작을 경우에는 loop closure가 일어나지 않아서 루프를 돌 때마다 기존 맵에 맞추려는 것이 아니라, **그 값이 작을 수록 새로운 맵을 그리려고 하는 경향이 강해진다**.  (50000.0 이상의 증가는 큰 효과를 주지 않음)

2. loop closure 최적화

  ```lua
      POSE_GRAPH.optimize_every_n_nodes = 1 
  ```
- 노드가 하나라도 쌓이면 즉시 최적화를 수행합니다. (pure_localization 모드에서 초기 위치를 빠르게 찾기 위함)

  ```lua
      POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001 
  ```
- 루프 클로저 시 constraint 반영 비율을 결정.

3. global_sampling_ratio

  ```lua
      POSE_GRAPHPOSE_GRAPH.global_sampling_ratio = 0.00498 
  ```
  - 글로벌 위치 재설정 또는 루프 클로저 검출 시 사용되는 최소 매칭 점수를 설정.(값이 클수록 pure_localization 정확도는 향상되나, 연산 비용이 증가함)

4. Minimum Score (min_score)

  ```lua
      POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
  ```
   - global matching 수행 시, 매칭에 필요한 최소 점수. 


  ```lua
      POSE_GRAPH.constraint_builder.min_score = 0.75 
  ```
   - fast_correlative_scan_matcher 수행 시, 매칭에 필요한 최소 점수. 

5. IMU 관련
  
  ```lua
      TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0 
  ```
   - 지정한 시간(30초) 동안의 데이터를 기반으로 중력 벡터를 산출.(값을 크게 하면 급격한 드리프트 상황에서 안정적인 pose 추정에 도움)

6. Ceres 기반 Scan Matcher

  ```lua
      TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 400.0
      TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 200.0
      TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0
  ```
   - LiDAR 데이터 기반 scan matching 시, occupancy map상의 장애물 분포와 현재 데이터의 정합 강도를 조절.

7. fast_correlative_scan_matcher

  ```lua
     POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.01  -- (단위: meter)
     POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(1.5)  -- (단위: radian)
  ```
   - scan matching을 빠르게(fast)하는 알고리즘이며, 현재 라이더를 기존 맵(pbstream, 서브 맵)과 빠르게 정렬하여 pose 또는 **loop closure**를 검출하는데 사용됨. global/local에서 모두 사용됨. 낮은 해상도에서 높은 해상도로 차례대로 올려가며 세부적으로 scan matching을 수행하며 계산 속도를 줄이고 최적의 매칭을 빠르게 찾음. 
   
   - 탐색 범위에 대한 단위는 meter, radian이다. 각각 선형/회전 변환의 탐지 범위에 대해서 결정함. 

8. real_time_correlative_scan_matcher
   - 로봇의 현재 위치를 실시간으로 추정하기 위해 LiDAR 스캔 데이터를 기반으로 후보 변환들을 평가.(탐색 윈도우 값을 작게 설정하면 높은 정확도를 기대할 수 있다.)

  ```lua
      TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01  -- (meter)
      TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.5)  -- (radian)
      TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 200.0
      TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 200.0
  ```
   - 로봇의 현재 위치를 추정할 때 쓰이는 파라미터 그룹. local-SLAM에서 로봇의 현재 위치 추정을 최적화하기 위해 아래의 파라미터가 쓰임. 즉 pure_localization 모드에서, 주요한 역할을 하는 파라미터임. Real_time correlative_scan_matching은 현재 lidar 스캔 데이터를 기존의 local submap(pbstream 등)과 비교하여 여러 변환(translation/rotation)을 평가하여 가장 높은 상관성을 가진 변환을 선택함. 

   - **선형변환(x, y. 단위: meter). 작을 수록 더 높은 정확도를 보임**. 비교하는 영역이 적기 때문에, 좋은 pbstream이 있다면 조금만 비교해도 **빠르게**기존 맵에 대해서 어디있는지 찾을 수 있음. 즉 모든 후보군들이 정확하다면(pbstream이 정확하다면) 아주 작은 스캔 데이터로만 비교해도 기존 맵에서 어디있는지 알 수 있음. 예를 들어서, 내가 5호관 모든 장소와 사물들을 정확하게 기억한다면, 어느 한 강의실의 책상 다리만 보고도 내가 지금 몇호실에 있는지 파악할 수 있음. 

   - **회전변환(yaw. 단위: radian). 작을 수록 더 높은 정확도를 보임** 앞선 설명과 동일함.
   
   - 변환에 대한 가중치는 크게 늘려도 미미한 효과를 보였음. 오히려 앞선 두 변수의 값을 줄이는 것이 더 극단적인 효과가 있었음. 

   
   #### Fast vs Real-Time Correlative Scan Matcher 비교
      | 특성              | Fast Correlative Scan Matching       | Real-Time Correlative Scan Matching |
      |-------------------|--------------------------------------|-------------------------------------|
      | **속도**          | 더 빠름                              | 비교적 느림                          |
      | **매칭 범위**     | 전역 및 로컬                         | 주로 로컬                            |
      | **계산 복잡도**   | 다단계 계층적 접근 사용               | 단순 탐색 기반 접근                   |
      | **용도**          | 전역 위치 추정, 루프 클로저            | 로컬 위치 추적                        |

9. 기타(아래 변수들은 초기 위치를 잡는데 별 영향을 주지 않음)


  ```lua
      POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
      MAP_BUILDER.num_background_threads = 4
      TRAJECTORY_BUILDER_2D.min_range = 0.1
      TRAJECTORY_BUILDER_2D.max_range = 20.0
      TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
      TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
      TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
      TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
      TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
      TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 5 }
      TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
  ```

- max_constraint_distance: 서브맵 간 최대 거리 (meter)
- num_background_threads: 백그라운드 작업용 스레드 수 (DAMVI는 최대 8 스레드 지원, 안정성을 위해 4로 설정)
- min_range / max_range: LiDAR의 최소/최대 탐지 범위
- missing_data_ray_length: 데이터가 없는 영역에 대한 임의의 거리 값
- adaptive_voxel_filter: 포인트 클라우드 filtering 관련 파라미터 (max_length가 크면 데이터 밀도가 낮아져 계산 속도가 빨라짐)
- voxel_filter_size: voxel filter를 통한 세밀한 맵 표현 (cell = 5cm)
- num_accumulated_range_data: 스캔 데이터 누적 개수 (pure_localization에서는 최소값 권장)
- pure_localization_trimmer: 유지할 서브맵 수 제한 (너무 많으면 재매핑 경향 발생)
- submaps.grid_options_2d.resolution: 맵 해상도 (5cm)


