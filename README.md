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
