include "map_builder.lua"
include "trajectory_builder.lua"

-- 기본 설정
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",
  
  provide_odom_frame = true,
  use_odometry = false,
  
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  publish_to_tf = true,
  
  lookup_transform_timeout_sec = 0.2,

  -- Pure Localization 모드 관련 변수
  -- ◆ 서브맵 데이터를 주기적으로 퍼블리시하는 주기 (초 단위)
  submap_publish_period_sec = 0.1,
  -- ◆ 위치 데이터를 퍼블리시하는 주기 (초 단위)
  pose_publish_period_sec = 0.05,
  -- ◆ 로봇의 경로 데이터를 퍼블리시하는 주기 (초 단위)
  trajectory_publish_period_sec = 0.1,
  
  -- 센서 샘플링 관련 변수 (Pure Localization에 간접적으로 사용)
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D Trajectory 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
  -- 해상도 설정, 0.1 = 10cm 단위. 현재는 5cm
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 

-- Pure Localization 모드 관련 변수
-- ◆ Pure Localization 모드 활성화 (기존 맵을 활용하여 로컬라이제이션만 수행). 0에 가까울수록 높은 정렬 정확도. 0초이후에 constraint serach 수행
  -- 값을 키우면 스캔이 쌓인 뒤에 매칭을 수행하기때문에, 초기에 틀어짐은 있을수는 있어도 맵이 올바르게 맞춰짐.  
  POSE_GRAPH.global_constraint_search_after_n_seconds = 0


-- ◆ Pure Localization 트리머 설정
--    로컬라이제이션 수행 시, 유지할 서브맵의 최대 개수를 제한
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 20,
}

-- ◆ 전역 매칭 시 필요한 최소 점수,
  -- 전역 매칭을 통해 새로운 constraint를 추가할 때, 매칭 점수가 이 값 이상이어야만 인정
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
-- ◆ 로컬 매칭에서 필요한 최소 점수
  -- Pure Localization 모드에서, local 스캔을 매칭하여 위치 보정을 할 때 이 점수를 만족해야 매칭이 유효
  POSE_GRAPH.constraint_builder.min_score = 0.75 --0.65

-- ◆ 누적 레이저 스캔 데이터를 Submap에 추가하는 단위
  -- 한 번의 스캔 매칭을 수행하기 위해 몇 개의 스캔(레인지데이터)을 누적할지 결정
  TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ◆ 전역 서브맵 매칭을 위한 탐색 창
  -- 전역 매칭(또는 큰 오프셋이 있을 때) 시, “평면상에서 몇 m 범위까지 후보를 탐색할 것인가”를 정함
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 20.0 --1
  -- 전역 매칭 시, 회전(각도)에 대해 몇 도(deg) 범위까지 탐색할지 정함
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(50.0) -- 20

-- Pure Localization 관련 변수가 아닌 일반 변수 정렬
-- LiDAR 설정
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Scan Matcher 설정
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 10.0 --0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.0) --20.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 15.0 --10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.5 --1.0

-- IMU 설정. 급격한 steering이 발생할 경우에는 timeconstant와 rotation weight을 키우면 됨. 
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 80.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 90.0 --40.0

-- 최적화 및 매핑 설정
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 3 --10

-- ◆ 전역 매칭을 위한 Submap 간 최대 거리
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- Loop Closure 설정
  -- Loop Closure(또는 전역 constraint)에서 “새로운 서브맵 매칭이 기존 맵과 부딪혔을 때, 그 매칭을 얼마나 강하게(가중치 높게) 반영할지” 결정
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 30.0 --1.0 
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 10.0 --1.0

-- 전역 샘플링 설정
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
  -- global_sampling_ratio는 전역 매칭(큰 오프셋) 후보를 얼마나 자주 탐색할지 결정. 0.005(=0.5%)면 200장 중 1장 꼴로 전역 매칭 시도
POSE_GRAPH.global_sampling_ratio = 0.01

return options
