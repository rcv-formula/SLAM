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
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.1,
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

-- LiDAR 설정
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Scan Matcher 설정
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

-- IMU 설정
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- 최적화 및 매핑 설정
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- Fast Correlative Matcher 설정
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.1
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.0)

-- Loop Closure 설정
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.0

return options
