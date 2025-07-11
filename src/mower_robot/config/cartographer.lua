include "map_builder.lua"
include "trajectory_builder.lua"

options.tracking_frame = "base_link"
options.published_frame = "odom"
options.num_point_clouds = 1
options.provide_odom_frame = true
options.use_odometry = true
options.use_nav_sat = false
options.use_landmarks = false
options.num_laser_scans = 1
options.num_multi_echo_laser_scans = 0
options.num_subdivisions_per_laser_scan = 1
options.rangefinder_sampling_ratio = 1.0
options.odometry_sampling_ratio = 1.0
options.fixed_frame_pose_sampling_ratio = 1.0

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
MAP_BUILDER.optimize_every_n_nodes = 20

return options

