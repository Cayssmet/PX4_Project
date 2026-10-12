-- 包含必要的构建器文件
include "map_builder.lua"           -- 地图构建器配置
include "trajectory_builder.lua"    -- 轨迹构建器配置

-- 主要配置选项
options = {
  map_builder = MAP_BUILDER,                    -- 地图构建器实例
  trajectory_builder = TRAJECTORY_BUILDER,      -- 轨迹构建器实例
  
  -- 坐标系设置
  map_frame = "map",                    -- 地图坐标系名称
  tracking_frame = "laser",             -- 追踪帧（激光雷达坐标系）
  published_frame = "base_link",        -- 发布的基坐标系
  odom_frame = "odom",                  -- 里程计坐标系
  provide_odom_frame = true,            -- 是否提供odom到base_link的TF变换
  publish_frame_projected_to_2d = true, -- 将发布的帧投影到2D平面
  
  -- 传感器使用配置
  use_pose_extrapolator = true,         -- 使用位姿推断器
  -- N10本身已经由Cartographer执行相关性扫描匹配。laser_scan_matcher
  -- 从同一份/scan再次计算的里程计在快速移动或倾斜时会把错误初值反馈回来。
  use_odometry = false,                 -- A/B方案：不使用同源激光里程计
  use_nav_sat = false,                  -- 不使用GPS数据
  use_landmarks = false,                -- 不使用地标数据
  
  -- 激光雷达配置
  num_laser_scans = 1,                  -- 使用1个单线激光雷达
  num_multi_echo_laser_scans = 0,       -- 不使用多回波激光雷达
  num_subdivisions_per_laser_scan = 1,  -- 每束激光扫描的分割数
  num_point_clouds = 0,                 -- 不使用点云数据
  
  -- 时间参数
  lookup_transform_timeout_sec = 0.2,   -- TF变换查找超时时间（秒）
  submap_publish_period_sec = 0.3,      -- 子图发布周期（秒）
  pose_publish_period_sec = 20e-3,       -- 位姿发布周期（秒，50Hz）
  trajectory_publish_period_sec = 30e-3,-- 轨迹发布周期（秒）
  
  -- 数据采样比例
  rangefinder_sampling_ratio = 1.,      -- 测距仪数据采样比例（100%）
  odometry_sampling_ratio = 1.,         -- 里程计数据采样比例（100%）
  fixed_frame_pose_sampling_ratio = 1., -- 固定帧位姿采样比例
  imu_sampling_ratio = 1.,              -- IMU数据采样比例
  landmarks_sampling_ratio = 1.,        -- 地标数据采样比例
}

-- 地图构建器配置：使用2D轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D轨迹构建器参数配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 每个子图包含的激光扫描次数
TRAJECTORY_BUILDER_2D.min_range = 0.2               -- 与雷达一致的最小检测范围（米）
TRAJECTORY_BUILDER_2D.max_range = 7.              -- 4x5m比赛场地：保留边界特征并排除更远场外杂波
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. -- 缺失数据的射线长度（米）
TRAJECTORY_BUILDER_2D.use_imu_data = false         -- 不使用IMU数据

-- 在线相关性扫描匹配配置
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关性扫描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1    -- 线性搜索窗口大小（米）
-- 比赛飞行全程保持固定机头；默认约20度的搜索窗口曾把一帧误匹配成约14度旋转。
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(3.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.   -- 平移变化代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.      -- 强约束不合理的瞬时旋转
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100.  -- 细匹配同样优先保持连续航向

-- 位姿图优化配置
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- Huber损失函数尺度参数
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4  -- 匹配当前 Ceres 编译上限
POSE_GRAPH.optimize_every_n_nodes = 0             -- 短程比赛关闭周期全局优化，避免map跳变
POSE_GRAPH.constraint_builder.min_score = 0.65     -- 约束构建的最小分数阈值

-- 返回配置选项
return options
