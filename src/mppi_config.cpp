#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi
{

void MPPIConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
{
    nh.param("odom_topic", odom_topic, odom_topic);
    nh.param("map_frame", map_frame, map_frame);

    // controller
    nh.param("controller_frequency", controller.controller_frequency, controller.controller_frequency);
    nh.param("time_steps", controller.time_steps, controller.time_steps);
    nh.param("model_dt", controller.model_dt, controller.model_dt);
    nh.param("batch_size", controller.batch_size, controller.batch_size);
    nh.param("vx_std", controller.vx_std, controller.vx_std);
    nh.param("vy_std", controller.vy_std, controller.vy_std);
    nh.param("wz_std", controller.wz_std, controller.wz_std);
    nh.param("vx_max", controller.vx_max, controller.vx_max);
    nh.param("vx_min", controller.vx_min, controller.vx_min);
    nh.param("vy_max", controller.vy_max, controller.vy_max);
    nh.param("wz_max", controller.wz_max, controller.wz_max);
    nh.param("iteration_count", controller.iteration_count, controller.iteration_count);
    nh.param("temperature", controller.temperature, controller.temperature);
    nh.param("gamma", controller.gamma, controller.gamma);
    nh.param("motion_model", controller.motion_model, controller.motion_model);
    nh.param("visualize", controller.visualize, controller.visualize);
    nh.param("shift_control_sequence", controller.shift_control_sequence, controller.shift_control_sequence);
    nh.param("retry_attempt_limit", controller.retry_attempt_limit, controller.retry_attempt_limit);
    nh.param("look_ahead", controller.look_ahead, controller.look_ahead);
    nh.param("curvature_estimate_point", controller.curvature_estimate_point, controller.curvature_estimate_point);

    // path handler
    nh.param("max_robot_pose_search_dist", path_handler.max_robot_pose_search_dist, path_handler.max_robot_pose_search_dist);
    nh.param("prune_distance", path_handler.prune_distance, path_handler.prune_distance);
    nh.param("transform_tolerance", path_handler.transform_tolerance, path_handler.transform_tolerance);
    nh.param("enforce_path_inversion", path_handler.enforce_path_inversion, path_handler.enforce_path_inversion);
    nh.param("inversion_xy_tolerance", path_handler.inversion_xy_tolerance, path_handler.inversion_xy_tolerance);
    nh.param("inversion_yaw_tolerance", path_handler.inversion_yaw_tolerance, path_handler.inversion_yaw_tolerance);
    nh.param("inversion_locale", path_handler.inversion_locale, path_handler.inversion_locale);

    // noise generator
    nh.param("regenerate", noise_genetrator.regenerate, noise_genetrator.regenerate);

    // trajectory visualizer
    nh.param("tv_trajectory_step", trajectory_visualizer.trajectory_step, trajectory_visualizer.trajectory_step);
    nh.param("tv_time_step", trajectory_visualizer.time_step, trajectory_visualizer.time_step);

    // ackermann condtraints
    nh.param("ac_min_turning_r", ackermann_constraints.min_turning_r, ackermann_constraints.min_turning_r);

    // constraint critic
    nh.param("constraint_enabled", constraint_critic.enabled, constraint_critic.enabled);
    nh.param("constraint_cost_power", constraint_critic.cost_power, constraint_critic.cost_power);
    nh.param("constraint_cost_weight", constraint_critic.cost_weight, constraint_critic.cost_weight);
    nh.param("constraint_vx_acc_max", constraint_critic.vx_acc_max, constraint_critic.vx_acc_max);
    nh.param("constraint_wz_acc_max", constraint_critic.wz_acc_max, constraint_critic.wz_acc_max);

    // goal critic
    nh.param("goal_enabled", goal_critic.enabled, goal_critic.enabled);
    nh.param("goal_cost_power", goal_critic.cost_power, goal_critic.cost_power);
    nh.param("goal_cost_weight", goal_critic.cost_weight, goal_critic.cost_weight);
    nh.param("goal_threshold_to_consider", goal_critic.threshold_to_consider, goal_critic.threshold_to_consider);

    // goal angle critic
    nh.param("goal_angle_enabled", goal_angle_critic.enabled, goal_angle_critic.enabled);
    nh.param("goal_angle_cost_power", goal_angle_critic.cost_power, goal_angle_critic.cost_power);
    nh.param("goal_angle_cost_weight", goal_angle_critic.cost_weight, goal_angle_critic.cost_weight);
    nh.param("goal_angle_threshold_to_consider", goal_angle_critic.threhold_to_consider, goal_angle_critic.threhold_to_consider);

    // prefer forward critic
    nh.param("preforward_enabled", prefer_forward_critic.enabled, prefer_forward_critic.enabled);
    nh.param("preforward_cost_power", prefer_forward_critic.cost_power, prefer_forward_critic.cost_power);
    nh.param("preforward_cost_weight", prefer_forward_critic.cost_weight, prefer_forward_critic.cost_weight);
    nh.param("preforward_threshold_to_consider", prefer_forward_critic.threhold_to_consider, prefer_forward_critic.threhold_to_consider);

    // obstacle critic
    nh.param("obstacle_enabled", obstacle_critic.enabled, obstacle_critic.enabled);
    nh.param("obstacle_cost_power", obstacle_critic.cost_power, obstacle_critic.cost_power);
    nh.param("obstacle_repulsion_weight", obstacle_critic.repulsion_weight, obstacle_critic.repulsion_weight);
    nh.param("obstacle_critical_weight", obstacle_critic.critical_weight, obstacle_critic.critical_weight);
    nh.param("obstacle_consider_footprint", obstacle_critic.consider_footprint, obstacle_critic.consider_footprint);
    nh.param("obstacle_collision_cost", obstacle_critic.collision_cost, obstacle_critic.collision_cost);
    nh.param("obstacle_collision_margin_distance", obstacle_critic.collision_margin_distance, obstacle_critic.collision_margin_distance);
    nh.param("obstacle_near_goal_distance", obstacle_critic.near_goal_distance, obstacle_critic.near_goal_distance);

    // cost critic
    nh.param("cost_enabled", cost_critic.enabled, cost_critic.enabled);
    nh.param("cost_cost_power", cost_critic.cost_power, cost_critic.cost_power);
    nh.param("cost_cost_weight", cost_critic.cost_weight, cost_critic.cost_weight);
    nh.param("cost_critical_cost", cost_critic.critical_cost, cost_critic.critical_cost);
    nh.param("cost_consider_footprint", cost_critic.consider_footprint, cost_critic.consider_footprint);
    nh.param("cost_collision_cost", cost_critic.collision_cost, cost_critic.collision_cost);
    nh.param("cost_near_goal_distance", cost_critic.near_goal_distance, cost_critic.near_goal_distance);

    // path align critic
    nh.param("path_align_enabled", path_align_critic.enabled, path_align_critic.enabled);
    nh.param("path_align_cost_power", path_align_critic.cost_power, path_align_critic.cost_power);
    nh.param("path_align_cost_weight", path_align_critic.cost_weight, path_align_critic.cost_weight);
    nh.param("path_align_max_path_occupancy_ratio", path_align_critic.max_path_occupancy_ratio, path_align_critic.max_path_occupancy_ratio);
    nh.param("path_align_trajectory_point_step", path_align_critic.trajectory_point_step, path_align_critic.trajectory_point_step);
    nh.param("path_align_threshold_to_consider", path_align_critic.threshold_to_consider, path_align_critic.threshold_to_consider);
    nh.param("path_align_offset_from_furthest", path_align_critic.offset_from_furthest, path_align_critic.offset_from_furthest);
    nh.param("path_align_use_path_orientations", path_align_critic.use_path_orientations, path_align_critic.use_path_orientations);
    nh.param("path_align_orientation_weight", path_align_critic.orientation_weight, path_align_critic.orientation_weight);

    // path follow critic
    nh.param("path_follow_enabled", path_follow_critic.enabled, path_follow_critic.enabled);
    nh.param("path_follow_cost_power", path_follow_critic.cost_power, path_follow_critic.cost_power);
    nh.param("path_follow_cost_weight", path_follow_critic.cost_weight, path_follow_critic.cost_weight);
    nh.param("path_follow_offset_from_furthest", path_follow_critic.offset_from_furthest, path_follow_critic.offset_from_furthest);
    nh.param("path_follow_threshold_to_consider", path_follow_critic.threhold_to_consider, path_follow_critic.threhold_to_consider);

    // path angle critic
    nh.param("path_angle_enabled", path_angle_critic.enabled, path_angle_critic.enabled);
    nh.param("path_angle_cost_power", path_angle_critic.cost_power, path_angle_critic.cost_power);
    nh.param("path_angle_cost_weight", path_angle_critic.cost_weight, path_angle_critic.cost_weight);
    nh.param("path_angle_offset_from_furthest", path_angle_critic.offset_from_furthest, path_angle_critic.offset_from_furthest);
    nh.param("path_angle_threshold_to_consider", path_angle_critic.threshold_to_consider, path_angle_critic.threshold_to_consider);
    nh.param("path_angle_max_angle_to_furthest", path_angle_critic.max_angle_to_furthest, path_angle_critic.max_angle_to_furthest);
    nh.param("path_angle_mode", path_angle_critic.mode, path_angle_critic.mode);

    // twirling critic
    nh.param("twirling_enabled", twirling_critic.enabled, twirling_critic.enabled);
    nh.param("twirling_cost_power", twirling_critic.twirling_cost_power, twirling_critic.twirling_cost_power);
    nh.param("twirling_cost_weight", twirling_critic.twirling_cost_weight, twirling_critic.twirling_cost_weight);

    // velocity critic
    nh.param("velocity_enabled", velocity_critic.enabled, velocity_critic.enabled);
    nh.param("velocity_cost_power", velocity_critic.cost_power, velocity_critic.cost_power);
    nh.param("velocity_cost_weight", velocity_critic.cost_weight, velocity_critic.cost_weight);
    nh.param("velocity_threshold_to_consider", velocity_critic.threshold_to_consider, velocity_critic.threshold_to_consider);
    nh.param("velocity_max_vel_x", velocity_critic.max_vel_x, velocity_critic.max_vel_x);
}

void MPPIConfig::reconfigure(mppi_local_planner::MPPILocalPlannerReconfigureConfig& cfg)
{
    std::scoped_lock l(config_mutex_);

    // controller
    controller.time_steps = cfg.time_steps;
    controller.model_dt = cfg.model_dt;
    controller.batch_size = cfg.batch_size;
    controller.vx_std = cfg.vx_std;
    controller.vy_std = cfg.vy_std;
    controller.wz_std = cfg.wz_std;
    controller.vx_max = cfg.vx_max;
    controller.vx_min = cfg.vx_min;
    controller.vy_max = cfg.vy_max;
    controller.wz_max = cfg.wz_max;
    controller.temperature = cfg.temperature;
    controller.gamma = cfg.gamma;
    controller.shift_control_sequence = cfg.shift_control_sequence;
    controller.retry_attempt_limit = cfg.retry_attempt_limit;
    controller.look_ahead = cfg.look_ahead;
    controller.curvature_estimate_point = cfg.curvature_estimate_point;
    controller.visualize = cfg.visualize;

    // path handler
    path_handler.prune_distance = cfg.path_handler_prune_distance;
    path_handler.transform_tolerance = cfg.path_handler_transform_tolerance;
    path_handler.enforce_path_inversion = cfg.path_handler_enforce_path_inversion;
    path_handler.inversion_xy_tolerance = cfg.path_handler_inversion_xy_tolerance;
    path_handler.inversion_yaw_tolerance = cfg.path_handler_inversion_yaw_tolerance;
    path_handler.inversion_locale = cfg.path_handler_inversion_locale;

    // noise generator
    noise_genetrator.regenerate = cfg.noise_generator_regenerate;

    // trajectory visualizer
    trajectory_visualizer.time_step = cfg.trajectory_visualizer_time_step;
    trajectory_visualizer.trajectory_step = cfg.trajectory_visualizer_trajectory_step;

    // ackermann constraints
    ackermann_constraints.min_turning_r = cfg.ackermann_min_turning_r;

    // constraint critic
    constraint_critic.enabled = cfg.constraint_enabled;
    constraint_critic.cost_power = cfg.constraint_cost_power;
    constraint_critic.cost_weight = cfg.constraint_cost_weight;
    constraint_critic.vx_acc_max = cfg.constraint_vx_acc_max;
    constraint_critic.wz_acc_max = cfg.constraint_wz_acc_max;

    // goal critic
    goal_critic.enabled = cfg.goal_enabled;
    goal_critic.cost_power = cfg.goal_cost_power;
    goal_critic.cost_weight = cfg.goal_cost_weight;
    goal_critic.threshold_to_consider = cfg.goal_threshold_to_consider;

    // goal angle critic
    goal_angle_critic.enabled = cfg.goal_angle_enabled;
    goal_angle_critic.cost_power = cfg.goal_angle_cost_power;
    goal_angle_critic.cost_weight = cfg.goal_angle_cost_weight;
    goal_angle_critic.threhold_to_consider = cfg.goal_angle_threshold_to_consider;

    // prefer forward critic
    prefer_forward_critic.enabled = cfg.prefer_forward_enabled;
    prefer_forward_critic.cost_power = cfg.prefer_forward_cost_power;
    prefer_forward_critic.cost_weight = cfg.prefer_forward_cost_weight;
    prefer_forward_critic.threhold_to_consider = cfg.prefer_forward_threshold_to_consider;

    // obstacles critic
    obstacle_critic.enabled = cfg.obstacles_enabled;
    obstacle_critic.cost_power = cfg.obstacles_cost_power;
    obstacle_critic.collision_cost = cfg.obstacles_collision_cost;
    obstacle_critic.critical_weight = cfg.obstacles_critical_weight;
    obstacle_critic.repulsion_weight = cfg.obstacles_repulsion_weight;
    obstacle_critic.consider_footprint = cfg.obstacles_consider_footprint;
    obstacle_critic.near_goal_distance = cfg.obstacles_near_goal_distance;
    obstacle_critic.collision_margin_distance = cfg.obstacles_collision_margin_distance;

    // cost critic (obstacle)
    cost_critic.enabled = cfg.cost_enabled;
    cost_critic.cost_power = cfg.cost_power;
    cost_critic.cost_weight = cfg.cost_weight;
    cost_critic.critical_cost = cfg.cost_critical_cost;
    cost_critic.collision_cost = cfg.cost_collision_cost;
    cost_critic.consider_footprint = cfg.cost_consider_footprint;
    cost_critic.near_goal_distance = cfg.cost_near_goal_distance;

    // path align critic
    path_align_critic.enabled = cfg.path_align_enabled;
    path_align_critic.cost_power = cfg.path_align_cost_power;
    path_align_critic.cost_weight = cfg.path_align_cost_weight;
    path_align_critic.offset_from_furthest = cfg.path_align_offset_from_furthest;
    path_align_critic.threshold_to_consider = cfg.path_align_threshold_to_consider;
    path_align_critic.trajectory_point_step = cfg.path_align_trajectory_point_step;
    path_align_critic.use_path_orientations = cfg.path_align_use_path_orientations;
    path_align_critic.orientation_weight = cfg.path_align_orientation_weight;
    path_align_critic.max_path_occupancy_ratio = cfg.path_align_max_path_occupancy_ratio;

    // path follow critic
    path_follow_critic.enabled = cfg.path_follow_enabled;
    path_follow_critic.cost_power = cfg.path_follow_cost_power;
    path_follow_critic.cost_weight = cfg.path_follow_cost_weight;
    path_follow_critic.offset_from_furthest = cfg.path_follow_offset_from_furthest;
    path_follow_critic.threhold_to_consider = cfg.path_follow_threshold_to_consider;

    // path angle critic
    path_angle_critic.enabled = cfg.path_angle_enabled;
    path_angle_critic.cost_power = cfg.path_angle_cost_power;
    path_angle_critic.cost_weight = cfg.path_angle_cost_weight;
    // mode 0: FORWARD_PREFERENCE
    // mode 1: NO_DIRECTIONAL_PREFERENCE
    // mode 2: CONSIDER_FEASIBLE_PATH_ORIENTATIONS
    path_angle_critic.mode = cfg.path_angle_mode;
    path_angle_critic.offset_from_furthest = cfg.path_angle_offset_from_furthest;
    path_angle_critic.max_angle_to_furthest = cfg.path_angle_max_angle_to_furthest;
    path_angle_critic.threshold_to_consider = cfg.path_angle_threshold_to_consider;

    // twirling critic
    twirling_critic.enabled = cfg.twirling_enabled;
    twirling_critic.twirling_cost_power = cfg.twirling_cost_power;
    twirling_critic.twirling_cost_weight = cfg.twirling_cost_weight;

    // velocity critic
    velocity_critic.enabled = cfg.velocity_enabled;
    velocity_critic.cost_power = cfg.velocity_cost_power;
    velocity_critic.cost_weight = cfg.velocity_cost_weight;
    velocity_critic.threshold_to_consider = cfg.velocity_threshold_to_consider;
    velocity_critic.max_vel_x = cfg.velocity_max_vel_x;
}

} // namespace mppi
