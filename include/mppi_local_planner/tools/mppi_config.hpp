#ifndef MPPI_LOCAL_PLANNER_TOOLS_MPPI_CONFIG_HPP
#define MPPI_LOCAL_PLANNER_TOOLS_MPPI_CONFIG_HPP

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <mutex>

#include <mppi_local_planner/MPPILocalPlannerReconfigureConfig.h>

namespace mppi
{

/**
 * @class MPPIConfig
 * @brief Config class for the mppi_local_planner and its components.
 */
class MPPIConfig
{
public:

    std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
    std::string map_frame; //!< Global planning frame

    struct Controller
    {
        double controller_frequency;
        int time_steps;
        double model_dt;
        int batch_size;
        double vx_std;
        double vy_std;
        double wz_std;
        double vx_max;
        double vx_min;
        double vy_max;
        double wz_max;
        int iteration_count;
        double temperature;
        double gamma;
        std::string motion_model;
        bool visualize;
        bool shift_control_sequence;
        int retry_attempt_limit;
        int  look_ahead;
        int curvature_estimate_point;
    } controller;

    struct PathHandler
    {
        double max_robot_pose_search_dist;
        double prune_distance;
        double transform_tolerance;
        bool enforce_path_inversion;
        double inversion_xy_tolerance; // inversion_xy_tolerance
        double inversion_yaw_tolerance; // inversion_yaw_tolerance
        int inversion_locale;
    } path_handler;

    struct NoiseGenerator
    {
        bool regenerate;
    } noise_genetrator;

    struct TrajectoryVisualizer
    {
        int trajectory_step;
        int time_step;
    } trajectory_visualizer;

    struct AckermannConstraints
    {
        double min_turning_r;
    } ackermann_constraints;

    struct ConstraintCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double vx_acc_max;
        double wz_acc_max;
    } constraint_critic;

    struct GoalCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threshold_to_consider;
    } goal_critic;

    struct GoalAngleCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threhold_to_consider;
    } goal_angle_critic;

    struct PreferForwardCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threhold_to_consider;
    } prefer_forward_critic;

    struct ObstaclesCritic
    {
        bool enabled;
        double cost_power;
        double repulsion_weight;
        double critical_weight;
        double consider_footprint;
        double collision_cost;
        double collision_margin_distance;
        double near_goal_distance;
    } obstacle_critic;

    struct CostCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double critical_cost;
        bool consider_footprint;
        double collision_cost;
        double near_goal_distance;
    } cost_critic;

    struct PathAlignCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double max_path_occupancy_ratio;
        int trajectory_point_step;
        double threshold_to_consider;
        double offset_from_furthest;
        bool use_path_orientations;
        double orientation_weight;
    } path_align_critic;

    struct PathFollowCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double offset_from_furthest;
        double threhold_to_consider;
    } path_follow_critic;

    struct PathAngleCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double offset_from_furthest;
        double threshold_to_consider;
        double max_angle_to_furthest;
        int mode;
    } path_angle_critic;

    struct TwirlingCritic
    {
        bool enabled;
        double twirling_cost_power;
        double twirling_cost_weight;
    } twirling_critic;

    struct VelocityCritic
    {
        bool enabled;
        double cost_power;
        double cost_weight;
        double threshold_to_consider;
        double max_vel_x;
    } velocity_critic;



    MPPIConfig()
    {
        odom_topic = "odom";
        map_frame = "odom";

        // controller
        controller.controller_frequency = 10.0;
        controller.time_steps = 32;
        controller.model_dt = 0.1;
        controller.batch_size = 1800;
        controller.vx_std = 0.2;
        controller.vy_std = 0.0;
        controller.wz_std = 0.4;
        controller.vx_max = 0.7;
        controller.vx_min = -0.11;
        controller.vy_max = 0.0;
        controller.wz_max = 1.0;
        controller.iteration_count = 1;
        controller.temperature = 0.3;
        controller.gamma = 0.015;
        controller.motion_model = "DiffDrive";
        controller.visualize = false;
        controller.shift_control_sequence = false;
        controller.retry_attempt_limit = 3;
        controller.look_ahead = 10;
        controller.curvature_estimate_point = 6;

        // path handler
        path_handler.max_robot_pose_search_dist = 1.5;
        path_handler.prune_distance = 1.5;
        path_handler.transform_tolerance = 0.1;
        path_handler.enforce_path_inversion = false;
        path_handler.inversion_xy_tolerance = 0.2;
        path_handler.inversion_yaw_tolerance = 0.4;
        path_handler.inversion_locale = 0;

        // noise generator
        noise_genetrator.regenerate = true;

        // trajectory visualizer
        trajectory_visualizer.time_step = 3;
        trajectory_visualizer.trajectory_step = 5;

        // ackermann constraints
        ackermann_constraints.min_turning_r = 0.25;

        // constraint critic
        constraint_critic.enabled = true;
        constraint_critic.cost_power = 1;
        constraint_critic.cost_weight = 4.0;
        constraint_critic.vx_acc_max = 0.6;
        constraint_critic.wz_acc_max = 0.8;

        // goal critic
        goal_critic.enabled = true;
        goal_critic.cost_power = 1.0;
        goal_critic.cost_weight = 10.0;
        goal_critic.threshold_to_consider = 1.0;

        // goal angle critic
        goal_angle_critic.enabled = true;
        goal_angle_critic.cost_power = 1.0;
        goal_angle_critic.cost_weight = 5.0;
        goal_angle_critic.threhold_to_consider = 1.0;

        // prefer forward critic
        prefer_forward_critic.enabled = true;
        prefer_forward_critic.cost_power = 1.0;
        prefer_forward_critic.cost_weight = 20.0;
        prefer_forward_critic.threhold_to_consider = 0.3;

        // obstacles critic
        obstacle_critic.enabled = true;
        obstacle_critic.cost_power = 1.0;
        obstacle_critic.collision_cost = 100000.0;
        obstacle_critic.critical_weight = 20.0;
        obstacle_critic.repulsion_weight = 1.5;
        obstacle_critic.consider_footprint = false;
        obstacle_critic.near_goal_distance = 0.5;
        obstacle_critic.collision_margin_distance = 0.1;

        // cost critic (obstacle)
        cost_critic.enabled = true;
        cost_critic.cost_power = 1.0;
        cost_critic.cost_weight = 10.0;
        cost_critic.critical_cost = 300;
        cost_critic.collision_cost = 1000000;
        cost_critic.consider_footprint = true;
        cost_critic.near_goal_distance = 0.5;

        // path align critic
        path_align_critic.enabled = true;
        path_align_critic.cost_power = 1.0;
        path_align_critic.cost_weight = 30.0;
        path_align_critic.offset_from_furthest = 0;
        path_align_critic.threshold_to_consider = 0.1;
        path_align_critic.trajectory_point_step = 4;
        path_align_critic.use_path_orientations = true;
        path_align_critic.orientation_weight = 0.05;
        path_align_critic.max_path_occupancy_ratio = 0.3;

        // path follow critic
        path_follow_critic.enabled = true;
        path_follow_critic.cost_power = 1.0;
        path_follow_critic.cost_weight = 5.0;
        path_follow_critic.offset_from_furthest = 6;
        path_follow_critic.threhold_to_consider = 0.1;

        // path angle critic
        path_angle_critic.enabled = true;
        path_angle_critic.cost_power = 1.0;
        path_angle_critic.cost_weight = 10.0;
        // mode 0: FORWARD_PREFERENCE
        // mode 1: NO_DIRECTIONAL_PREFERENCE
        // mode 2: CONSIDER_FEASIBLE_PATH_ORIENTATIONS
        path_angle_critic.mode = 0;
        path_angle_critic.offset_from_furthest = 4;
        path_angle_critic.max_angle_to_furthest = 0.785398;
        path_angle_critic.threshold_to_consider = 0.1;

        // twirling critic
        twirling_critic.enabled = true;
        twirling_critic.twirling_cost_power = 1.0;
        twirling_critic.twirling_cost_weight = 5.0;

        // velocity critic
        velocity_critic.enabled = true;
        velocity_critic.cost_power = 1.0;
        velocity_critic.cost_weight = 0.2;
        velocity_critic.threshold_to_consider = 0.1;
        velocity_critic.max_vel_x = 0.7;
    }

    /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
    void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

    /**
     * @brief Reconfigure parameters from the dynamic_reconfigure config.
     * Change parameters dynamically (e.g. with <c>rosrun rqt_reconfigure rqt_reconfigure</c>).
     * A reconfigure server needs to be instantiated that calls this method in it's callback.
     * In case of the plugin \e mppi_local_planner default values are defined
     * in \e PROJECT_SRC/cfg/MppiLocalPlannerReconfigure.cfg.
     * @param cfg Config class autogenerated by dynamic_reconfigure according to the cfg-file mentioned above.
     */
    void reconfigure(mppi_local_planner::MPPILocalPlannerReconfigureConfig& cfg);

    /**
   * @brief Return the internal config mutex
   */
    std::mutex& configMutex() {return config_mutex_;}

private:
    std::mutex config_mutex_; //!< Mutex for config accesses and changes

};


} // namespace mppi

#endif // MPPI_LOCAL_PLANNER_TOOLS_MPPI_CONFIG_HPP
