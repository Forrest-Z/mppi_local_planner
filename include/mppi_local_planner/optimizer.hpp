#ifndef MPPI_LOCAL_PLANNER_OPTIMIZER_HPP
#define MPPI_LOCAL_PLANNER_OPTIMIZER_HPP

#include <string>
#include <memory>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <mppi_local_planner/models/optimizer_settings.hpp>
#include <mppi_local_planner/motion_models.hpp>
#include <mppi_local_planner/critic_manager.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/models/trajectories.hpp>
#include <mppi_local_planner/models/path.hpp>
#include <mppi_local_planner/tools/noise_generator.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi
{

/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */
class Optimizer
{
public:
    /**
    * @brief Constructor for mppi::Optimizer
    */
    Optimizer() = default;

    /**
   * @brief Destructor for mppi::Optimizer
   */
    ~Optimizer() {shutdown();}


    /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
    void initialize(
        ros::NodeHandle* nh, const std::string & name,
        costmap_2d::Costmap2DROS* costmap_ros, MPPIConfig& cfg);

    /**
   * @brief Shutdown for optimizer at process end
   */
    void shutdown();

    /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
    geometry_msgs::TwistStamped evalControl(
        const geometry_msgs::PoseStamped & robot_pose,
        const geometry_msgs::Twist & robot_speed, const nav_msgs::Path & plan);

    /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
    models::Trajectories & getGeneratedTrajectories();

    /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
    /**
    xt::xtensor<float, 2> getOptimizedTrajectory();**/

    Eigen::ArrayXXf arrGetOptimizedTrajectory();

    /**
   * @brief Set the maximum speed based on the speed limits callback
   * @param speed_limit Limit of the speed for use
   * @param percentage Whether the speed limit is absolute or relative
   */
    void setSpeedLimit(double speed_limit, bool percentage);

    /**
   * @brief Reset the optimization problem to initial conditions
   */
    void reset();

protected:
    /**
   * @brief Main function to generate, score, and return trajectories
   */
    void optimize();

    /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   */
    void prepare(
        const geometry_msgs::PoseStamped & robot_pose,
        const geometry_msgs::Twist & robot_speed,
        const nav_msgs::Path & plan);

    /**
   * @brief Obtain the main controller's parameters
   */
    void getParams(MPPIConfig& cfg);

    /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
    void setMotionModel(const std::string & model);

    /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
    void shiftControlSequence();

    /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
    void generateNoisedTrajectories();

    /**
   * @brief Apply hard vehicle constraints on control sequence
   */
    void applyControlSequenceConstraints();

    /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
    void updateStateVelocities(models::State & state) const;

    /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
    void updateInitialStateVelocities(models::State & state) const;

    /**
   * @brief predict velocities in state using model
   * for time horizon equal to timesteps
   * @param state fill state
   */
    void propagateStateVelocitiesFromInitials(models::State & state) const;

    /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
    void integrateStateVelocities(
        models::Trajectories & trajectories,
        const models::State & state) const;

    /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
    /**
    void integrateStateVelocities(
        xt::xtensor<float, 2> & trajectories,
        const xt::xtensor<float, 2> & state) const;**/

    void integrateStateVelocities(
        Eigen::ArrayXXf & trajectories,
        const Eigen::ArrayXXf & state) const;

    /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
    void updateControlSequence();

    /**
   * @brief Convert control sequence to a twist commant
   * @param stamp Timestamp to use
   * @return TwistStamped of command to send to robot base
   */
    geometry_msgs::TwistStamped
    getControlFromSequenceAsTwist(const ros::Time & stamp);

    /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
    bool isHolonomic() const;

    /**
   * @brief Using control frequence and time step size, determine if trajectory
   * offset should be used to populate initial state of the next cycle
   */
    void setOffset(double controller_frequency);

    /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
    bool fallback(bool fail);

    // gx
    void setCurrentFurthest(CriticData & data);

    float curvatureEstimate(CriticData & data, size_t curvature_estimate_point);

protected:
    ros::NodeHandle* nh_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D * costmap_;
    std::string name_;

    std::shared_ptr<MotionModel> motion_model_;

    CriticManager critic_manager_;
    NoiseGenerator noise_generator_;

    models::OptimizerSettings settings_;

    models::State state_;
    models::ControlSequence control_sequence_;
    std::array<mppi::models::Control, 4> control_history_;
    models::Trajectories generated_trajectories_;
    models::Path path_;
    /**
    xt::xtensor<float, 1> costs_;**/

    //Eigen::Tensor<float, 1> eig_costs_;
    Eigen::ArrayXf arr_costs_;

    float goal_tolerance_;
    // gx
    size_t current_furthest_;
    size_t curvature_estimate_point_;
    size_t look_ahead_;

    CriticData critics_data_ =
        {state_, generated_trajectories_, path_, arr_costs_, settings_.model_dt, false, nullptr,
         std::nullopt, std::nullopt, goal_tolerance_, current_furthest_};  /// Caution, keep references

public:
    bool retry_attemp_failed_;
};

}  // namespace mppi

#endif // MPPI_LOCAL_PLANNER_OPTIMIZER_HPP
