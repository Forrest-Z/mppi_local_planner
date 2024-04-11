#ifndef MPPI_LOCAL_PLANNER_MPPI_LOCAL_PLANNER_ROS_HPP
#define MPPI_LOCAL_PLANNER_MPPI_LOCAL_PLANNER_ROS_HPP

#include <string>
#include <memory>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>

#include <mppi_local_planner/tools/path_handler.hpp>
#include <mppi_local_planner/optimizer.hpp>
#include <mppi_local_planner/tools/trajectory_visualizer.hpp>
#include <mppi_local_planner/models/constraints.hpp>
#include <mppi_local_planner/tools/utils.hpp>

#include <dynamic_reconfigure/server.h>
#include <mppi_local_planner/MPPILocalPlannerReconfigureConfig.h>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>


namespace mppi
{

//using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPILocalPlanner
 * @brief Main plugin controller for MPPI LocalPlanner
 */
class MPPILocalPlannerROS : public nav_core::BaseLocalPlanner
{
public:

    MPPILocalPlannerROS();

    ~MPPILocalPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    geometry_msgs::Twist computeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                                                        const geometry_msgs::Twist& robot_speed);

    bool isGoalReached();

    bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

    void setSpeedLimit(const double& speed_limit, const bool& percentage);

    void visualize(nav_msgs::Path transformed_plan);

    bool cancel() { return false; };

    void cleanup();

    void reset();

    void activeVisual();

    void deactiveVisual();

    void reconfigureCB(mppi_local_planner::MPPILocalPlannerReconfigureConfig &config, uint32_t level);


private:
    // external objects (store weak pointers)
    costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
    costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
    tf2_ros::Buffer* tf_; //!< pointer to tf buffer

    boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
    MPPIConfig cfg_; //!< Config class that stores and manages all related parameters

    std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

    base_local_planner::OdometryHelperRos odom_helper_; //!< Provides an interface to receive the current velocity from the robot

    std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot

    std::string global_frame_; //!< The frame in which the controller will run
    std::string robot_base_frame_; //!< Used as the base frame id of the robot
    std::string name_; //!< For use with the ros nodehandle

    Optimizer optimizer_;

    PathHandler path_hander_;

    TrajectoryVisualizer trajectory_visualizer_;

    boost::shared_ptr<dynamic_reconfigure::Server<mppi_local_planner::MPPILocalPlannerReconfigureConfig> >
        dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime

    // flags
    bool goal_reached_;
    bool visualize_;
    bool initialized_; //!< Keeps track about the correct initialization of this class

    double last_vx_;
    double last_wz_;

    bool is_plan_set_;
};

}  // namespace mppi

#endif // MPPI_LOCAL_PLANNER_MPPI_LOCAL_PLANNER_ROS_HPP
