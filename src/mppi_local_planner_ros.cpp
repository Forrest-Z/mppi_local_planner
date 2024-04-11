#include <mppi_local_planner/mppi_local_planner_ros.hpp>
#include <pluginlib/class_list_macros.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(mppi::MPPILocalPlannerROS, nav_core::BaseLocalPlanner)

namespace mppi
{

MPPILocalPlannerROS::MPPILocalPlannerROS():
    costmap_ros_(NULL)
    , dynamic_recfg_(NULL)
    , initialized_(false)
    , goal_reached_(false)
{

}

MPPILocalPlannerROS::~MPPILocalPlannerROS()
{

}

void MPPILocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    // check if the plugin is already initialized
    if(!initialized_)
    {
        name_ = name;

        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle nh("~/" + name);

        // get parameters of MPPIConfig via the nodehandle and override the default config
        cfg_.loadRosParamFromNodeHandle(nh);

        // reserve some memory for obstacles


        // init other variables
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

        costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

        global_frame_ = costmap_ros_->getGlobalFrameID();
        cfg_.map_frame = global_frame_; // TODO
        robot_base_frame_ = costmap_ros_->getBaseFrameID();


        // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        footprint_spec_ = costmap_ros_->getRobotFootprint();

        // init the odom helper to receive the robot's velocity from odom messages
        odom_helper_.setOdomTopic(cfg_.odom_topic);

        // optimizer initialize
        optimizer_.initialize(&nh, name_, costmap_ros_, cfg_);

        // path_handler initialize
        path_hander_.initialize(&nh, name_, costmap_ros_, tf_);

        // trajectory_visualizer initialize
        trajectory_visualizer_.on_configure(&nh, name_, costmap_ros_->getGlobalFrameID());

        last_vx_ = 0.0;
        last_wz_ = 0.0;

        // setup dynamic reconfigure
        dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<mppi_local_planner::MPPILocalPlannerReconfigureConfig> >(nh);
        dynamic_reconfigure::Server<mppi_local_planner::MPPILocalPlannerReconfigureConfig>::CallbackType cb =
            boost::bind(&MPPILocalPlannerROS::reconfigureCB, this, _1, _2);
        dynamic_recfg_->setCallback(cb);

        // set initialized flag
        initialized_ = true;

        // gx
        is_plan_set_ = false;

        ROS_DEBUG("mppi_local_planner plugin initialized.");
    }
    else
    {
        ROS_WARN("mppi_local_planner has already been initialized, doing nothing.");
    }
}

void MPPILocalPlannerROS::reset()
{
    optimizer_.reset();
}

void MPPILocalPlannerROS::cleanup()
{
    optimizer_.shutdown();
    trajectory_visualizer_.on_cleanup();
    ROS_INFO("Cleaned up MPPI local planner");
}

void MPPILocalPlannerROS::activeVisual()
{
    trajectory_visualizer_.on_activate();
    ROS_INFO("Actived MPPI local planner");
}

void MPPILocalPlannerROS::deactiveVisual()
{
    trajectory_visualizer_.on_deactivate();
    ROS_INFO("Deactived MPPI local planner");
}

void MPPILocalPlannerROS::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
    optimizer_.setSpeedLimit(speed_limit, percentage);
}

bool MPPILocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
{
    if (orig_global_plan.empty() || !initialized_)
    {
        return false;
    }
    is_plan_set_ = false;
    nav_msgs::Path path;
    path.poses = orig_global_plan;
    path.header.frame_id = orig_global_plan.back().header.frame_id;
    path.header.stamp = orig_global_plan.back().header.stamp;
    if (path_hander_.setPath(path))
    {
        is_plan_set_ = true;
        return true;
    }
    return false;
}

void MPPILocalPlannerROS::visualize(nav_msgs::Path transformed_plan)
{
    trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(),
                               "Candidate Trajectories");
//    trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(),
//                               "Optimal Trajectory");
    trajectory_visualizer_.add(optimizer_.arrGetOptimizedTrajectory(),
                               "Optimal Trajectory");
    trajectory_visualizer_.visualize(std::move(transformed_plan));
}

bool MPPILocalPlannerROS::isGoalReached()
{
    if (goal_reached_)
    {
        ROS_INFO("GOAL Reached!");
        cleanup();
        return true;
    }
    return false;
}

bool MPPILocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    if (!initialized_)
    {
        ROS_ERROR("Please initialize before computing velocity");
        return false;
    }

    // gx
    if (!is_plan_set_)
    {
        ROS_ERROR("Please set plan brefore call computeVelocityCommands");
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);

    // prevent the robot into twirling state (nan or inf)
    geometry_msgs::Twist robot_speed;
    robot_speed.linear.x = 0.0;
    robot_speed.linear.y = 0.0;
    robot_speed.linear.z = 0.0;
    robot_speed.angular.x = 0.0;
    robot_speed.angular.y = 0.0;
    robot_speed.angular.z = 0.0;
    geometry_msgs::PoseStamped robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);

    robot_speed.linear.x = robot_vel_tf.pose.position.x;
    robot_speed.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

    if (std::to_string(robot_speed.linear.x) == "-nan" ||
        std::to_string(robot_speed.linear.x) == "nan" ||
        std::to_string(robot_speed.linear.x) == "-inf" ||
        std::to_string(robot_speed.linear.x) == "inf")
    {
        robot_speed.linear.x = last_vx_;
    }
    if (std::to_string(robot_speed.angular.z) == "-nan" ||
        std::to_string(robot_speed.angular.z) == "nan" ||
        std::to_string(robot_speed.angular.z) == "-inf" ||
        std::to_string(robot_speed.angular.z) == "inf")
    {
        robot_speed.angular.z = last_wz_;
    }

    last_vx_ = robot_speed.linear.x;
    last_wz_ = robot_speed.angular.z;

    setSpeedLimit(90.0, true);
    cmd_vel = computeVelocityCommands(robot_pose, robot_speed);

    return true;
}

geometry_msgs::Twist MPPILocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped &robot_pose,
                                                                  const geometry_msgs::Twist &robot_speed)
{
    // loop execution time test
    auto start = std::chrono::system_clock::now();

    // TODO: lock params

    // init velocity command
    geometry_msgs::TwistStamped cmd;
    static uint32_t seq = 0;
    cmd.header.seq = seq++;
    cmd.header.frame_id = robot_base_frame_;
    cmd.header.stamp = ros::Time::now();
    cmd.twist.linear.x = 0;
    cmd.twist.linear.y = 0;
    cmd.twist.angular.z = 0;
    geometry_msgs::TwistStamped zero_cmd = cmd;

    // set goal reached flag
    goal_reached_ = false;

    // set retry attemp flag
    optimizer_.retry_attemp_failed_ = false;
    // transform global plan
    nav_msgs::Path transformed_plan = path_hander_.transformPath(robot_pose);

    // check if global goal is reached
    double dx = transformed_plan.poses.back().pose.position.x; - robot_pose.pose.position.x;
    double dy = transformed_plan.poses.back().pose.position.y; - robot_pose.pose.position.y;
    double delta_orient = utils::normalize_theta(tf2::getYaw(transformed_plan.poses.back().pose.orientation)
                                          - tf2::getYaw(robot_pose.pose.orientation));
    if (fabs(std::sqrt(dx*dx+dy*dy)) < 0.03
        && fabs(delta_orient) < 0.05)
    {
        goal_reached_ = true;
        ROS_INFO("-[MPPI]: Robot arrived at goal");
        return zero_cmd.twist;
    }

    costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

    cmd = optimizer_.evalControl(robot_pose, robot_speed, transformed_plan);

    // address attemped failed
    if (optimizer_.retry_attemp_failed_)
    {
        ROS_INFO("-[MPPI]: Retry attemped failed, set velocity to zero");
        return zero_cmd.twist;
    }

    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                    (end - start).count();
    ROS_INFO("-[MPPI]: Control loop execution time: %ld [ms]", duration);

    if (cfg_.controller.visualize)
    {
        visualize(std::move(transformed_plan));
    }

    return cmd.twist;
}

void MPPILocalPlannerROS::reconfigureCB(mppi_local_planner::MPPILocalPlannerReconfigureConfig &config, uint32_t level)
{
    cfg_.reconfigure(config);
}


} //end namespace mppi




