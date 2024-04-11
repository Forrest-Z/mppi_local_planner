#include <mppi_local_planner/tools/path_handler.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/geometry_utils.hpp>
#include <costmap_2d/costmap_2d_ros.h>

namespace mppi
{

void PathHandler::initialize(
    ros::NodeHandle* nh, const std::string & name,
    costmap_2d::Costmap2DROS* costmap,
    tf2_ros::Buffer* buffer)
{
    name_ = name;
    costmap_ = costmap;

    // TODO: get param
    max_robot_pose_search_dist_ = getMaxCostmapDist();
    prune_distance_ = 1.5; //1.5
    transform_tolerance_ = 0.1;
    enforce_path_inversion_ = false; //fasle

    if (enforce_path_inversion_) {
        // TODO: get param
        inversion_xy_tolerance_ = 0.2; // inversion_xy_tolerance
        inversion_yaw_tolerance = 0.4; // inversion_yaw_tolerance
        inversion_locale_ = 0u;
    }
}

std::pair<nav_msgs::Path, PathIterator>
PathHandler::getGlobalPlanConsideringBoundsInCostmapFrame(
    const geometry_msgs::PoseStamped & global_pose)
{
    using geometry_utils::euclidean_distance;

    auto begin = global_plan_up_to_inversion_.poses.begin();

    // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
    // find first point of exceeding distance max_robot_pose_search_dist_
    auto closest_pose_upper_bound =
        geometry_utils::first_after_integrated_distance(
            global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
            max_robot_pose_search_dist_);

    // Find closest point to the robot
    auto closest_point = geometry_utils::min_by(
        begin, closest_pose_upper_bound,
        [&global_pose](const geometry_msgs::PoseStamped & ps) {
            return euclidean_distance(global_pose, ps);
        });

    nav_msgs::Path transformed_plan;
    transformed_plan.header.frame_id = costmap_->getGlobalFrameID();
    transformed_plan.header.stamp = global_pose.header.stamp;

    auto pruned_plan_end =
        geometry_utils::first_after_integrated_distance(
            closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);

    unsigned int mx, my;
    // Find the furthest relevent pose on the path to consider within costmap
    // bounds
    // Transforming it to the costmap frame in the same loop
    for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
         ++global_plan_pose)
    {
        // Transform from global plan frame to costmap frame
        geometry_msgs::PoseStamped costmap_plan_pose;
        global_plan_pose->header.stamp = global_pose.header.stamp;
        global_plan_pose->header.frame_id = global_plan_.header.frame_id;
        transformPose(costmap_->getGlobalFrameID(), *global_plan_pose, costmap_plan_pose);

        // Check if pose is inside the costmap
        if (!costmap_->getCostmap()->worldToMap(
                costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y, mx, my))
        {
            return {transformed_plan, closest_point};
        }

        // Filling the transformed plan to return with the transformed pose
        transformed_plan.poses.push_back(costmap_plan_pose);
    }

    return {transformed_plan, closest_point};
}

geometry_msgs::PoseStamped PathHandler::transformToGlobalPlanFrame(
    const geometry_msgs::PoseStamped & pose)
{
    if (global_plan_up_to_inversion_.poses.empty()) {
        //throw nav2_core::InvalidPath("Received plan with zero length");
        // gx
        ROS_ERROR("Received plan with zero length");
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!transformPose(global_plan_up_to_inversion_.header.frame_id, pose, robot_pose)) {
        //throw nav2_core::ControllerTFError(
        //   "Unable to transform robot pose into global plan's frame");
        ROS_ERROR("Unable to transform robot pose into global plan's frame");
    }

    return robot_pose;
}

nav_msgs::Path PathHandler::transformPath(
    const geometry_msgs::PoseStamped & robot_pose)
{
    // Find relevent bounds of path to use
    geometry_msgs::PoseStamped global_pose =
        transformToGlobalPlanFrame(robot_pose);
    auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose);

    prunePlan(global_plan_up_to_inversion_, lower_bound);

    if (enforce_path_inversion_ && inversion_locale_ != 0u) {
        if (isWithinInversionTolerances(global_pose)) {
            prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
            global_plan_up_to_inversion_ = global_plan_;
            inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
        }
    }

    if (transformed_plan.poses.empty()) {
        //throw nav2_core::InvalidPath("Resulting plan has 0 poses in it.");
        ROS_ERROR("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool PathHandler::transformPose(
    const std::string & frame, const geometry_msgs::PoseStamped & in_pose,
    geometry_msgs::PoseStamped & out_pose) const
{
    if (in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        tf_buffer_->transform(
            in_pose, out_pose, frame,
            ros::Duration(0.8));
        out_pose.header.frame_id = frame;
        return true;
    } catch (tf2::TransformException & ex) {
        //RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
        ROS_ERROR("Exception in transformPose: %s", ex.what());
    }
    return false;
}

double PathHandler::getMaxCostmapDist()
{
    const auto & costmap = costmap_->getCostmap();
    return static_cast<double>(std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY())) *
           costmap->getResolution() * 0.50;
}

bool PathHandler::setPath(const nav_msgs::Path & plan)
{
    if (plan.poses.empty())
    {
        ROS_WARN("set path failed due to input plan is empty");
        return false;
    }
    global_plan_ = plan;
    global_plan_up_to_inversion_ = global_plan_;
    if (enforce_path_inversion_) {
        inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
    }
    return true;
}

nav_msgs::Path & PathHandler::getPath() {return global_plan_;}

void PathHandler::prunePlan(nav_msgs::Path & plan, const PathIterator end)
{
    plan.poses.erase(plan.poses.begin(), end);
}

bool PathHandler::isWithinInversionTolerances(const geometry_msgs::PoseStamped & robot_pose)
{
    // Keep full path if we are within tolerance of the inversion pose
    const auto last_pose = global_plan_up_to_inversion_.poses.back();
    float distance = hypotf(
        robot_pose.pose.position.x - last_pose.pose.position.x,
        robot_pose.pose.position.y - last_pose.pose.position.y);

    float angle_distance = angles::shortest_angular_distance(
        tf2::getYaw(robot_pose.pose.orientation),
        tf2::getYaw(last_pose.pose.orientation));

    return distance <= inversion_xy_tolerance_ && fabs(angle_distance) <= inversion_yaw_tolerance;
}

}  // namespace mppi
