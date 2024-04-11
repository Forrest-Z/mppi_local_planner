#ifndef MPPI_LOCAL_PLANNER_TOOLS_UTILS_HPP
#define MPPI_LOCAL_PLANNER_TOOLS_UTILS_HPP

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include <angles/angles.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <ros/ros.h>

#include <mppi_local_planner/models/optimizer_settings.hpp>
#include <mppi_local_planner/models/control_sequence.hpp>
#include <mppi_local_planner/models/path.hpp>

#include <mppi_local_planner/critic_data.hpp>

#define M_PIF 3.141592653589793238462643383279502884e+00F
#define M_PIF_2 1.5707963267948966e+00F

namespace mppi::utils
{
/**
using xt::evaluation_strategy::immediate;**/

/**
 * @brief Convert data into pose
 * @param x X position
 * @param y Y position
 * @param z Z position
 * @return Pose object
 */
inline geometry_msgs::Pose createPose(double x, double y, double z)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    return pose;
}

/**
 * @brief Convert data into scale
 * @param x X scale
 * @param y Y scale
 * @param z Z scale
 * @return Scale object
 */
inline geometry_msgs::Vector3 createScale(double x, double y, double z)
{
    geometry_msgs::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
}

/**
 * @brief Convert data into color
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param a Alpha component (transparency)
 * @return Color object
 */
inline std_msgs::ColorRGBA createColor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

/**
 * @brief Convert data into a Maarker
 * @param id Marker ID
 * @param pose Marker pose
 * @param scale Marker scale
 * @param color Marker color
 * @param frame Reference frame to use
 * @return Visualization Marker
 */
inline visualization_msgs::Marker createMarker(
    int id, const geometry_msgs::Pose & pose, const geometry_msgs::Vector3 & scale,
    const std_msgs::ColorRGBA & color, const std::string & frame_id, const std::string & ns)
{
    using visualization_msgs::Marker;
    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;

    marker.pose = pose;
    marker.scale = scale;
    marker.color = color;
    return marker;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::TwistStamped toTwistStamped(
    float vx, float wz, const ros::Time & stamp, const std::string & frame)
{
    geometry_msgs::TwistStamped twist;
    twist.header.frame_id = frame;
    twist.header.stamp = stamp;
    twist.twist.linear.x = vx;
    twist.twist.angular.z = wz;

    return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::TwistStamped toTwistStamped(
    float vx, float vy, float wz, const ros::Time & stamp,
    const std::string & frame)
{
    auto twist = toTwistStamped(vx, wz, stamp, frame);
    twist.twist.linear.y = vy;

    return twist;
}

/**
 * @brief Convert path to a tensor
 * @param path Path to convert
 * @return Path tensor
 */
inline models::Path toTensor(const nav_msgs::Path & path)
{
    auto result = models::Path{};
    result.reset(path.poses.size());

    for (size_t i = 0; i < path.poses.size(); ++i) {
        /**
        result.x(i) = path.poses[i].pose.position.x;
        result.y(i) = path.poses[i].pose.position.y;
        result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);**/

        /*****************************************************************************/
        result.arr_x(i) = path.poses[i].pose.position.x;
        result.arr_y(i) = path.poses[i].pose.position.y;
        result.arr_yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
        /*****************************************************************************/

    }

    return result;
}

/**
 * @brief Check if the robot pose is within tolerance to the goal
 * @param pose_tolerance Pose tolerance to use
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within tolerance to the goal
 */
inline bool withinPositionGoalTolerance(
    float pose_tolerance,
    const geometry_msgs::Pose & robot,
    const models::Path & path)
{
    /**
    // recalculate dist to goal
    float goal_dist = 0.0;
    for (int i = 1; i < path.x.shape(0); ++i)
    {
        goal_dist += powf(powf(path.x(i) - path.x(i-1), 2) + powf(path.y(i) - path.y(i-1), 2), 0.5);
    }

    //const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

    if (goal_dist < pose_tolerance) {
        return true;
    }

    return false;**/

    /***************************************************************************************/
    float arr_goal_dist = 0.0;
    for (unsigned int i = 1; i < path.arr_x.rows(); ++i)
    {
        arr_goal_dist += powf(powf(path.arr_x(i) - path.arr_x(i-1), 2) +
                              powf(path.arr_y(i) - path.arr_y(i-1), 2),
                              0.5);
    }
    if (arr_goal_dist < pose_tolerance)
    {
        return true;
    }

    return false;
    /****************************************************************************************/
}

/**
  * @brief normalize
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */
/**
template<typename T>
auto normalize_angles(const T & angles)
{
    auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
    return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}**/

/*****************************************************************************/
inline Eigen::ArrayXf arr_normalize_angles(const Eigen::ArrayXf & angles)
{
    float divisor = 2.0 * M_PI;
    Eigen::ArrayXf theta = (angles + float(M_PI)).unaryExpr(
        [divisor](float x){ return std::fmod(x, divisor);});
    Eigen::Array<bool, Eigen::Dynamic, 1> condition;
    condition = (theta <= float(0.0));
    return condition.select(theta + float(M_PI), theta - float(M_PI));
}
/*****************************************************************************/

/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */
/**
template<typename F, typename T>
auto shortest_angular_distance(
    const F & from,
    const T & to)
{
    return normalize_angles(to - from);
}**/

/*****************************************************************************/
template<typename T>
inline Eigen::ArrayXf arr_shortest_angular_distance(
    const Eigen::ArrayXf & from,
    const T & to)
{
    return arr_normalize_angles(to - from);
}
/*****************************************************************************/

/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of furthest path point reached by a set of trajectories
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
    /**
    const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
    const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());


    const auto dx = data.path.x - traj_x;
    const auto dy = data.path.y - traj_y;

    const auto dists = dx * dx + dy * dy;**/


    /*****************************************************************************/
    auto last_col_index = data.trajectories.arr_x.cols() - 1;

    const Eigen::ArrayXf arr_traj_x = data.trajectories.arr_x.col(last_col_index);
    const Eigen::ArrayXf arr_traj_y = data.trajectories.arr_y.col(last_col_index);

    Eigen::ArrayXXf arr_dx(arr_traj_x.rows(), data.path.arr_x.rows());
    Eigen::ArrayXXf arr_dy(arr_traj_x.rows(), data.path.arr_x.rows());

    for (unsigned int i = 0; i < arr_traj_x.rows(); ++i)
    {
        arr_dx.row(i) = data.path.arr_x - arr_traj_x(i);
        arr_dy.row(i) = data.path.arr_y - arr_traj_y(i);
    }

    const Eigen::ArrayXXf arr_dists = arr_dx.square() + arr_dy.square();
    /*****************************************************************************/


    /**
    size_t max_id_by_trajectories = 0, min_id_by_path = 0;
    float min_distance_by_path = std::numeric_limits<float>::max();
    float cur_dist = 0.0f;

    for (size_t i = 0; i < dists.shape(0); i++) {
        min_id_by_path = 0;
        min_distance_by_path = std::numeric_limits<float>::max();
        for (size_t j = 0; j < dists.shape(1); j++) {
            cur_dist = dists(i, j);
            if (cur_dist < min_distance_by_path) {
                min_distance_by_path = cur_dist;
                min_id_by_path = j;
            }
        }
        max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
    }**/



    /*****************************************************************************/
    unsigned int arr_max_id_by_trajectories = 0, arr_min_id_by_path = 0;
    float arr_min_distance_by_path = std::numeric_limits<float>::max();
    float arr_cur_dist = 0.0f;

    for (unsigned int i = 0; i < arr_dists.rows(); ++i)
    {
        arr_min_id_by_path = 0;
        arr_min_distance_by_path = std::numeric_limits<float>::max();
        for (unsigned int j = 0; j < arr_dists.cols(); ++j)
        {
            arr_cur_dist = arr_dists(i, j);
            if (arr_cur_dist < arr_min_distance_by_path)
            {
                arr_min_distance_by_path = arr_cur_dist;
                arr_min_id_by_path = j;
            }
        }
        arr_max_id_by_trajectories = std::max(arr_max_id_by_trajectories, arr_min_id_by_path);
    }
    //std::cout << arr_min_distance_by_path << std::endl;
    //std::cout << arr_max_id_by_trajectories << std::endl;
    return arr_max_id_by_trajectories;
    /*****************************************************************************/

    /**
    return max_id_by_trajectories;*/
}

/**
 * @brief Evaluate closest point idx of data.path which is
 * nearset to the start of the trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of closest path point at start of the trajectories
 */
inline size_t findPathTrajectoryInitialPoint(const CriticData & data)
{
    /**
    // First point should be the same for all trajectories from initial conditions
    const auto dx = data.path.x - data.trajectories.x(0, 0);
    const auto dy = data.path.y - data.trajectories.y(0, 0);
    const auto dists = dx * dx + dy * dy;

    float min_distance_by_path = std::numeric_limits<float>::max();
    size_t min_id = 0;
    for (size_t j = 0; j < dists.shape(0); j++) {
        if (dists(j) < min_distance_by_path) {
            min_distance_by_path = dists(j);
            min_id = j;
        }
    }**/


    /*****************************************************************************/
    const Eigen::ArrayXf arr_dx = data.path.arr_x - data.trajectories.arr_x(0, 0);
    const Eigen::ArrayXf arr_dy = data.path.arr_y - data.trajectories.arr_y(0, 0);
    const Eigen::ArrayXf arr_dists = arr_dx.square() + arr_dy.square();

    float arr_min_distance_by_path = std::numeric_limits<float>::max();
    unsigned int arr_min_id = 0;
    for (unsigned int i = 0; i < arr_dists.rows(); ++i)
    {
        if (arr_dists(i) < arr_min_distance_by_path)
        {
            arr_min_distance_by_path = arr_dists(i);
            arr_min_id = i;
        }
    }
    return arr_min_id;
    /*****************************************************************************/


    /**
    return min_id;**/
}

/**
 * @brief Evaluate closest point idx of data.path which is
 * nearset to the start of the trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of closest path point at start of the trajectories
 */
inline size_t findPathTrajectoryInitialPoint(const CriticData & data, float & min_dist_by_path)
{
    /**
    // First point should be the same for all trajectories from initial conditions
    const auto dx = data.path.x - data.trajectories.x(0, 0);
    const auto dy = data.path.y - data.trajectories.y(0, 0);
    const auto dists = dx * dx + dy * dy;

    min_dist_by_path = std::numeric_limits<float>::max();
    size_t min_id = 0;
    for (size_t j = 0; j < dists.shape(0); j++) {
        if (dists(j) < min_dist_by_path) {
            min_dist_by_path = dists(j);
            min_id = j;
        }
    }

    return min_id;**/

    /*****************************************************************************/
    const Eigen::ArrayXf arr_dx = data.path.arr_x - data.trajectories.arr_x(0, 0);
    const Eigen::ArrayXf arr_dy = data.path.arr_y - data.trajectories.arr_y(0, 0);
    const Eigen::ArrayXf arr_dists = arr_dx.square() + arr_dy.square();

    float arr_min_distance_by_path = std::numeric_limits<float>::max();
    unsigned int arr_min_id = 0;
    for (unsigned int i = 0; i < arr_dists.rows(); ++i)
    {
        if (arr_dists(i) < arr_min_distance_by_path)
        {
            arr_min_distance_by_path = arr_dists(i);
            arr_min_id = i;
        }
    }
    min_dist_by_path = arr_min_distance_by_path;
    return arr_min_id;
    /*****************************************************************************/
}

/**
 * @brief evaluate path furthest point if it is not set
 * @param data Data to use
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
    if (!data.furthest_reached_path_point) {
        data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
    }
}

/**
 * @brief evaluate path costs
 * @param data Data to use
 */
inline void findPathCosts(
    CriticData & data,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    auto * costmap = costmap_ros->getCostmap();
    unsigned int map_x, map_y;


    /**
    const size_t path_segments_count = data.path.x.shape(0) - 1;
    data.path_pts_valid = std::vector<bool>(path_segments_count, false);
    for (unsigned int idx = 0; idx < path_segments_count; idx++) {
        const auto path_x = data.path.x(idx);
        const auto path_y = data.path.y(idx);
        if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {
            (*data.path_pts_valid)[idx] = false;
            continue;
        }

        switch (costmap->getCost(map_x, map_y)) {
            using namespace costmap_2d; // NOLINT
        case (LETHAL_OBSTACLE):
            (*data.path_pts_valid)[idx] = false;
            continue;
        case (INSCRIBED_INFLATED_OBSTACLE):
            (*data.path_pts_valid)[idx] = false;
            continue;
        case (NO_INFORMATION):
            const bool is_tracking_unknown =
                costmap_ros->getLayeredCostmap()->isTrackingUnknown();
            (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
            continue;
        }

        (*data.path_pts_valid)[idx] = true;
    }**/


    /**************************************************************************/
    const size_t arr_path_segments_count = data.path.arr_x.rows() - 1;
    data.path_pts_valid = std::vector<bool>(arr_path_segments_count, false);
    for (unsigned int idx = 0; idx < arr_path_segments_count; idx++) {
        const auto path_x = data.path.arr_x(idx);
        const auto path_y = data.path.arr_y(idx);
        if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {
            (*data.path_pts_valid)[idx] = false;
            continue;
        }

        switch (costmap->getCost(map_x, map_y)) {
            using namespace costmap_2d; // NOLINT
        case (LETHAL_OBSTACLE):
            (*data.path_pts_valid)[idx] = false;
            continue;
        case (INSCRIBED_INFLATED_OBSTACLE):
            (*data.path_pts_valid)[idx] = false;
            continue;
        case (NO_INFORMATION):
            const bool is_tracking_unknown =
                costmap_ros->getLayeredCostmap()->isTrackingUnknown();
            (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
            continue;
        }

        (*data.path_pts_valid)[idx] = true;
    }

    //for (int i = 0; i < (*data.path_pts_valid).size(); i++)
    //{
    //    std::cout << (*data.path_pts_valid)[i] << std::endl;
    //}
    /*****************************************************************************/
}

/**
 * @brief evaluate path costs if it is not set
 * @param data Data to use
 */
inline void setPathCostsIfNotSet(
    CriticData & data,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!data.path_pts_valid) {
        findPathCosts(data, costmap_ros);
    }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param forward_preference If reversing direction is valid
 * @return Angle between two points
 */
inline float posePointAngle(
    const geometry_msgs::Pose & pose, double point_x, double point_y, bool forward_preference)
{
    float pose_x = pose.position.x;
    float pose_y = pose.position.y;
    float pose_yaw = tf2::getYaw(pose.orientation);

    float yaw = atan2f(point_y - pose_y, point_x - pose_x);

    // If no preference for forward, return smallest angle either in heading or 180 of heading
    if (!forward_preference) {
        return std::min(
            fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
            fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PI))));
    }

    return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param point_yaw Yaw of the point to consider along Z axis
 * @return Angle between two points
 */
inline float posePointAngle(
    const geometry_msgs::Pose & pose,
    double point_x, double point_y, double point_yaw)
{
    float pose_x = pose.position.x;
    float pose_y = pose.position.y;
    float pose_yaw = tf2::getYaw(pose.orientation);

    float yaw = atan2f(point_y - pose_y, point_x - pose_x);

    if (fabs(angles::shortest_angular_distance(yaw, point_yaw)) > M_PI_2) {
        yaw = angles::normalize_angle(yaw + M_PI);
    }

    return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Apply Savisky-Golay filter to optimal trajectory
 * @param control_sequence Sequence to apply filter to
 * @param control_history Recent set of controls for edge-case handling
 * @param Settings Settings to use
 */
inline void savitskyGolayFilter(
    models::ControlSequence & control_sequence,
    std::array<mppi::models::Control, 4> & control_history,
    const models::OptimizerSettings & settings)
{
    /**
    // Savitzky-Golay Quadratic, 9-point Coefficients
    xt::xarray<float> filter = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
    filter /= 231.0;

    const unsigned int num_sequences = control_sequence.vx.shape(0) - 1;

    // Too short to smooth meaningfully
    if (num_sequences < 20) {
        return;
    }

    auto applyFilter = [&](const xt::xarray<float> & data) -> float {
        return xt::sum(data * filter, {0}, immediate)();
    };

    auto applyFilterOverAxis =
        [&](xt::xtensor<float, 1> & sequence,
            const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
        unsigned int idx = 0;
        sequence(idx) = applyFilter(
            {
                hist_0,
                hist_1,
                hist_2,
                hist_3,
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 2),
                sequence(idx + 3),
                sequence(idx + 4)});

        idx++;
        sequence(idx) = applyFilter(
            {
                hist_1,
                hist_2,
                hist_3,
                sequence(idx - 1),
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 2),
                sequence(idx + 3),
                sequence(idx + 4)});

        idx++;
        sequence(idx) = applyFilter(
            {
                hist_2,
                hist_3,
                sequence(idx - 2),
                sequence(idx - 1),
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 2),
                sequence(idx + 3),
                sequence(idx + 4)});

        idx++;
        sequence(idx) = applyFilter(
            {
                hist_3,
                sequence(idx - 3),
                sequence(idx - 2),
                sequence(idx - 1),
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 2),
                sequence(idx + 3),
                sequence(idx + 4)});

        for (idx = 4; idx != num_sequences - 4; idx++) {
            sequence(idx) = applyFilter(
                {
                    sequence(idx - 4),
                    sequence(idx - 3),
                    sequence(idx - 2),
                    sequence(idx - 1),
                    sequence(idx),
                    sequence(idx + 1),
                    sequence(idx + 2),
                    sequence(idx + 3),
                    sequence(idx + 4)});
        }

        idx++;
        sequence(idx) = applyFilter(
            {
                sequence(idx - 4),
                sequence(idx - 3),
                sequence(idx - 2),
                sequence(idx - 1),
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 2),
                sequence(idx + 3),
                sequence(idx + 3)});

        idx++;
        sequence(idx) = applyFilter(
            {
                sequence(idx - 4),
                sequence(idx - 3),
                sequence(idx - 2),
                sequence(idx - 1),
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 2),
                sequence(idx + 2),
                sequence(idx + 2)});

        idx++;
        sequence(idx) = applyFilter(
            {
                sequence(idx - 4),
                sequence(idx - 3),
                sequence(idx - 2),
                sequence(idx - 1),
                sequence(idx),
                sequence(idx + 1),
                sequence(idx + 1),
                sequence(idx + 1),
                sequence(idx + 1)});

        idx++;
        sequence(idx) = applyFilter(
            {
                sequence(idx - 4),
                sequence(idx - 3),
                sequence(idx - 2),
                sequence(idx - 1),
                sequence(idx),
                sequence(idx),
                sequence(idx),
                sequence(idx),
                sequence(idx)});
    };

    // Filter trajectories
    applyFilterOverAxis(
        control_sequence.vx, control_history[0].vx,
        control_history[1].vx, control_history[2].vx, control_history[3].vx);
    applyFilterOverAxis(
        control_sequence.vy, control_history[0].vy,
        control_history[1].vy, control_history[2].vy, control_history[3].vy);
    applyFilterOverAxis(
        control_sequence.wz, control_history[0].wz,
        control_history[1].wz, control_history[2].wz, control_history[3].wz);

    // Update control history
    unsigned int offset = settings.shift_control_sequence ? 1 : 0;
    control_history[0] = control_history[1];
    control_history[1] = control_history[2];
    control_history[2] = control_history[3];
    control_history[3] = {
        control_sequence.vx(offset),
        control_sequence.vy(offset),
        control_sequence.wz(offset)};**/



    /*****************************************************************************/
    // Savitzky-Golay Quadratic, 9-point Coefficients
    Eigen::Array<float, 9, 1> arr_filter;
    arr_filter << -21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0;
    arr_filter /= 231.0f;

    //std::cout << arr_filter << std::endl;

    const unsigned int arr_num_sequences = control_sequence.arr_vx.rows() - 1;

    // Too short to smooth meaningfully
    if (arr_num_sequences < 20) {
        return;
    }

    auto arr_applyFilter = [&](const Eigen::Array<float, 9, 1> & arr_data) -> float {
        return (arr_data * arr_filter).sum();
    };

    auto arr_applyFilterOverAxis =
        [&](Eigen::ArrayXf & arr_sequence,
            const float hist_0, const float hist_1, const float hist_2, const float hist_3) -> void
    {
        Eigen::Array<float, 9, 1> arr_data;

        unsigned int idx = 0;
        arr_data <<
            hist_0,
            hist_1,
            hist_2,
            hist_3,
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 2),
            arr_sequence(idx + 3),
            arr_sequence(idx + 4);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        idx++;
        arr_data <<
            hist_1,
            hist_2,
            hist_3,
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 2),
            arr_sequence(idx + 3),
            arr_sequence(idx + 4);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        idx++;
        arr_data <<
            hist_2,
            hist_3,
            arr_sequence(idx - 2),
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 2),
            arr_sequence(idx + 3),
            arr_sequence(idx + 4);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        idx++;
        arr_data <<
            hist_3,
            arr_sequence(idx - 3),
            arr_sequence(idx - 2),
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 2),
            arr_sequence(idx + 3),
            arr_sequence(idx + 4);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        for (idx = 4; idx != arr_num_sequences - 4; idx++) {
            arr_data <<
                arr_sequence(idx - 4),
                arr_sequence(idx - 3),
                arr_sequence(idx - 2),
                arr_sequence(idx - 1),
                arr_sequence(idx),
                arr_sequence(idx + 1),
                arr_sequence(idx + 2),
                arr_sequence(idx + 3),
                arr_sequence(idx + 4);
            arr_sequence(idx) = arr_applyFilter(arr_data);
        }

        idx++;
        arr_data <<
            arr_sequence(idx - 4),
            arr_sequence(idx - 3),
            arr_sequence(idx - 2),
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 2),
            arr_sequence(idx + 3),
            arr_sequence(idx + 3);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        idx++;
        arr_data <<
            arr_sequence(idx - 4),
            arr_sequence(idx - 3),
            arr_sequence(idx - 2),
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 2),
            arr_sequence(idx + 2),
            arr_sequence(idx + 2);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        idx++;
        arr_data <<
            arr_sequence(idx - 4),
            arr_sequence(idx - 3),
            arr_sequence(idx - 2),
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx + 1),
            arr_sequence(idx + 1),
            arr_sequence(idx + 1),
            arr_sequence(idx + 1);
        arr_sequence(idx) = arr_applyFilter(arr_data);

        idx++;
        arr_data <<
            arr_sequence(idx - 4),
            arr_sequence(idx - 3),
            arr_sequence(idx - 2),
            arr_sequence(idx - 1),
            arr_sequence(idx),
            arr_sequence(idx),
            arr_sequence(idx),
            arr_sequence(idx),
            arr_sequence(idx);
        arr_sequence(idx) = arr_applyFilter(arr_data);
    };

    // Filter trajectories
    arr_applyFilterOverAxis(
        control_sequence.arr_vx, control_history[0].vx,
        control_history[1].vx, control_history[2].vx, control_history[3].vx);
    arr_applyFilterOverAxis(
        control_sequence.arr_vy, control_history[0].vy,
        control_history[1].vy, control_history[2].vy, control_history[3].vy);
    arr_applyFilterOverAxis(
        control_sequence.arr_wz, control_history[0].wz,
        control_history[1].wz, control_history[2].wz, control_history[3].wz);

    // Update control history
    unsigned int arr_offset = settings.shift_control_sequence ? 1 : 0;
    control_history[0] = control_history[1];
    control_history[1] = control_history[2];
    control_history[2] = control_history[3];
    control_history[3] = {
        control_sequence.arr_vx(arr_offset),
        control_sequence.arr_vy(arr_offset),
        control_sequence.arr_wz(arr_offset)};

    /*****************************************************************************/

}

/**
 * @brief Find the iterator of the first pose at which there is an inversion on the path,
 * @param path to check for inversion
 * @return the first point after the inversion found in the path
 */
inline unsigned int findFirstPathInversion(nav_msgs::Path & path)
{
    // At least 3 poses for a possible inversion
    if (path.poses.size() < 3) {
        return path.poses.size();
    }

    // Iterating through the path to determine the position of the path inversion
    for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
        // We have two vectors for the dot product OA and AB. Determining the vectors.
        float oa_x = path.poses[idx].pose.position.x -
                     path.poses[idx - 1].pose.position.x;
        float oa_y = path.poses[idx].pose.position.y -
                     path.poses[idx - 1].pose.position.y;
        float ab_x = path.poses[idx + 1].pose.position.x -
                     path.poses[idx].pose.position.x;
        float ab_y = path.poses[idx + 1].pose.position.y -
                     path.poses[idx].pose.position.y;

        // Checking for the existance of cusp, in the path, using the dot product.
        float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
        if (dot_product < 0.0) {
            return idx + 1;
        }
    }

    return path.poses.size();
}

/**
 * @brief Find and remove poses after the first inversion in the path
 * @param path to check for inversion
 * @return The location of the inversion, return 0 if none exist
 */
inline unsigned int removePosesAfterFirstInversion(nav_msgs::Path & path)
{
    nav_msgs::Path cropped_path = path;
    const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
    if (first_after_inversion == path.poses.size()) {
        return 0u;
    }

    cropped_path.poses.erase(
        cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
    path = cropped_path;
    return first_after_inversion;
}

/**
 * @brief Compare to trajectory points to find closest path point along integrated distances
 * @param vec Vect to check
 * @return dist Distance to look for
 */
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
    auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);
    if (iter == vec.begin() + init) {
        return 0;
    }
    if (dist - *(iter - 1) < *iter - dist) {
        return iter - 1 - vec.begin();
    }
    return iter - vec.begin();
}

/**
 * normalize the angle
 */
inline float normalize_theta(float theta)
{
    if (theta >= -M_PI && theta < M_PI)
        return theta;

    float multiplier = std::floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;

    return theta;
}

// A struct to hold pose data in floating point resolution
struct Pose2D
{
    float x, y, theta;
};

}  // namespace mppi::utils

#endif // MPPI_LOCAL_PLANNER_TOOLS_UTILS_HPP
