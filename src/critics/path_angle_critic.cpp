#include <mppi_local_planner/critics/path_angle_critic.hpp>

#include <math.h>

namespace mppi::critics
{
/**
using xt::evaluation_strategy::immediate;**/

void PathAngleCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;

    float vx_min = -0.1;
    vx_min = cfg.controller.vx_min;

    if (fabs(vx_min) < 1e-6) {  // zero
        reversing_allowed_ = false;
    } else if (vx_min < 0.0) {   // reversing possible
        reversing_allowed_ = true;
    }

    enabled_ = cfg.path_angle_critic.enabled;
    power_ = cfg.path_angle_critic.cost_power;
    weight_ = cfg.path_angle_critic.cost_weight;
    offset_from_furthest_ = cfg.path_angle_critic.offset_from_furthest;
    threshold_to_consider_ = cfg.path_angle_critic.threshold_to_consider;
    max_angle_to_furthest_ = cfg.path_angle_critic.max_angle_to_furthest;

    // TODO: get param
    //offset_from_furthest_ = 4; // 4
    //power_ = 1;
    //weight_ = 10.0; // 2.2 // 10
    //threshold_to_consider_ = 0.1; // 0.5
    //max_angle_to_furthest_ = 0.785398; // 0.785398

    // TODO: set mode
    int mode = 0; // 0
    mode = cfg.path_angle_critic.mode;

    mode_ = static_cast<PathAngleMode>(mode);
    if (!reversing_allowed_ && mode_ == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
        mode_ = PathAngleMode::FORWARD_PREFERENCE;
        ROS_WARN(
            "Path angle mode set to no directional preference, but controller's settings "
            "don't allow for reversing! Setting mode to forward preference.");
    }

    ROS_INFO("path angle critic plugin initialized");
    ROS_INFO(
        "PathAngleCritic instantiated with %d power and %f weight. Mode set to: %s",
        power_, weight_, modeToStr(mode_).c_str());
}

void PathAngleCritic::reconfigure(const MPPIConfig *cfg)
{
    float vx_min = -0.1;
    vx_min = cfg->controller.vx_min;

    if (fabs(vx_min) < 1e-6) {  // zero
        reversing_allowed_ = false;
    } else if (vx_min < 0.0) {   // reversing possible
        reversing_allowed_ = true;
    }

    enabled_ = cfg->path_angle_critic.enabled;
    power_ = cfg->path_angle_critic.cost_power;
    weight_ = cfg->path_angle_critic.cost_weight;
    offset_from_furthest_ = cfg->path_angle_critic.offset_from_furthest;
    threshold_to_consider_ = cfg->path_angle_critic.threshold_to_consider;
    max_angle_to_furthest_ = cfg->path_angle_critic.max_angle_to_furthest;

    int mode = 0; // 0
    mode = cfg->path_angle_critic.mode;

    mode_ = static_cast<PathAngleMode>(mode);
    if (!reversing_allowed_ && mode_ == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
        mode_ = PathAngleMode::FORWARD_PREFERENCE;
        ROS_WARN(
            "Path angle mode set to no directional preference, but controller's settings "
            "don't allow for reversing! Setting mode to forward preference.");
    }

}

void PathAngleCritic::score(CriticData & data)
{
    reconfigure(cfg_);

    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("Path angle critic is disabled");
        }
        return;
    }

    utils::setPathFurthestPointIfNotSet(data);

    /**
    auto offseted_idx = std::min(
        *data.furthest_reached_path_point + offset_from_furthest_, data.path.x.shape(0) - 1);

    const float goal_x = data.path.x(offseted_idx);
    const float goal_y = data.path.y(offseted_idx);
    const float goal_yaw = data.path.yaws(offseted_idx);
    const geometry_msgs::Pose & pose = data.state.pose.pose;**/

    /*******************************************************************************************************/
    auto arr_offseted_idx = std::min(
        float(*data.furthest_reached_path_point + offset_from_furthest_), float(data.path.arr_x.rows() - 1));

    const float arr_goal_x = data.path.arr_x(arr_offseted_idx);
    const float arr_goal_y = data.path.arr_y(arr_offseted_idx);
    const float arr_goal_yaw = data.path.arr_yaws(arr_offseted_idx);
    const geometry_msgs::Pose & arr_pose = data.state.pose.pose;
    /*******************************************************************************************************/

    /**
    switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
        if (utils::posePointAngle(pose, goal_x, goal_y, true) < max_angle_to_furthest_) {
            //std::cout << "path angle critic invalid 2" << std::endl;
            return;
        }
        break;
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
        if (utils::posePointAngle(pose, goal_x, goal_y, false) < max_angle_to_furthest_) {
            return;
        }
        break;
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
        if (utils::posePointAngle(pose, goal_x, goal_y, goal_yaw) < max_angle_to_furthest_) {
            return;
        }
        break;
    default:
        ROS_ERROR("Invalid path angle mode!");
        break;
    }**/



    /*****************************************************************************************************/
    switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
        if (utils::posePointAngle(arr_pose, arr_goal_x, arr_goal_y, true) < max_angle_to_furthest_) {
            //std::cout << "path angle critic invalid 2" << std::endl;
            return;
        }
        break;
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
        if (utils::posePointAngle(arr_pose, arr_goal_x, arr_goal_y, false) < max_angle_to_furthest_) {
            return;
        }
        break;
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
        if (utils::posePointAngle(arr_pose, arr_goal_x, arr_goal_y, arr_goal_yaw) < max_angle_to_furthest_) {
            return;
        }
        break;
    default:
        ROS_ERROR("Invalid path angle mode!");
        break;
    }
    /********************************************************************************************************/




    /**
    auto yaws_between_points = xt::atan2(
        goal_y - xt::view(data.trajectories.y, xt::all(), -1),
        goal_x - xt::view(data.trajectories.x, xt::all(), -1));

    auto yaws =
        xt::fabs(utils::shortest_angular_distance(xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points));**/




    /**************************************************************************/
    Eigen::ArrayXf arr_yaws_between_points(data.trajectories.arr_yaws.rows());
    unsigned int last_col_index = data.trajectories.arr_yaws.cols() - 1;
    for (unsigned int i = 0; i < data.trajectories.arr_yaws.rows(); ++i)
    {
        arr_yaws_between_points(i) = std::atan2(arr_goal_y - data.trajectories.arr_y(i, last_col_index),
                                                arr_goal_x - data.trajectories.arr_x(i, last_col_index));
    }

    Eigen::ArrayXf arr_yaws =
        utils::arr_shortest_angular_distance(
            data.trajectories.arr_yaws.col(last_col_index),
            arr_yaws_between_points);
    /***************************************************************************/




    /**
    switch (mode_)
    {
    case PathAngleMode::FORWARD_PREFERENCE:
    {
        //data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
        if (power_ > 1u) {
            data.costs += xt::pow(std::move(yaws) * weight_, power_);
        } else {
            data.costs += std::move(yaws) * weight_;
        }

        return;
    }
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
    {
        //        const auto yaws_between_points_corrected = xt::where(
        //            yaws < M_PI_2, yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
        //        const auto corrected_yaws = xt::abs(
        //            utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
        //        data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);

        const auto yaws_between_points_corrected = xt::where(
            yaws < M_PIF_2, yaws_between_points,
            utils::normalize_angles(yaws_between_points + M_PIF));
        const auto corrected_yaws = xt::fabs(
            utils::shortest_angular_distance(
                xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points_corrected));
        if (power_ > 1u) {
            data.costs += xt::pow(std::move(corrected_yaws) * weight_, power_);
        } else {
            data.costs += std::move(corrected_yaws) * weight_;
        }

        return;
    }
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
    {
        //        const auto yaws_between_points_corrected = xt::where(
        //            xt::abs(utils::shortest_angular_distance(yaws_between_points, goal_yaw)) < M_PI_2,
        //            yaws_between_points, utils::normalize_angles(yaws_between_points + M_PI));
        //        const auto corrected_yaws = xt::abs(
        //            utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points_corrected));
        //        data.costs += xt::pow(xt::mean(corrected_yaws, {1}, immediate) * weight_, power_);

        const auto yaws_between_points_corrected = xt::where(
            xt::fabs(utils::shortest_angular_distance(yaws_between_points, goal_yaw)) < M_PIF_2,
            yaws_between_points, utils::normalize_angles(yaws_between_points + M_PIF));
        const auto corrected_yaws = xt::fabs(
            utils::shortest_angular_distance(
                xt::view(data.trajectories.yaws, xt::all(), -1), yaws_between_points_corrected));
        if (power_ > 1u) {
            data.costs += xt::pow(std::move(corrected_yaws) * weight_, power_);
        } else {
            data.costs += std::move(corrected_yaws) * weight_;
        }

        return;
    }
    }**/




    /******************************************************************************************************/
    switch (mode_) {
    case PathAngleMode::FORWARD_PREFERENCE:
    {
        //data.costs += xt::pow(xt::mean(yaws, {1}, immediate) * weight_, power_);
        if (power_ > 1u) {
            data.arr_costs_ += (std::move(arr_yaws.square()) * weight_).pow(power_);
        } else {
            data.arr_costs_ += std::move(arr_yaws.square()) * weight_;
        }
        return;
    }
    case PathAngleMode::NO_DIRECTIONAL_PREFERENCE:
    {
        Eigen::Array<bool, Eigen::Dynamic, 1> condition(arr_yaws.rows());
        condition = arr_yaws < M_PIF_2;
        const auto arr_yaws_between_points_corrected =
            condition.select(arr_yaws_between_points, utils::arr_normalize_angles(arr_yaws_between_points + M_PIF));
        const auto arr_corrected_yaws = (utils::arr_shortest_angular_distance(
                                             data.trajectories.arr_yaws.col(data.trajectories.arr_yaws.cols() - 1),
                                             arr_yaws_between_points_corrected)).abs();
        if (power_ > 1u) {
            data.arr_costs_ += (std::move(arr_corrected_yaws) * weight_).pow(power_);
        } else {
            data.arr_costs_ += std::move(arr_corrected_yaws) * weight_;
        }

        return;
    }
    case PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS:
    {
        Eigen::Array<bool, Eigen::Dynamic, 1> condition(arr_yaws.rows());
        condition = (utils::arr_shortest_angular_distance(arr_yaws_between_points, arr_goal_yaw)).abs() < M_PI_2;
        const auto arr_yaws_between_points_corrected = condition.select(
            arr_yaws_between_points, utils::arr_normalize_angles(arr_yaws_between_points + M_PIF));
        const auto arr_corrected_yaws = (utils::arr_shortest_angular_distance(
                                             data.trajectories.arr_yaws.col(data.trajectories.arr_yaws.cols() - 1),
                                             arr_yaws_between_points_corrected)).abs();
        if (power_ > 1u) {
            data.arr_costs_ += (std::move(arr_corrected_yaws) * weight_).pow(power_);
        } else {
            data.arr_costs_ += std::move(arr_corrected_yaws) * weight_;
        }

        return;
    }
    }
    /*************************************************************************************************************/
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PathAngleCritic,
    mppi::critics::CriticFunction)
