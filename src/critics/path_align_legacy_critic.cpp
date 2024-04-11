#include <mppi_local_planner/critics/path_align_legacy_critic.hpp>

namespace mppi::critics
{
/**
using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;**/

void PathAlignLegacyCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.path_align_critic.enabled;
    power_ = cfg.path_align_critic.cost_power;
    weight_ = cfg.path_align_critic.cost_weight;
    max_path_occupancy_ratio_ = cfg.path_align_critic.max_path_occupancy_ratio;
    offset_from_furthest_ = cfg.path_align_critic.offset_from_furthest;
    trajectory_point_step_ = cfg.path_align_critic.trajectory_point_step;
    threshold_to_consider_ = cfg.path_align_critic.threshold_to_consider;
    use_path_orientations_ = cfg.path_align_critic.use_path_orientations;
    orientation_weight_ = cfg.path_align_critic.orientation_weight;
    // TODO: get param
    //power_ = 1; // 1
    //weight_ = 30.0; // 10 30.0
    //max_path_occupancy_ratio_ = 0.3; // 0.13
    //offset_from_furthest_ = 0; // 2
    //trajectory_point_step_ = 4; // 4
    //threshold_to_consider_ = 0.1; // 0.5
    //use_path_orientations_ = true; // false
    //orientation_weight_ = 0.05;

    ROS_INFO("path align legacy critic plugin initialized");
    ROS_INFO(
        "ReferenceTrajectoryCritic instantiated with %d power and %f weight",
        power_, weight_);
}

void PathAlignLegacyCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->path_align_critic.enabled;
    power_ = cfg->path_align_critic.cost_power;
    weight_ = cfg->path_align_critic.cost_weight;
    max_path_occupancy_ratio_ = cfg->path_align_critic.max_path_occupancy_ratio;
    offset_from_furthest_ = cfg->path_align_critic.offset_from_furthest;
    trajectory_point_step_ = cfg->path_align_critic.trajectory_point_step;
    threshold_to_consider_ = cfg->path_align_critic.threshold_to_consider;
    use_path_orientations_ = cfg->path_align_critic.use_path_orientations;
    orientation_weight_ = cfg->path_align_critic.orientation_weight;
}

void PathAlignLegacyCritic::score(CriticData & data)
{

    reconfigure(cfg_);

    // Don't apply close to goal, let the goal critics take over
    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("Path align legacy critic is disabled");
        }
        return;
    }

    // Don't apply when first getting bearing w.r.t. the path
    utils::setPathFurthestPointIfNotSet(data);
    if (*data.furthest_reached_path_point < offset_from_furthest_) {
        std::cout << "path align legacy critic invalid 2" << std::endl;
        return;
    }

    // Don't apply when dynamic obstacles are blocking significant proportions of the local path
    utils::setPathCostsIfNotSet(data, costmap_ros_);
    const size_t closest_initial_path_point = utils::findPathTrajectoryInitialPoint(data);
    unsigned int invalid_ctr = 0;
    const float range = *data.furthest_reached_path_point - closest_initial_path_point;
    for (size_t i = closest_initial_path_point; i < *data.furthest_reached_path_point; i++) {
        if (!(*data.path_pts_valid)[i]) {invalid_ctr++;}
        if (static_cast<float>(invalid_ctr) / range > max_path_occupancy_ratio_ && invalid_ctr > 2) {
            std::cout << "path align legacy critic invalid 3" << std::endl;
            return;
        }
    }

    /**
    const auto & T_x = data.trajectories.x;
    const auto & T_y = data.trajectories.y;
    const auto & T_yaw = data.trajectories.yaws;

    const auto P_x = xt::view(data.path.x, xt::range(_, -1));  // path points
    const auto P_y = xt::view(data.path.y, xt::range(_, -1));  // path points
    const auto P_yaw = xt::view(data.path.yaws, xt::range(_, -1));  // path points

    const size_t batch_size = T_x.shape(0);
    const size_t time_steps = T_x.shape(1);
    const size_t traj_pts_eval = floor(time_steps / trajectory_point_step_);
    const size_t path_segments_count = data.path.x.shape(0) - 1;
    auto && cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});

    if (path_segments_count < 1) {
        std::cout << "path align legacy critic invalid 4" << std::endl;
        return;
    }

    float dist_sq = 0.0f, dx = 0.0f, dy = 0.0f, dyaw = 0.0f, summed_dist = 0.0f;
    float min_dist_sq = std::numeric_limits<float>::max();
    size_t min_s = 0;

    // search nearest point on global path for every trajectory
    for (size_t t = 0; t < batch_size; ++t) {
        summed_dist = 0.0f;
        for (size_t p = trajectory_point_step_; p < time_steps; p += trajectory_point_step_) {
            min_dist_sq = std::numeric_limits<float>::max();
            min_s = 0;

            // Find closest path segment to the trajectory point
            for (size_t s = 0; s < path_segments_count - 1; s++) {
                xt::xtensor_fixed<float, xt::xshape<2>> P;
                dx = P_x(s) - T_x(t, p);
                dy = P_y(s) - T_y(t, p);
                if (use_path_orientations_) {
                    dyaw = orientation_weight_ * angles::shortest_angular_distance(P_yaw(s), T_yaw(t, p));
                    dist_sq = dx * dx + dy * dy + dyaw * dyaw;
                } else {
                    dist_sq = dx * dx + dy * dy;
                }
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    min_s = s;
                }
            }

            // The nearest path point to align to needs to be not in collision, else
            // let the obstacle critic take over in this region due to dynamic obstacles
            if (min_s != 0 && (*data.path_pts_valid)[min_s]) {
                //summed_dist += sqrtf(min_dist_sq);
                // gx
                summed_dist += sqrtf(min_dist_sq) * (1.0 + float(path_segments_count - min_s) / float(path_segments_count));
            }
        }

        cost[t] = summed_dist / traj_pts_eval;
    }

    data.costs += xt::pow(std::move(cost) * weight_, power_);**/



    /*************************************************************************************************/
    const auto & arr_T_x = data.trajectories.arr_x;
    const auto & arr_T_y = data.trajectories.arr_y;
    const auto & arr_T_yaw = data.trajectories.arr_yaws;

    const auto arr_P_x = data.path.arr_x;  // path points
    const auto arr_P_y = data.path.arr_y;  // path points
    const auto arr_P_yaw = data.path.arr_yaws;  // path points

    const size_t arr_batch_size = arr_T_x.rows();
    const size_t arr_time_steps = arr_T_x.cols();
    const size_t arr_traj_pts_eval = floor(arr_time_steps / trajectory_point_step_);
    const size_t arr_path_segments_count = data.path.arr_x.rows() - 1;
    Eigen::ArrayXf arr_cost(arr_batch_size);
    arr_cost.setZero();

    if (arr_path_segments_count < 1) {
        std::cout << "path align legacy critic invalid 4" << std::endl;
        return;
    }

    float arr_dist_sq = 0.0f, arr_dx = 0.0f, arr_dy = 0.0f, arr_dyaw = 0.0f, arr_summed_dist = 0.0f;
    float arr_min_dist_sq = std::numeric_limits<float>::max();
    size_t arr_min_s = 0;

    // search nearest point on global path for every trajectory
    for (size_t t = 0; t < arr_batch_size; ++t) {
        arr_summed_dist = 0.0f;
        for (size_t p = trajectory_point_step_; p < arr_time_steps; p += trajectory_point_step_) {
            arr_min_dist_sq = std::numeric_limits<float>::max();
            arr_min_s = 0;

            // Find closest path segment to the trajectory point
            for (size_t s = 0; s < arr_path_segments_count - 1; s++) {
                arr_dx = arr_P_x(s) - arr_T_x(t, p);
                arr_dy = arr_P_y(s) - arr_T_y(t, p);
                if (use_path_orientations_) {
                    arr_dyaw = orientation_weight_ * angles::shortest_angular_distance(arr_P_yaw(s), arr_T_yaw(t, p));
                    arr_dist_sq = arr_dx * arr_dx + arr_dy * arr_dy + arr_dyaw * arr_dyaw;
                } else {
                    arr_dist_sq = arr_dx * arr_dx + arr_dy * arr_dy;
                }
                if (arr_dist_sq < arr_min_dist_sq) {
                    arr_min_dist_sq = arr_dist_sq;
                    arr_min_s = s;
                }
            }

            // The nearest path point to align to needs to be not in collision, else
            // let the obstacle critic take over in this region due to dynamic obstacles
            if (arr_min_s != 0 && (*data.path_pts_valid)[arr_min_s]) {
                //summed_dist += sqrtf(min_dist_sq);
                // gx
                arr_summed_dist += sqrtf(arr_min_dist_sq) * (1.0 + float(arr_path_segments_count - arr_min_s) / float(arr_path_segments_count));
            }
        }

        arr_cost[t] = arr_summed_dist / arr_traj_pts_eval;
    }

    data.arr_costs_ += (std::move(arr_cost) * weight_).pow(power_);
    /*********************************************************************************************************/
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PathAlignLegacyCritic,
    mppi::critics::CriticFunction)
