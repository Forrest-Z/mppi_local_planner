#include <mppi_local_planner/critics/path_follow_critic.hpp>

namespace mppi::critics
{

void PathFollowCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_  = cfg.path_follow_critic.enabled;
    power_ = cfg.path_follow_critic.cost_power;
    weight_ = cfg.path_follow_critic.cost_weight;
    threshold_to_consider_ = cfg.path_follow_critic.threhold_to_consider;
    offset_from_furthest_ = cfg.path_follow_critic.offset_from_furthest;
    lookahead_ = cfg.controller.look_ahead;
    // TODO: get param
    //threshold_to_consider_ = 0.1; // 0.5
    //offset_from_furthest_ = 6; // 6
    //power_ = 1;
    //weight_ = 5.0; // 5.0
    //lookahead_ = 10; // 10

    ROS_INFO("path follow critic plugin initialized");
}

void PathFollowCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_  = cfg->path_follow_critic.enabled;
    power_ = cfg->path_follow_critic.cost_power;
    weight_ = cfg->path_follow_critic.cost_weight;
    threshold_to_consider_ = cfg->path_follow_critic.threhold_to_consider;
    offset_from_furthest_ = cfg->path_follow_critic.offset_from_furthest;
    lookahead_ = cfg->controller.look_ahead;
}

void PathFollowCritic::score(CriticData & data)
{
    reconfigure(cfg_);

    /**
    if (!enabled_ || data.path.x.shape(0) < 2 ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        std::cout << "path follow critic invalid 1" << std::endl;
        return;
    }**/


    /**********************************************************************************************/
    if (!enabled_ || data.path.arr_x.rows() < 2 ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("path follow critic is disabled");
        }
        return;
    }
    /**********************************************************************************************/


    utils::setPathFurthestPointIfNotSet(data);
    utils::setPathCostsIfNotSet(data, costmap_ros_);


    /**
    const size_t path_size = data.path.x.shape(0) - 1;

    auto offseted_idx = std::min(
        *data.furthest_reached_path_point + offset_from_furthest_, path_size);

    // gx
    offseted_idx = std::min(data.current_furthest_, offseted_idx);


    // Drive to the first valid path point, in case of dynamic obstacles on path
    // we want to drive past it, not through it
    bool valid = false;
    while (!valid && offseted_idx < path_size - 1) {
        valid = (*data.path_pts_valid)[offseted_idx];
        if (!valid) {
            offseted_idx++;
        }
    }

    // Prevent the anchor from being too far
    offseted_idx = std::min(offseted_idx, lookahead_);

    //std::cout << "offseted idx: " << offseted_idx << std::endl;

    const auto path_x = data.path.x(offseted_idx);
    const auto path_y = data.path.y(offseted_idx);

    // last point on all trajectories
    const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
    const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);

    auto dists = xt::sqrt(
        xt::pow(last_x - path_x, 2) +
        xt::pow(last_y - path_y, 2));

    data.costs += xt::pow(weight_ * std::move(dists), power_);**/




    /******************************************************************************/
    const size_t arr_path_size = data.path.arr_x.rows() - 1;

    auto arr_offseted_idx = std::min(
        *data.furthest_reached_path_point + offset_from_furthest_, arr_path_size);

    // gx
    arr_offseted_idx = std::min(data.current_furthest_, arr_offseted_idx);


    // Drive to the first valid path point, in case of dynamic obstacles on path
    // we want to drive past it, not through it
    bool arr_valid = false;
    while (!arr_valid && arr_offseted_idx < arr_path_size - 1) {
        arr_valid = (*data.path_pts_valid)[arr_offseted_idx];
        if (!arr_valid) {
            arr_offseted_idx++;
        }
    }

    // Prevent the anchor from being too far
    arr_offseted_idx = std::min(arr_offseted_idx, lookahead_);

    const float arr_path_x = data.path.arr_x(arr_offseted_idx);
    const float arr_path_y = data.path.arr_y(arr_offseted_idx);

    // last point on all trajectories
    unsigned int last_col_index = data.trajectories.arr_x.cols() - 1;
    const Eigen::ArrayXf arr_last_x = data.trajectories.arr_x.col(last_col_index);
    const Eigen::ArrayXf arr_last_y = data.trajectories.arr_y.col(last_col_index);

    Eigen::ArrayXf arr_dists = ((arr_last_x - arr_path_x).square() +
                      (arr_last_y - arr_path_y).square()).sqrt();

    //std::cout << arr_dists.rows() << " " << arr_dists.cols() << std::endl;

    data.arr_costs_ += (weight_ * std::move(arr_dists)).pow(power_);
    /******************************************************************************/


}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PathFollowCritic,
    mppi::critics::CriticFunction)
