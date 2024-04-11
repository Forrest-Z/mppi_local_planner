#include <mppi_local_planner/critics/goal_angle_critic.hpp>

namespace mppi::critics
{

void GoalAngleCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.goal_angle_critic.enabled;
    power_ = cfg.goal_angle_critic.cost_power;
    weight_ = cfg.goal_angle_critic.cost_weight;
    threshold_to_consider_ = cfg.goal_angle_critic.threhold_to_consider;
    // TODO: get param from ros node handle
    //power_ = 1;
    //weight_ = 5.0; // 3.0
    //threshold_to_consider_ = 1.0; // 1.0

    ROS_INFO("goal angle critic plugin initialized");
    ROS_INFO(
        "GoalAngleCritic instantiated with %d power, %f weight, and %f "
        "angular threshold.",
        power_, weight_, threshold_to_consider_);
}

void GoalAngleCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->goal_angle_critic.enabled;
    power_ = cfg->goal_angle_critic.cost_power;
    weight_ = cfg->goal_angle_critic.cost_weight;
    threshold_to_consider_ = cfg->goal_angle_critic.threhold_to_consider;
}

void GoalAngleCritic::score(CriticData & data)
{
    reconfigure(cfg_);

    if (!enabled_ || !utils::withinPositionGoalTolerance(
                         threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("Goal angle critic is disabled");
        }
        return;
    }

    /**
    const auto goal_idx = data.path.x.shape(0) - 1;
    const float goal_yaw = data.path.yaws(goal_idx);

    data.costs += xt::pow(
        xt::mean(xt::abs(utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)), {1}) *
            weight_, power_);**/

    /***********************************************************************************************/
    const auto arr_goal_idx = data.path.arr_x.rows() - 1;
    const float arr_goal_yaw = data.path.arr_yaws(arr_goal_idx);

    data.arr_costs_ += (weight_ *
                        (utils::arr_shortest_angular_distance(data.trajectories.arr_yaws, arr_goal_yaw))
                            .abs().rowwise().mean()).pow(power_);
    /***********************************************************************************************/
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::GoalAngleCritic,
    mppi::critics::CriticFunction)
