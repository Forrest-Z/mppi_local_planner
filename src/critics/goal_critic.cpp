#include <mppi_local_planner/critics/goal_critic.hpp>

namespace mppi::critics
{
/**
using xt::evaluation_strategy::immediate;**/

void GoalCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.goal_critic.enabled;
    power_ = cfg.goal_critic.cost_power;
    weight_ = cfg.goal_critic.cost_weight;
    threshold_to_consider_ = cfg.goal_critic.threshold_to_consider;
    // TODO: get param from ros node handle
    //power_ = 1;
    //weight_ = 10.0; // 5.0
    //threshold_to_consider_ = 1.0; // 1.0

    ROS_INFO("goal critic critic plugin initialized");
    ROS_INFO(
         "GoalCritic instantiated with %d power and %f weight.",
        power_, weight_);
}

void GoalCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->goal_critic.enabled;
    power_ = cfg->goal_critic.cost_power;
    weight_ = cfg->goal_critic.cost_weight;
    threshold_to_consider_ = cfg->goal_critic.threshold_to_consider;
}

void GoalCritic::score(CriticData & data)
{
    reconfigure(cfg_);

    if (!enabled_ || !utils::withinPositionGoalTolerance(
                         threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("goal critic is disabled");
        }
        return;
    }

    /**
    const auto goal_idx = data.path.x.shape(0) - 1;

    const auto goal_x = data.path.x(goal_idx);
    const auto goal_y = data.path.y(goal_idx);

    const auto traj_x = xt::view(data.trajectories.x, xt::all(), xt::all());
    const auto traj_y = xt::view(data.trajectories.y, xt::all(), xt::all());


    if (power_ > 1u)
    {
        data.costs += xt::pow(
            xt::mean(
                xt::hypot(traj_x - goal_x, traj_y - goal_y),
                {1}, immediate)*weight_, power_);
    }
    else
    {
        data.costs += xt::mean(
            xt::hypot(traj_x - goal_x, traj_y - goal_y),
                          {1}, immediate)*weight_;
    }**/


    /**********************************************************************/
    const auto arr_goal_idx = data.path.arr_x.rows() - 1;
    const auto arr_goal_x = data.path.arr_x(arr_goal_idx);
    const auto arr_goal_y = data.path.arr_y(arr_goal_idx);
    const auto arr_traj_x = data.trajectories.arr_x;
    const auto arr_traj_y = data.trajectories.arr_y;

    if (power_ > 1u)
    {
        data.arr_costs_ += (((arr_traj_x - arr_goal_x).square() +
                             (arr_traj_y - arr_goal_y).square())
                                .sqrt().rowwise().mean() * weight_).pow(power_);
    }
    else
    {
        data.arr_costs_ += ((arr_traj_x - arr_goal_x).square() +
                            (arr_traj_y - arr_goal_y).square())
                               .sqrt().rowwise().mean() * weight_;
    }
    /**********************************************************************/

}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
