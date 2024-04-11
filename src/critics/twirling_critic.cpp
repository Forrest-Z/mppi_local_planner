#include <mppi_local_planner/critics/twirling_critic.hpp>

namespace mppi::critics
{

void TwirlingCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.twirling_critic.enabled;
    power_ = cfg.twirling_critic.twirling_cost_power;
    weight_ = cfg.twirling_critic.twirling_cost_weight;

    // TODO: get param
    //power_ = 1;
    //weight_ = 5.0; //

    ROS_INFO("twirling critic plugin initialized");
    ROS_INFO(
         "TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

void TwirlingCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->twirling_critic.enabled;
    power_ = cfg->twirling_critic.twirling_cost_power;
    weight_ = cfg->twirling_critic.twirling_cost_weight;
}

void TwirlingCritic::score(CriticData & data)
{
    reconfigure(cfg_);

    /**
    using xt::evaluation_strategy::immediate;**/

    if (!enabled_ ||
        utils::withinPositionGoalTolerance(data.pose_tolerance, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("Twirling critic is disabled");
        }
        return;
    }

    /**
    if (data.state.speed.linear.x < 0) // gx
    {
        const auto wz = xt::abs(data.state.wz);
        data.costs += xt::pow(xt::mean(wz, {1}, immediate) * weight_, power_);
    }**/

    /*********************************************************************************/
    if (data.state.speed.linear.x < 0) // gx
    {
        const auto arr_wz = data.state.arr_wz.abs();
        data.arr_costs_ += (arr_wz.rowwise().mean() * weight_).pow(power_);
    }
    /*********************************************************************************/
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::TwirlingCritic,
    mppi::critics::CriticFunction)
