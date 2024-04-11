#include <mppi_local_planner/critics/prefer_forward_critic.hpp>

namespace mppi::critics
{

void PreferForwardCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.prefer_forward_critic.enabled;
    power_ = cfg.prefer_forward_critic.cost_power;
    weight_ = cfg.prefer_forward_critic.cost_weight;
    threshold_to_consider_ = cfg.prefer_forward_critic.threhold_to_consider;

    // TODO: get param
    //power_ = 1;
    //weight_ = 20.0; // 5.0 // 20.0
    //threshold_to_consider_ = 0.3;

    ROS_INFO("prefer forward critic plugin initialized");
    ROS_INFO(
         "PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->prefer_forward_critic.enabled;
    power_ = cfg->prefer_forward_critic.cost_power;
    weight_ = cfg->prefer_forward_critic.cost_weight;
    threshold_to_consider_ = cfg->prefer_forward_critic.threhold_to_consider;
}

void PreferForwardCritic::score(CriticData & data)
{

    reconfigure(cfg_);

    /**
    using xt::evaluation_strategy::immediate;**/

    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("Prefer forward critic is disabled");
        }
        return;
    }

    /**
    auto backward_motion = xt::maximum(-data.state.vx, 0);
    data.costs += xt::pow(
        xt::sum(
            std::move(
                backward_motion) * data.model_dt, {1}, immediate) * weight_, power_);**/


    /*******************************************************************************************/
    auto arr_back_motion = 0.0f - data.state.arr_vx.min(0.0);
    data.arr_costs_ += ((arr_back_motion * data.model_dt).rowwise().sum() * weight_).pow(power_);
    /*******************************************************************************************/
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::PreferForwardCritic,
    mppi::critics::CriticFunction)
