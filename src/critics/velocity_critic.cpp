#include <mppi_local_planner/critics/velocity_critic.hpp>

namespace mppi::critics
{

void VelocityCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.velocity_critic.enabled;
    power_ = cfg.velocity_critic.cost_power;
    weight_ = cfg.velocity_critic.cost_weight;
    threshold_to_consider_ = cfg.velocity_critic.threshold_to_consider;
    max_vel_x_ = cfg.velocity_critic.max_vel_x;
    // TODO: get param
    //power_ = 1;
    //weight_ = 0.1; // 5.0
    //threshold_to_consider_ = 0.1; // 0.5
    //max_vel_x_ = 0.6;

    ROS_INFO("velocity critic plugin initialized");
    ROS_INFO(
        "VelocityCritic instantiated with %d power and %f weight.", power_, weight_);
}

void VelocityCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->velocity_critic.enabled;
    power_ = cfg->velocity_critic.cost_power;
    weight_ = cfg->velocity_critic.cost_weight;
    threshold_to_consider_ = cfg->velocity_critic.threshold_to_consider;
    max_vel_x_ = cfg->velocity_critic.max_vel_x;
}

void VelocityCritic::score(CriticData & data)
{

    reconfigure(cfg_);

    /**
    using xt::evaluation_strategy::immediate;**/


    if (!enabled_ ||
        utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.path))
    {
        if (!enabled_)
        {
            ROS_WARN("Velocity critic is disabled");
        }
        return;
    }

    /**
    if (data.state.speed.linear.x > 0 && data.state.vx.shape(1) > 15 && data.current_furthest_ > 8)
    {
        for (size_t i = 0; i < data.state.vx.shape(0); ++i)
        {
            for (size_t j = 0; j < 15; ++j)
            {
                const auto error_vx = fabs(max_vel_x_ - data.state.vx(i, j));
                data.costs(i) += powf(error_vx * weight_, power_);
            }
        }
    }**/

    /**************************************************************************************************/
    if (data.state.speed.linear.x > 0 && data.state.arr_vx.cols() > 15 && data.current_furthest_ > 8)
    {
        for (size_t i = 0; i < data.state.arr_vx.rows(); ++i)
        {
            for (size_t j = 0; j < 15; ++j)
            {
                const auto error_vx = fabs(max_vel_x_ - data.state.arr_vx(i, j));
                data.arr_costs_(i) += powf(error_vx * weight_, power_);
            }
        }
    }
    /**************************************************************************************************/
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::VelocityCritic,
    mppi::critics::CriticFunction)
