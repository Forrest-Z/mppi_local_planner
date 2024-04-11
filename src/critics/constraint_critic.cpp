#include <mppi_local_planner/critics/constraint_critic.hpp>

namespace mppi::critics
{

void ConstraintCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.constraint_critic.enabled;
    power_ = cfg.constraint_critic.cost_power;
    weight_ = cfg.constraint_critic.cost_weight;
    //power_ = 1;
    //weight_ = 4.0; // 4.0

    float vx_max, vy_max, vx_min;
    vx_max = cfg.controller.vx_max;
    vy_max = cfg.controller.vy_max;
    vx_min = cfg.controller.vx_min;

    float wz_max = cfg.controller.wz_max;

    float vx_acc_max, wz_acc_max;
    vx_acc_max = cfg.constraint_critic.vx_acc_max;
    wz_acc_max = cfg.constraint_critic.wz_acc_max;

    const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
    max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
    min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

    wz_max_ = wz_max;
    vx_acc_max_ = vx_acc_max;
    wz_acc_max_ = wz_acc_max;

    ROS_INFO("constraint critic plugin initialized");
}

void ConstraintCritic::reconfigure(const MPPIConfig* cfg)
{
    enabled_ = cfg->constraint_critic.enabled;
    power_ = cfg->constraint_critic.cost_power;
    weight_ = cfg->constraint_critic.cost_weight;

    float vx_max, vy_max, vx_min;
    vx_max = cfg->controller.vx_max;
    vy_max = cfg->controller.vy_max;
    vx_min = cfg->controller.vx_min;

    float wz_max = cfg->controller.wz_max;

    float vx_acc_max, wz_acc_max;
    vx_acc_max = cfg->constraint_critic.vx_acc_max;
    wz_acc_max = cfg->constraint_critic.wz_acc_max;

    const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
    max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
    min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

    wz_max_ = wz_max;
    vx_acc_max_ = vx_acc_max;
    wz_acc_max_ = wz_acc_max;
}

void ConstraintCritic::score(CriticData & data)
{
    reconfigure(cfg_);
    /**
    using xt::evaluation_strategy::immediate;**/

    if (!enabled_) {
        ROS_WARN("Constraint critic is disabled");
        return;
    }

    /**
    // gx, suppose the robot is a differential model
    auto out_of_max_bounds_wz = xt::maximum(data.state.wz - wz_max_, 0.0f);
    auto out_of_min_bounds_wz = xt::maximum(-wz_max_ - data.state.wz, 0.0f);**/

    /***********************************************************************************/
    Eigen::ArrayXXf arr_out_of_max_bounds_vx = (data.state.arr_vx - max_vel_).max(0.0f);
    Eigen::ArrayXXf arr_out_of_min_bounds_vx = (min_vel_ - data.state.arr_vx).max(0.0f);
    Eigen::ArrayXXf arr_out_of_max_bounds_wz = (data.state.arr_wz - wz_max_).max(0.0f);
    Eigen::ArrayXXf arr_out_of_min_bounds_wz = (-wz_max_ - data.state.arr_wz).max(0.0f);
    /***********************************************************************************/


    // Differential motion model
    auto diff = dynamic_cast<DiffDriveMotionModel *>(data.motion_model.get());
    if (diff != nullptr) {
        if (power_ > 1u) {
            /**
            data.costs += xt::pow(
                xt::sum(
                    (std::move(
                        xt::maximum(data.state.vx - max_vel_, 0.0f) +
                        xt::maximum(min_vel_ - data.state.vx, 0.0f)) +
                     std::move(out_of_max_bounds_wz) +
                     std::move(out_of_min_bounds_wz)) *
                        data.model_dt, {1}, immediate) * weight_, power_);**/


            /************************************************************************************/
            data.arr_costs_ += ((arr_out_of_max_bounds_vx +
                               arr_out_of_min_bounds_vx +
                               arr_out_of_max_bounds_wz +
                               arr_out_of_min_bounds_wz).rowwise().sum() * weight_).pow(power_);
            /************************************************************************************/

        } else {
            /**
            data.costs += xt::sum(
                              (std::move(
                                  xt::maximum(data.state.vx - max_vel_, 0.0f) +
                                  xt::maximum(min_vel_ - data.state.vx, 0.0f)) +
                               std::move(out_of_max_bounds_wz) +
                               std::move(out_of_min_bounds_wz)) *
                                  data.model_dt, {1}, immediate) * weight_;**/


            /************************************************************************************/
            data.arr_costs_ += (arr_out_of_max_bounds_vx +
                                 arr_out_of_min_bounds_vx +
                                 arr_out_of_max_bounds_wz +
                                 arr_out_of_min_bounds_wz).rowwise().sum() * weight_;
            /************************************************************************************/
        }

        return;
    }

    /**
    // Omnidirectional motion model
    auto omni = dynamic_cast<OmniMotionModel *>(data.motion_model.get());
    if (omni != nullptr) {
        auto sgn = xt::eval(xt::where(data.state.vx > 0.0f, 1.0f, -1.0f));
        auto vel_total = sgn * xt::hypot(data.state.vx, data.state.vy);
        if (power_ > 1u) {
            data.costs += xt::pow(
                xt::sum(
                    (std::move(
                        xt::maximum(vel_total - max_vel_, 0.0f) +
                        xt::maximum(min_vel_ - vel_total, 0.0f))) *
                        data.model_dt, {1}, immediate) * weight_, power_);
        } else {
            data.costs += xt::sum(
                              (std::move(
                                  xt::maximum(vel_total - max_vel_, 0.0f) +
                                  xt::maximum(min_vel_ - vel_total, 0.0f))) *
                                  data.model_dt, {1}, immediate) * weight_;
        }
        return;
    }

    // Ackermann motion model
    auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
    if (acker != nullptr) {
        auto & vx = data.state.vx;
        auto & wz = data.state.wz;
        auto out_of_turning_rad_motion = xt::maximum(
            acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0f);
        if (power_ > 1u) {
            data.costs += xt::pow(
                xt::sum(
                    (std::move(
                        xt::maximum(data.state.vx - max_vel_, 0.0f) +
                        xt::maximum(min_vel_ - data.state.vx, 0.0f) + out_of_turning_rad_motion)) *
                        data.model_dt, {1}, immediate) * weight_, power_);
        } else {
            data.costs += xt::sum(
                              (std::move(
                                  xt::maximum(data.state.vx - max_vel_, 0.0f) +
                                  xt::maximum(min_vel_ - data.state.vx, 0.0f) + out_of_turning_rad_motion)) *
                                  data.model_dt, {1}, immediate) * weight_;
        }
        return;
    }**/

    return;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
