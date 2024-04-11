#include <mppi_local_planner/optimizer.hpp>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

namespace mppi
{

void Optimizer::initialize(
    ros::NodeHandle* nh, const std::string & name,
    costmap_2d::Costmap2DROS* costmap_ros, MPPIConfig& cfg)
{
    nh_ = nh;
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    retry_attemp_failed_ = false;

    getParams(cfg);

    critic_manager_.on_configure(nh_, name_, costmap_ros_, cfg);
    noise_generator_.initialize(settings_, isHolonomic(), name_, nh_);

    reset();
}

void Optimizer::shutdown()
{
    noise_generator_.shutdown();
}

void Optimizer::getParams(MPPIConfig& cfg)
{
    std::string motion_model_name;

    auto & s = settings_;

    s.model_dt = cfg.controller.model_dt;
    s.time_steps = cfg.controller.time_steps;
    s.batch_size = cfg.controller.batch_size;
    s.iteration_count = cfg.controller.iteration_count;
    s.temperature = cfg.controller.temperature;
    s.gamma = cfg.controller.gamma;
    s.base_constraints.vx_max = cfg.controller.vx_max;
    s.base_constraints.vx_min = cfg.controller.vx_min;
    s.base_constraints.vy = cfg.controller.vy_max;
    s.base_constraints.wz = cfg.controller.wz_max;
    s.sampling_std.vx = cfg.controller.vx_std;
    s.sampling_std.vy = cfg.controller.vy_std;
    s.sampling_std.wz = cfg.controller.wz_std;
    s.retry_attempt_limit = cfg.controller.retry_attempt_limit;

    s.constraints = s.base_constraints;

    s.look_ahead_ = cfg.controller.look_ahead;
    s.curvature_estimate_point_ = cfg.controller.curvature_estimate_point;

    s.noise_regenerate = cfg.noise_genetrator.regenerate;

    motion_model_name = "DiffDrive";

    // TODO: get param
//    s.model_dt = 0.1f; // 0.05f 0.1f
//    s.time_steps = 32; // 56
//    s.batch_size = 1800; // 1000
//    s.iteration_count = 1;
//    s.temperature = 0.3f; // 0.3f
//    s.gamma = 0.015f; // 0.015
//    s.base_constraints.vx_max = 0.6f; // 0.7f
//    s.base_constraints.vx_min = -0.11f;
//    s.base_constraints.vy = 0.0f; // vy_max
//    s.base_constraints.wz = 1.0f; // wz_max, 1.9
//    s.sampling_std.vx = 0.2f; // vx_std, 0.2
//    s.sampling_std.vy = 0.0f; // vy_std
//    s.sampling_std.wz = 0.4f; // wz_std, 0.4
//    s.retry_attempt_limit = 3; // 1

//    s.constraints = s.base_constraints;

//    // gx
//    s.look_ahead_ = 10;
//    s.curvature_estimate_point_ = 6;

//    motion_model_name = "DiffDrive";

    setMotionModel(motion_model_name);

    reset();

    double controller_frequency;
    // TODO: get controller frequency
    controller_frequency = 10;
    setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
    const double controller_period = 1.0 / controller_frequency;
    constexpr double eps = 1e-6;

    if ((controller_period + eps) < settings_.model_dt) {
        ROS_WARN(
            "Controller period is less then model dt, consider setting it equal");
    } else if (abs(controller_period - settings_.model_dt) < eps) {
        ROS_INFO(
            "Controller period is equal to model dt. Control sequence "
            "shifting is ON");
        settings_.shift_control_sequence = true;
    } else {
        ROS_ERROR(
            "Controller period more then model dt, set it equal to model dt");
    }
}

void Optimizer::reset()
{
    state_.reset(settings_.batch_size, settings_.time_steps);
    control_sequence_.reset(settings_.time_steps);
    control_history_[0] = {0.0f, 0.0f, 0.0f};
    control_history_[1] = {0.0f, 0.0f, 0.0f};
    control_history_[2] = {0.0f, 0.0f, 0.0f};
    control_history_[3] = {0.0f, 0.0f, 0.0f};

    settings_.constraints = settings_.base_constraints;

    /**
    costs_ = xt::zeros<float>({settings_.batch_size});**/

    arr_costs_ = Eigen::ArrayXf(settings_.batch_size);
    arr_costs_.setZero();

    generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

    noise_generator_.reset(settings_, isHolonomic());
    ROS_INFO("Optimizer reset");
}

bool Optimizer::isHolonomic() const
{
    return motion_model_->isHolonomic();
}

geometry_msgs::TwistStamped Optimizer::evalControl(
    const geometry_msgs::PoseStamped & robot_pose,
    const geometry_msgs::Twist & robot_speed,
    const nav_msgs::Path & plan)
{
    prepare(robot_pose, robot_speed, plan);

    setCurrentFurthest(critics_data_);

    do {
        optimize();
    } while (fallback(critics_data_.fail_flag));

    utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    auto control = getControlFromSequenceAsTwist(plan.header.stamp);

    if (settings_.shift_control_sequence) {
        shiftControlSequence();
    }

    return control;
}

void Optimizer::optimize()
{
    for (size_t i = 0; i < settings_.iteration_count; ++i) {
        generateNoisedTrajectories();
        critic_manager_.evalTrajectoriesScores(critics_data_);
        updateControlSequence();
    }
}

bool Optimizer::fallback(bool fail)
{
    static size_t counter = 0;

    if (!fail) {
        counter = 0;
        retry_attemp_failed_ = false;
        return false;
    }

    reset();

    if (++counter > settings_.retry_attempt_limit) {
        counter = 0;
        retry_attemp_failed_ = true;
        ROS_ERROR("-[MPPI]: Optimizer fail to compute path");
        //throw nav2_core::NoValidControl("Optimizer fail to compute path");
    }

    return true;
}

void Optimizer::prepare(
    const geometry_msgs::PoseStamped & robot_pose,
    const geometry_msgs::Twist & robot_speed,
    const nav_msgs::Path & plan)
{
    state_.pose = robot_pose;
    state_.speed = robot_speed;
    path_ = utils::toTensor(plan); // also to eigen array
    /**
    costs_.fill(0);**/

    arr_costs_.setZero();

    // TODO: adjust goal tolerance
    goal_tolerance_ = 0.05;

    critics_data_.fail_flag = false;
    // TODO: goal check prepare
    critics_data_.motion_model = motion_model_;
    critics_data_.furthest_reached_path_point.reset();
    critics_data_.path_pts_valid.reset();

    // gx
    critics_data_.current_furthest_ = settings_.look_ahead_;
}

void Optimizer::shiftControlSequence()
{
    /**
    using namespace xt::placeholders;  // NOLINT
    control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
    control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


    xt::view(control_sequence_.vx, -1) =
        xt::view(control_sequence_.vx, -2);

    xt::view(control_sequence_.wz, -1) =
        xt::view(control_sequence_.wz, -2);


    if (isHolonomic()) {
        control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
        xt::view(control_sequence_.vy, -1) =
            xt::view(control_sequence_.vy, -2);
    }**/


    /*****************************************************************************/
    control_sequence_.arr_vx.segment(0, control_sequence_.arr_vx.rows() - 1) =
        control_sequence_.arr_vx.segment(1, control_sequence_.arr_vx.rows() - 1);

    control_sequence_.arr_wz.segment(0, control_sequence_.arr_wz.rows() - 1) =
        control_sequence_.arr_wz.segment(1, control_sequence_.arr_wz.rows() - 1);

    if (isHolonomic())
    {
        control_sequence_.arr_vy.segment(0, control_sequence_.arr_vy.rows() - 1) =
            control_sequence_.arr_vy.segment(1, control_sequence_.arr_vy.rows() - 1);
    }

    //std::cout << control_sequence_.arr_vx << std::endl;
    /*****************************************************************************/
}

void Optimizer::generateNoisedTrajectories()
{
    noise_generator_.setNoisedControls(state_, control_sequence_);
    noise_generator_.generateNextNoises();
    updateStateVelocities(state_);
    integrateStateVelocities(generated_trajectories_, state_);
}

void Optimizer::applyControlSequenceConstraints()
{
    auto & s = settings_;

    /**
    if (isHolonomic()) {
        control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
    }

    control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
    control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);**/


    /*************************************************************************************/
    control_sequence_.arr_vx.cwiseMax(s.constraints.vx_min).cwiseMin(s.constraints.vx_max);
    control_sequence_.arr_wz.cwiseMax(-s.constraints.wz).cwiseMin(s.constraints.wz);
    if (isHolonomic())
    {
        control_sequence_.arr_vy.cwiseMax(-s.constraints.vy).cwiseMin(s.constraints.vy);
    }
    /*************************************************************************************/


    motion_model_->applyConstraints(control_sequence_);
}

void Optimizer::updateStateVelocities(
    models::State & state) const
{
    updateInitialStateVelocities(state);
    propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
    models::State & state) const
{
    /**
    xt::noalias(xt::view(state.vx, xt::all(), 0)) = static_cast<float>(state.speed.linear.x);
    xt::noalias(xt::view(state.wz, xt::all(), 0)) = static_cast<float>(state.speed.angular.z);

    if (isHolonomic()) {
        xt::noalias(xt::view(state.vy, xt::all(), 0)) = static_cast<float>(state.speed.linear.y);
    }**/

    /**************************************************************************/
    state.arr_vx.col(0) = static_cast<float>(state.speed.linear.x);
    state.arr_wz.col(0) = static_cast<float>(state.speed.angular.z);
    if (isHolonomic())
    {
        state.arr_vy.col(0) = static_cast<float>(state.speed.linear.y);
    }
    /**************************************************************************/
}

void Optimizer::propagateStateVelocitiesFromInitials(
    models::State & state) const
{
    motion_model_->predict(state);
}

/**
void Optimizer::integrateStateVelocities(
    xt::xtensor<float, 2> & trajectory,
    const xt::xtensor<float, 2> & sequence) const
{
    float initial_yaw = static_cast<float>(tf2::getYaw(state_.pose.pose.orientation));

    const auto vx = xt::view(sequence, xt::all(), 0);
    const auto vy = xt::view(sequence, xt::all(), 2);
    const auto wz = xt::view(sequence, xt::all(), 1);

    auto traj_x = xt::view(trajectory, xt::all(), 0);
    auto traj_y = xt::view(trajectory, xt::all(), 1);
    auto traj_yaws = xt::view(trajectory, xt::all(), 2);

    xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

    //auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
    //auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
    auto yaw_cos = xt::roll(xt::eval(xt::cos(traj_yaws)), 1);
    auto yaw_sin = xt::roll(xt::eval(xt::sin(traj_yaws)), 1);
    xt::view(yaw_cos, 0) = cosf(initial_yaw);
    xt::view(yaw_sin, 0) = sinf(initial_yaw);

    const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

//    xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
//    xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
//    xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
//    xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

    auto && dx = xt::eval(vx * yaw_cos);
    auto && dy = xt::eval(vx * yaw_sin);

    if (isHolonomic())
    {
        const auto vy = xt::view(sequence, xt::all(), 2);
        dx = dx - vy * yaw_sin;
        dy = dy + vy * yaw_cos;
    }

    xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
    xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}**/

/***********************************************************************************/
void Optimizer::integrateStateVelocities(
    Eigen::ArrayXXf & trajectory,
    const Eigen::ArrayXXf & sequence) const
{
    float initial_yaw = static_cast<float>(tf2::getYaw(state_.pose.pose.orientation));

    Eigen::ArrayXf vx = sequence.col(0);
    Eigen::ArrayXf wz = sequence.col(1);

    Eigen::ArrayXf traj_x = trajectory.col(0);
    Eigen::ArrayXf traj_y = trajectory.col(1);
    Eigen::ArrayXf traj_yaws = trajectory.col(2);

    Eigen::ArrayXf dwz = wz * settings_.model_dt;
    for (unsigned int i = 1; i < dwz.rows(); ++i)
    {
        dwz(i) += dwz(i-1);
    }

    traj_yaws = dwz + initial_yaw;

    Eigen::ArrayXf yaw_cos = traj_yaws.cos();
    Eigen::ArrayXf yaw_sin = traj_yaws.sin();

    Eigen::ArrayXf dx = vx * yaw_cos;
    Eigen::ArrayXf dy = vx * yaw_sin;

    if (isHolonomic())
    {
        Eigen::ArrayXf vy = sequence.col(2);
        dx = dx - vy * yaw_sin;
        dy = dy + vy * yaw_cos;
    }

    Eigen::ArrayXf dxt = dx * settings_.model_dt;
    Eigen::ArrayXf dyt = dy * settings_.model_dt;

    for (unsigned int i = 1; i < dxt.rows(); ++i)
    {
        dxt(i) += dxt(i-1);
        dyt(i) += dyt(i-1);
    }

    trajectory.col(0) = state_.pose.pose.position.x + dxt;
    trajectory.col(1) = state_.pose.pose.position.y + dyt;
    trajectory.col(2) = traj_yaws;
}
/****************************************************************************************/


void Optimizer::integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const
{
    const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

    /**
    xt::noalias(trajectories.yaws) =
        xt::cumsum(state.wz * settings_.model_dt, {1}) + initial_yaw;**/

    /****************************************************************/
    Eigen::ArrayXXf dwz = state.arr_wz * settings_.model_dt;
    for (unsigned int i = 0; i < dwz.rows(); ++i)
    {
        for (unsigned int j = 1; j < dwz.cols(); ++j)
        {
            dwz(i, j) += dwz(i, j-1);
        }
    }
    trajectories.arr_yaws = dwz + initial_yaw;
    /****************************************************************/

    /**
    auto yaw_cos = xt::roll(xt::eval(xt::cos(trajectories.yaws)), 1, 1);
    auto yaw_sin = xt::roll(xt::eval(xt::sin(trajectories.yaws)), 1, 1);
    xt::view(yaw_cos, xt::all(), 0) = cosf(initial_yaw);
    xt::view(yaw_sin, xt::all(), 0) = sinf(initial_yaw);**/

    /****************************************************************/
    Eigen::ArrayXXf arr_yaw_cos = trajectories.arr_yaws.cos();
    Eigen::ArrayXXf arr_yaw_sin = trajectories.arr_yaws.sin();
    /****************************************************************/


    /**
    auto && dx = xt::eval(state.vx * yaw_cos);
    auto && dy = xt::eval(state.vx * yaw_sin);**/


    /****************************************************************/
    Eigen::ArrayXXf arr_dx = state.arr_vx * arr_yaw_cos;
    Eigen::ArrayXXf arr_dy = state.arr_vx * arr_yaw_sin;
    /****************************************************************/


    if (isHolonomic()) {

        /**
        dx = dx - state.vy * yaw_sin;
        dy = dy + state.vy * yaw_cos;**/

        /********************************************/
        arr_dx = arr_dx - state.arr_vy * arr_yaw_sin;
        arr_dy = arr_dy + state.arr_vy * arr_yaw_cos;
        /********************************************/
    }


    /**
    xt::noalias(trajectories.x) = state.pose.pose.position.x +
                                  xt::cumsum(dx * settings_.model_dt, {1});
    xt::noalias(trajectories.y) = state.pose.pose.position.y +
                                  xt::cumsum(dy * settings_.model_dt, {1});**/


    /******************************************************************************/
    Eigen::ArrayXXf dxt = arr_dx * settings_.model_dt;
    Eigen::ArrayXXf dyt = arr_dy * settings_.model_dt;
    for (unsigned int i = 0; i < dxt.rows(); ++i)
    {
        for (unsigned int j = 1; j < dxt.cols(); ++j)
        {
            dxt(i, j) += dxt(i, j-1);
            dyt(i, j) += dyt(i, j-1);
        }
    }
    trajectories.arr_x = state.pose.pose.position.x + dxt;
    trajectories.arr_y = state.pose.pose.position.y + dyt;
    /******************************************************************************/
}

/**
xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
    const bool is_holo = isHolonomic();
    auto && sequence =
        xt::xtensor<float, 2>::from_shape({settings_.time_steps, is_holo ? 3u : 2u});
    auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

    xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
    xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

    if (is_holo) {
        xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
    }

    integrateStateVelocities(trajectories, sequence);
    return std::move(trajectories);
}**/

/*********************************************************************************/
Eigen::ArrayXXf Optimizer::arrGetOptimizedTrajectory()
{
    const bool is_holo = isHolonomic();
    Eigen::ArrayXXf sequence(settings_.time_steps, is_holo ? 3u : 2u);
    Eigen::ArrayXXf trajectories(settings_.time_steps, 3u);

    sequence.col(0) = control_sequence_.arr_vx;
    sequence.col(1) = control_sequence_.arr_wz;

    if (is_holo)
    {
        sequence.col(2) = control_sequence_.arr_vy;
    }

    integrateStateVelocities(trajectories, sequence);
    return std::move(trajectories);
}
/*********************************************************************************/

void Optimizer::updateControlSequence()
{
    const bool is_holo = isHolonomic();
    auto & s = settings_;

    /**
    auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
    auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
    xt::noalias(costs_) +=
        s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
                                                   xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
    xt::noalias(costs_) +=
        s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
                                                   xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

    if (is_holo) {
        auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
        xt::noalias(costs_) +=
            s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
                                                       xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
                                                       1, immediate);
    }**/


    /*********************************************************************************************************/
    for (unsigned int i = 0; i < arr_costs_.rows(); ++i)
    {
        // TODO: add powf2 to smoothness
        arr_costs_(i) +=
            s.gamma / powf(s.sampling_std.vx, 2) *
            ((state_.arr_cvx.row(i) - control_sequence_.arr_vx.transpose()) *
                  control_sequence_.arr_vx.transpose()).sum();
        arr_costs_(i) +=
            s.gamma / powf(s.sampling_std.wz, 2) *
            ((state_.arr_cwz.row(i) - control_sequence_.arr_wz.transpose()) *
                  control_sequence_.arr_wz.transpose()).sum();
        if (is_holo)
        {
            arr_costs_(i) +=
                s.gamma / powf(s.sampling_std.vy, 2) *
                ((state_.arr_vy.row(i) - control_sequence_.arr_vy.transpose()) *
                      control_sequence_.arr_vy.transpose()).sum();
        }
//        arr_costs_(i) +=
//            s.gamma / powf(s.sampling_std.vx, 2) *
//            powf(((state_.arr_cvx.row(i) - control_sequence_.arr_vx.transpose()) *
//                  control_sequence_.arr_vx.transpose()).sum(), 2);
//        arr_costs_(i) +=
//            s.gamma / powf(s.sampling_std.wz, 2) *
//            powf(((state_.arr_cwz.row(i) - control_sequence_.arr_wz.transpose()) *
//                  control_sequence_.arr_wz.transpose()).sum(), 2);
//        if (is_holo)
//        {
//            arr_costs_(i) +=
//                s.gamma / powf(s.sampling_std.vy, 2) *
//                powf(((state_.arr_vy.row(i) - control_sequence_.arr_vy.transpose()) *
//                      control_sequence_.arr_vy.transpose()).sum(), 2);
//        }
    }
    /*********************************************************************************************************/


    /**
    auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
    auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
    auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
    auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

    xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
    xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
    if (is_holo) {
        xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
    }**/


    /*********************************************************************************************************/
    Eigen::ArrayXf arr_costs_normalized = arr_costs_ - arr_costs_.minCoeff();
    Eigen::ArrayXf arr_exponents = ((-1 / settings_.temperature) * arr_costs_normalized).exp();
    Eigen::ArrayXf arr_softmaxes = arr_exponents / (arr_exponents.sum());

    for (unsigned int i = 0; i < control_sequence_.arr_vx.rows(); ++i)
    {
        control_sequence_.arr_vx(i) = (state_.arr_cvx.col(i) * arr_softmaxes).sum();
        control_sequence_.arr_wz(i) = (state_.arr_cwz.col(i) * arr_softmaxes).sum();
        if (is_holo)
        {
            control_sequence_.arr_vy(i) = (state_.arr_cvy.col(i) * arr_softmaxes).sum();
        }
    }
    /*********************************************************************************************************/


    applyControlSequenceConstraints();
}

geometry_msgs::TwistStamped Optimizer::getControlFromSequenceAsTwist(
    const ros::Time & stamp)
{
    unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

    /**
    auto vx = control_sequence_.vx(offset);
    auto wz = control_sequence_.wz(offset);**/

    float arr_vx = control_sequence_.arr_vx(offset);
    float arr_wz = control_sequence_.arr_wz(offset);

    if (isHolonomic()) {

        /**
        auto vy = control_sequence_.vy(offset);**/

        float arr_vy = control_sequence_.arr_vy(offset);

        /**
        return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());**/

        return utils::toTwistStamped(arr_vx, arr_vy, arr_wz, stamp, costmap_ros_->getBaseFrameID());
    }

    /**
    return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());**/

    return utils::toTwistStamped(arr_vx, arr_wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & model)
{
    if (model == "DiffDrive") {
        motion_model_ = std::make_shared<DiffDriveMotionModel>();
    } else if (model == "Omni") {
        motion_model_ = std::make_shared<OmniMotionModel>();
    } else if (model == "Ackermann") {
        motion_model_ = std::make_shared<AckermannMotionModel>(nh_);
    } else {
//        throw nav2_core::ControllerException(
//            std::string(
//                "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
//                                   "or Ackermann"));
        ROS_ERROR("Model is not valid! Valid options are DiffDrive, Omni, "
                                   "or Ackermann");
    }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
    auto & s = settings_;
    // TODO: no speed limit in costmap?
    double costmap_no_speed_limit  = 1.0;
    if (speed_limit == costmap_no_speed_limit) {
        s.constraints.vx_max = s.base_constraints.vx_max;
        s.constraints.vx_min = s.base_constraints.vx_min;
        s.constraints.vy = s.base_constraints.vy;
        s.constraints.wz = s.base_constraints.wz;
    }
    else {
        if (percentage) {
            // Speed limit is expressed in % from maximum speed of robot
            double ratio = speed_limit / 100.0;
            s.constraints.vx_max = s.base_constraints.vx_max * ratio;
            s.constraints.vx_min = s.base_constraints.vx_min * ratio;
            s.constraints.vy = s.base_constraints.vy * ratio;
            s.constraints.wz = s.base_constraints.wz * ratio;
        }
        else {
            // Speed limit is expressed in absolute value
            double ratio = speed_limit / s.base_constraints.vx_max;
            s.constraints.vx_max = s.base_constraints.vx_max * ratio;
            s.constraints.vx_min = s.base_constraints.vx_min * ratio;
            s.constraints.vy = s.base_constraints.vy * ratio;
            s.constraints.wz = s.base_constraints.wz * ratio;
        }
    }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
    return generated_trajectories_;
}

// gx
void Optimizer::setCurrentFurthest(CriticData &data)
{
    float current_curvature = curvatureEstimate(data, settings_.curvature_estimate_point_);

    /**
    auto offseted_idx = std::min(settings_.look_ahead_, data.path.x.shape(0));**/

    /************************************************************************************************/
    unsigned int offseted_idx = std::min(float(settings_.look_ahead_), float(data.path.arr_x.rows()));
    /************************************************************************************************/

//    if (current_curvature > 0.06)
//    {
//        if (current_curvature < 0.1)
//        {
//            offseted_idx = offseted_idx - 1 >= 0 ? offseted_idx - 1 : offseted_idx;
//        }
//        else if (current_curvature < 0.25)
//        {
//            offseted_idx = offseted_idx - 2 >= 0 ? offseted_idx - 2 :
//                                               offseted_idx - 1 >= 0 ? offseted_idx - 1 : offseted_idx;
//        }
//        else if (current_curvature < 0.5)
//        {
//            offseted_idx = offseted_idx - 3 >= 0 ? offseted_idx - 3 :
//                                               offseted_idx - 2 >= 0 ? offseted_idx - 2 :
//                                               offseted_idx - 1 >= 0 ? offseted_idx - 1 : offseted_idx;
//        }
//        else
//        {
//            offseted_idx = offseted_idx - 4 >= 0 ? offseted_idx - 4 :
//                                                 offseted_idx - 3 >= 0 ? offseted_idx - 3 :
//                                                 offseted_idx - 2 >= 0 ? offseted_idx - 2 :
//                                                 offseted_idx - 1 >= 0 ? offseted_idx - 1 : offseted_idx;
//        }
//    }

    data.current_furthest_ = offseted_idx;

    //std::cout << "current furthest: " << data.current_furthest_ << std::endl;
}

// gx
float Optimizer::curvatureEstimate(CriticData &data, size_t curvature_estimate_point)
{
    /**
    curvature_estimate_point = std::min(curvature_estimate_point, data.path.x.shape(0));
    if (curvature_estimate_point < 1)
    {
        return std::numeric_limits<float>::max();
    }
    float curvature = 0.0f;
    for (size_t i = 1; i < curvature_estimate_point; ++i)
    {
        curvature += fabs(angles::shortest_angular_distance(data.path.yaws(i), data.path.yaws(i-1)));
    }
    curvature += fabs(angles::shortest_angular_distance(data.path.yaws(0), tf2::getYaw(data.state.pose.pose.orientation)));
    return curvature / static_cast<float>(curvature_estimate_point + 1);**/

    /*******************************************************************************************************************/
    curvature_estimate_point = std::min(float(curvature_estimate_point), float(data.path.arr_x.rows()));
    if (curvature_estimate_point < 1)
    {
        return std::numeric_limits<float>::max();
    }
    float arr_curvature = 0.0f;
    for (size_t i = 1; i < curvature_estimate_point; ++i)
    {
        arr_curvature += fabs(angles::shortest_angular_distance(data.path.arr_yaws(i), data.path.arr_yaws(i-1)));
    }
    arr_curvature += fabs(angles::shortest_angular_distance(data.path.arr_yaws(0), tf2::getYaw(data.state.pose.pose.orientation)));
    return arr_curvature / static_cast<float>(curvature_estimate_point + 1);
    /*******************************************************************************************************************/
}


}  // namespace mppi
