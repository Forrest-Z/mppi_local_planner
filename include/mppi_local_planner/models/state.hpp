#ifndef MPPI_LOCAL_PLANNER_MODELS_STATE_HPP
#define MPPI_LOCAL_PLANNER_MODELS_STATE_HPP

#include <Eigen/Dense>
//#include <unsupported/Eigen/CXX11/Tensor>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State
{
    /**
    xt::xtensor<float, 2> vx;
    xt::xtensor<float, 2> vy;
    xt::xtensor<float, 2> wz;

    xt::xtensor<float, 2> cvx;
    xt::xtensor<float, 2> cvy;
    xt::xtensor<float, 2> cwz;**/


    Eigen::ArrayXXf arr_vx;
    Eigen::ArrayXXf arr_vy;
    Eigen::ArrayXXf arr_wz;

    Eigen::ArrayXXf arr_cvx;
    Eigen::ArrayXXf arr_cvy;
    Eigen::ArrayXXf arr_cwz;


    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist speed;

    /**
    * @brief Reset state data
    */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        /**
        vx = xt::zeros<float>({batch_size, time_steps});
        vy = xt::zeros<float>({batch_size, time_steps});
        wz = xt::zeros<float>({batch_size, time_steps});

        cvx = xt::zeros<float>({batch_size, time_steps});
        cvy = xt::zeros<float>({batch_size, time_steps});
        cwz = xt::zeros<float>({batch_size, time_steps});**/

        arr_vx.resize(batch_size, time_steps);
        arr_vx.setZero();
        arr_vy.resize(batch_size, time_steps);
        arr_vy.setZero();
        arr_wz.resize(batch_size, time_steps);
        arr_wz.setZero();

        arr_cvx.resize(batch_size, time_steps);
        arr_cvx.setZero();
        arr_cvy.resize(batch_size, time_steps);
        arr_cvy.setZero();
        arr_cwz.resize(batch_size, time_steps);
        arr_cwz.setZero();
    }
};
}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_STATE_HPP
