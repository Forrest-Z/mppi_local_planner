#ifndef MPPI_LOCAL_PLANNER_MODELS_TRAJECTORIES_HPP
#define MPPI_LOCAL_PLANNER_MODELS_TRAJECTORIES_HPP


#include <Eigen/Dense>
//#include <unsupported/Eigen/CXX11/Tensor>

namespace mppi::models
{

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories
{
    /**
    xt::xtensor<float, 2> x;
    xt::xtensor<float, 2> y;
    xt::xtensor<float, 2> yaws;**/

    Eigen::ArrayXXf arr_x;
    Eigen::ArrayXXf arr_y;
    Eigen::ArrayXXf arr_yaws;

    /**
    * @brief Reset state data
    */
    void reset(unsigned int batch_size, unsigned int time_steps)
    {
        /**
        x = xt::zeros<float>({batch_size, time_steps});
        y = xt::zeros<float>({batch_size, time_steps});
        yaws = xt::zeros<float>({batch_size, time_steps});**/

        arr_x.resize(batch_size, time_steps);
        arr_x.setZero();
        arr_y.resize(batch_size, time_steps);
        arr_y.setZero();
        arr_yaws.resize(batch_size, time_steps);
        arr_yaws.setZero();
    }
};

}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_TRAJECTORIES_HPP
