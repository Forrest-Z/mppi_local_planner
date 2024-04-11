#ifndef MPPI_LOCAL_PLANNER_MODELS_PATH_HPP
#define MPPI_LOCAL_PLANNER_MODELS_PATH_HPP

#include <Eigen/Dense>
//#include <unsupported/Eigen/CXX11/Tensor>

namespace mppi::models
{

/**
 * @struct mppi::models::Path
 * @brief Path represented as a tensor
 */
struct Path
{
    /**
    xt::xtensor<float, 1> x;
    xt::xtensor<float, 1> y;
    xt::xtensor<float, 1> yaws;**/

    Eigen::ArrayXf arr_x;
    Eigen::ArrayXf arr_y;
    Eigen::ArrayXf arr_yaws;

    /**
    * @brief Reset path data
    */
    void reset(unsigned int size)
    {
        /**
        x = xt::zeros<float>({size});
        y = xt::zeros<float>({size});
        yaws = xt::zeros<float>({size});**/

        arr_x.resize(size);
        arr_y.resize(size);
        arr_yaws.resize(size);
        arr_x.setZero();
        arr_y.setZero();
        arr_yaws.setZero();
    }
};

}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_PATH_HPP
