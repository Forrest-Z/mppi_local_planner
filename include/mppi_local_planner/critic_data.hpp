#ifndef MPPI_LOCAL_PLANNER_CRITIC_DATA_HPP
#define MPPI_LOCAL_PLANNER_CRITIC_DATA_HPP

#include <memory>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/models/trajectories.hpp>
#include <mppi_local_planner/models/path.hpp>
#include <mppi_local_planner/motion_models.hpp>


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories, path, costs, and
 * important parameters to share
 */
struct CriticData
{
    const models::State & state;
    const models::Trajectories & trajectories;
    const models::Path & path;

    /**
    xt::xtensor<float, 1> & costs;**/

    //Eigen::Tensor<float, 1> & eig_costs;

    Eigen::ArrayXf & arr_costs_;

    float & model_dt;

    bool fail_flag;

    std::shared_ptr<MotionModel> motion_model;
    std::optional<std::vector<bool>> path_pts_valid;
    std::optional<size_t> furthest_reached_path_point;

    float pose_tolerance;

    size_t current_furthest_;
};

}  // namespace mppi

#endif // MPPI_LOCAL_PLANNER_CRITIC_DATA_HPP
