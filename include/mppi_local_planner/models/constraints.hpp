#ifndef MPPI_LOCAL_PLANNER_MODELS_CONSTRAINTS_HPP
#define MPPI_LOCAL_PLANNER_MODELS_CONSTRAINTS_HPP

namespace mppi::models
{

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
    float vx_max;
    float vx_min;
    float vy;
    float wz;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd
{
    float vx;
    float vy;
    float wz;
};

}  // namespace mppi::models

#endif // MPPI_LOCAL_PLANNER_MODELS_CONSTRAINTS_HPP
