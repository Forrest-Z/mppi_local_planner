#ifndef MPPI_LOCAL_PLANNER_MODELS_OPTIMIZER_SETTINGS_HPP
#define MPPI_LOCAL_PLANNER_MODELS_OPTIMIZER_SETTINGS_HPP

#include <cstddef>
#include <mppi_local_planner/models/constraints.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings
{
    models::ControlConstraints base_constraints{0.0f, 0.0f, 0.0f, 0.0f};
    models::ControlConstraints constraints{0.0f, 0.0f, 0.0f, 0.0f};
    models::SamplingStd sampling_std{0.0f, 0.0f, 0.0f};
    float model_dt{0.0f};
    float temperature{0.0f};
    float gamma{0.0f};
    unsigned int batch_size{0u};
    unsigned int time_steps{0u};
    unsigned int iteration_count{0u};
    bool shift_control_sequence{false};
    size_t retry_attempt_limit{0};
    size_t look_ahead_{5};
    size_t curvature_estimate_point_{6};
    bool noise_regenerate;
};

}  // namespace mppi::models


#endif // MPPI_LOCAL_PLANNER_MODELS_OPTIMIZER_SETTINGS_HPP
