#ifndef MPPI_LOCAL_PLANNER_CRITICS_PATH_ALIGN_LEGACY_CRITIC_HPP
#define MPPI_LOCAL_PLANNER_CRITICS_PATH_ALIGN_LEGACY_CRITIC_HPP

#include <mppi_local_planner/critic_function.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::PathAlignLegacyCritic
 * @brief Critic objective function for aligning to the path. Note:
 * High settings of this will follow the path more precisely, but also makes it
 * difficult (or impossible) to deviate in the presence of dynamic obstacles.
 * This is an important critic to tune and consider in tandem with Obstacle.
 * This is the initial 'Legacy' implementation before replacement Oct 2023.
 */
class PathAlignLegacyCritic : public CriticFunction
{
public:
    /**
    * @brief Initialize critic
    */
    void initialize(MPPIConfig& cfg) override;

    /**
   * @brief Evaluate cost related to trajectories path alignment
   *
   * @param costs [out] add reference cost values to this tensor
   */
    void score(CriticData & data) override;

    void reconfigure(const MPPIConfig* cfg);

protected:
    size_t offset_from_furthest_{0};
    int trajectory_point_step_{0};
    float threshold_to_consider_{0};
    float max_path_occupancy_ratio_{0};
    bool use_path_orientations_{false};
    float orientation_weight_{0.05};
    unsigned int power_{0};
    float weight_{0};
};

}  // namespace mppi::critics

#endif // MPPI_LOCAL_PLANNER_CRITICS_PATH_ALIGN_LEGACY_CRITIC_HPP
