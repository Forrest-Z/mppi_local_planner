#ifndef MPPI_LOCAL_PLANNER_CRITICS_PATH_FOLLOW_CRITIC_HPP
#define MPPI_LOCAL_PLANNER_CRITICS_PATH_FOLLOW_CRITIC_HPP

#include <mppi_local_planner/critic_function.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for following the path approximately
 * To allow for deviation from path in case of dynamic obstacles. Path Align
 * is what aligns the trajectories to the path more or less precisely, if desireable.
 * A higher weight here with an offset > 1 will accelerate the samples to full speed
 * faster and push the follow point further ahead, creating some shortcutting.
 */
class PathFollowCritic : public CriticFunction
{
public:
    /**
    * @brief Initialize critic
    */
    void initialize(MPPIConfig& cfg) override;

    /**
   * @brief Evaluate cost related to robot orientation at goal pose
   * (considered only if robot near last goal in current plan)
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
    void score(CriticData & data) override;

    void reconfigure(const MPPIConfig* cfg);

protected:
    float threshold_to_consider_{0.0f};
    size_t offset_from_furthest_{0};
    size_t lookahead_{10};

    unsigned int power_{0u};
    float weight_{0.0f};

};

}  // namespace mppi::critics

#endif // MPPI_LOCAL_PLANNER_CRITICS_PATH_FOLLOW_CRITIC_HPP
