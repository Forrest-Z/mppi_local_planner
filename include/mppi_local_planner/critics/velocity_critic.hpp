#ifndef MPPI_LOCAL_PLANNER_CRITICS_VELOCITY_CRITIC_HPP
#define MPPI_LOCAL_PLANNER_CRITICS_VELOCITY_CRITIC_HPP

#include <mppi_local_planner/critic_function.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::VelocityCritic
 * @brief Critic objective function for penalizing low velocity
 */
class VelocityCritic : public CriticFunction
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
    unsigned int power_{0};
    float weight_{0};
    float threshold_to_consider_{0};
    float max_vel_x_{0};
};

}  // namespace mppi::critics



#endif // MPPI_LOCAL_PLANNER_CRITICS_VELOCITY_CRITIC_HPP
