#ifndef MPPI_LOCAL_PLANNER_CRITICS_CONSTRAINT_CRITIC_HPP
#define MPPI_LOCAL_PLANNER_CRITICS_CONSTRAINT_CRITIC_HPP

#include <mppi_local_planner/critic_function.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction
{
public:
    /**
    * @brief Initialize critic
    */
    void initialize(MPPIConfig& cfg) override;

    /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
    void score(CriticData & data) override;

    float getMaxVelConstraint() {return max_vel_;}
    float getMinVelConstraint() {return min_vel_;}

    void reconfigure(const MPPIConfig* cfg);

protected:
    unsigned int power_{0};
    float weight_{0};
    float min_vel_;
    float max_vel_;
    float wz_max_;
    float vx_acc_max_;
    float wz_acc_max_;
};

}  // namespace mppi::critics

#endif // MPPI_LOCAL_PLANNER_CRITICS_CONSTRAINT_CRITIC_HPP
