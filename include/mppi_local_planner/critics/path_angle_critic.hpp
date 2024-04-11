#ifndef MPPI_LOCAL_PLANNER_CRITICS_PATH_ANGLE_CRITIC_HPP
#define MPPI_LOCAL_PLANNER_CRITICS_PATH_ANGLE_CRITIC_HPP

#include <string>
#include <mppi_local_planner/critic_function.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi::critics
{

/**
 * @brief Enum type for different modes of operation
 */
enum class PathAngleMode
{
    FORWARD_PREFERENCE = 0,
    NO_DIRECTIONAL_PREFERENCE = 1,
    CONSIDER_FEASIBLE_PATH_ORIENTATIONS = 2
};

/**
 * @brief Method to convert mode enum to string for printing
 */
std::string modeToStr(const PathAngleMode & mode)
{
    if (mode == PathAngleMode::FORWARD_PREFERENCE) {
        return "Forward Preference";
    } else if (mode == PathAngleMode::CONSIDER_FEASIBLE_PATH_ORIENTATIONS) {
        return "Consider Feasible Path Orientations";
    } else if (mode == PathAngleMode::NO_DIRECTIONAL_PREFERENCE) {
        return "No Directional Preference";
    } else {
        return "Invalid mode!";
    }
}

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for aligning to path in cases of extreme misalignment
 * or turning
 */
class PathAngleCritic : public CriticFunction
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
    float max_angle_to_furthest_{0};
    float threshold_to_consider_{0};

    size_t offset_from_furthest_{0};
    bool reversing_allowed_{true};
    PathAngleMode mode_{0};

    unsigned int power_{0};
    float weight_{0};
};

}  // namespace mppi::critics

#endif // MPPI_LOCAL_PLANNER_CRITICS_PATH_ANGLE_CRITIC_HPP
