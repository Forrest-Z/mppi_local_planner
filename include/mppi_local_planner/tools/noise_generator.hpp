#ifndef MPPI_LOCAL_PLANNER_TOOLS_NOISE_GENERATOR_HPP
#define MPPI_LOCAL_PLANNER_TOOLS_NOISE_GENERATOR_HPP

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <random>

#include <mppi_local_planner/models/optimizer_settings.hpp>
#include <mppi_local_planner/models/control_sequence.hpp>
#include <mppi_local_planner/models/state.hpp>
#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 */
class NoiseGenerator
{
public:
    /**
    * @brief Constructor for mppi::NoiseGenerator
    */
    NoiseGenerator() = default;

    /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   * @param name Namespace for configs
   * @param param_handler Get parameters util
   */
    void initialize(
        mppi::models::OptimizerSettings & settings,
        bool is_holonomic, const std::string & name, ros::NodeHandle* nh);

    /**
   * @brief Shutdown noise generator thread
   */
    void shutdown();

    /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
    void generateNextNoises();

    /**
   * @brief set noised control_sequence to state controls
   * @return noises vx, vy, wz
   */
    void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

    /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
    void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
    /**
   * @brief Thread to execute noise generation process
   */
    void noiseThread();

    /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
    void generateNoisedControls();

    /**
    xt::xtensor<float, 2> noises_vx_;
    xt::xtensor<float, 2> noises_vy_;
    xt::xtensor<float, 2> noises_wz_;**/

    Eigen::ArrayXXf arr_noises_vx_;
    Eigen::ArrayXXf arr_noises_vy_;
    Eigen::ArrayXXf arr_noises_wz_;

    mppi::models::OptimizerSettings settings_;
    bool is_holonomic_;

    std::thread noise_thread_;
    std::condition_variable noise_cond_;
    std::mutex noise_lock_;
    bool active_{false}, ready_{false}, regenerate_noises_{false};

private:
    ros::NodeHandle* nh_;
};

}  // namespace mppi


#endif // MPPI_LOCAL_PLANNER_TOOLS_NOISE_GENERATOR_HPP
