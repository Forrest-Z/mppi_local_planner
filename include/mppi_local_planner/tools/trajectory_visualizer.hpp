#ifndef MPPI_LOCAL_PLANNER_TOOLS_TRAJECTORY_VISUALIZER_HPP
#define MPPI_LOCAL_PLANNER_TOOLS_TRAJECTORY_VISUALIZER_HPP

#include <mppi_local_planner/tools/mppi_config.hpp>
#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/models/trajectories.hpp>

#include <visualization_msgs/MarkerArray.h>

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 */
class TrajectoryVisualizer
{
public:
    /**
    * @brief Constructor for mppi::TrajectoryVisualizer
    */
    TrajectoryVisualizer() = default;

    /**
    * @brief Configure trajectory visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param frame_id Frame to publish trajectories in
    * @param dynamic_parameter_handler Parameter handler object
    */
    void on_configure(
        ros::NodeHandle* nh, const std::string & name,
        const std::string & frame_id);

    /**
    * @brief Cleanup object on shutdown
    */
    void on_cleanup();

    /**
    * @brief Activate object
    */
    void on_activate();

    /**
    * @brief Deactivate object
    */
    void on_deactivate();

    /**
    * @brief Add an optimal trajectory to visualize
    * @param trajectory Optimal trajectory
    */
    /**
    void add(const xt::xtensor<float, 2> & trajectory, const std::string & marker_namespace);**/

    void add(const Eigen::ArrayXXf & trajectory, const std::string & marker_namespace);

    /**
    * @brief Add candidate trajectories to visualize
    * @param trajectories Candidate trajectories
    */
    void add(const models::Trajectories & trajectories, const std::string & marker_namespace);

    /**
    * @brief Visualize the plan
    * @param plan Plan to visualize
    */
    void visualize(const nav_msgs::Path & plan);

    /**
    * @brief Reset object
    */
    void reset();

protected:
    std::string frame_id_;

    ros::NodeHandle* nh_;

    // mark array
    ros::Publisher trajectories_publisher_;
    ros::Publisher transformed_path_pub_;

    std::unique_ptr<visualization_msgs::MarkerArray> points_;
    int marker_id_ = 0;

    size_t trajectory_step_{0};
    size_t time_step_{0};
};

}  // namespace mppi

#endif // MPPI_LOCAL_PLANNER_TOOLS_TRAJECTORY_VISUALIZER_HPP
