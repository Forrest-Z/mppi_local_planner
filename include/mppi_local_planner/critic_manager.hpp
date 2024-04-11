#ifndef MPPI_LOCAL_PLANNER_CRITIC_MANAGER_HPP
#define MPPI_LOCAL_PLANNER_CRITIC_MANAGER_HPP

#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>

#include <mppi_local_planner/tools/utils.hpp>
#include <mppi_local_planner/critic_data.hpp>
#include <mppi_local_planner/critic_function.hpp>

#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */
class CriticManager
{
public:
    /**
    * @brief Constructor for mppi::CriticManager
    */
    CriticManager() = default;


    /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
    virtual ~CriticManager() = default;

    /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
    void on_configure(
        ros::NodeHandle* nh, const std::string & name,
        costmap_2d::Costmap2DROS* costmap_ros, MPPIConfig& cfg);

    /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */
    void evalTrajectoriesScores(CriticData & data) const;

protected:
    /**
    * @brief Get parameters (critics to load)
    */
    void getParams();

    /**
    * @brief Load the critic plugins
    */
    virtual void loadCritics(MPPIConfig& cfg);

    /**
    * @brief Get full-name namespaced critic IDs
    */
    std::string getFullName(const std::string & name);

protected:
    ros::NodeHandle* nh_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::string name_;

    std::vector<std::string> critic_names_;
    std::unique_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;
    std::vector<std::unique_ptr<critics::CriticFunction>> critics_;
};

}  // namespace mppi

#endif // MPPI_LOCAL_PLANNER_CRITIC_MANAGER_HPP
