#ifndef MPPI_LOCAL_PLANNER_CRITIC_FUNCTION_HPP
#define MPPI_LOCAL_PLANNER_CRITIC_FUNCTION_HPP

#include <string>
#include <memory>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <mppi_local_planner/critic_data.hpp>

#include <mppi_local_planner/tools/mppi_config.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
    float cost{0};
    bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
    /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
    CriticFunction() = default;

    /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
    virtual ~CriticFunction() = default;

    /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
    void on_configure(
        ros::NodeHandle* nh,
        const std::string & parent_name,
        const std::string & name,
        costmap_2d::Costmap2DROS* costmap_ros,
        MPPIConfig& cfg)
    {
        nh_ = nh;
        name_ = name;
        parent_name_ = parent_name;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        // TODO: get param form node handle
        enabled_ = true;

        initialize(cfg);
    }

    /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
    virtual void score(CriticData & data) = 0;

    /**
    * @brief Initialize critic
    */
    virtual void initialize(MPPIConfig& cfg) = 0;

    /**
    * @brief Get name of critic
    */
    std::string getName()
    {
        return name_;
    }


protected:
    bool enabled_;
    std::string name_, parent_name_;
    ros::NodeHandle* nh_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_{nullptr};
    const MPPIConfig* cfg_;
};

}  // namespace mppi::critics


#endif // MPPI_LOCAL_PLANNER_CRITIC_FUNCTION_HPP
