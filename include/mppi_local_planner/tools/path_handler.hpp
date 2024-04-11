#ifndef MPPI_LOCAL_PLANNER_TOOLS_PATH_HANDLER_HPP
#define MPPI_LOCAL_PLANNER_TOOLS_PATH_HANDLER_HPP

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <mppi_local_planner/tools/mppi_config.hpp>


namespace mppi
{

using PathIterator = std::vector<geometry_msgs::PoseStamped>::iterator;
using PathRange = std::pair<PathIterator, PathIterator>;

/**
 * @class mppi::PathHandler
 * @brief Manager of incoming reference paths for transformation and processing
 */

class PathHandler
{
public:
    /**
    * @brief Constructor for mppi::PathHandler
    */
    PathHandler() = default;

    /**
    * @brief Destructor for mppi::PathHandler
    */
    ~PathHandler() = default;

    /**
    * @brief Initialize path handler on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param tf TF buffer for transformations
    * @param dynamic_parameter_handler Parameter handler object
    */
    void initialize(ros::NodeHandle* nh, const std::string & name,
        costmap_2d::Costmap2DROS* costmap, tf2_ros::Buffer* buffer);

    /**
    * @brief Set new reference path
    * @param Plan Path to use
    */
    bool setPath(const nav_msgs::Path & plan);

    /**
    * @brief Get reference path
    * @return Path
    */
    nav_msgs::Path & getPath();

    /**
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param robot_pose Pose of robot
   * @return global plan in local frame
   */
    nav_msgs::Path transformPath(const geometry_msgs::PoseStamped & robot_pose);

protected:
    /**
    * @brief Transform a pose to another frame
    * @param frame Frame to transform to
    * @param in_pose Input pose
    * @param out_pose Output pose
    * @return Bool if successful
    */
    bool transformPose(
        const std::string & frame, const geometry_msgs::PoseStamped & in_pose,
        geometry_msgs::PoseStamped & out_pose) const;

    /**
    * @brief Get largest dimension of costmap (radially)
    * @return Max distance from center of costmap to edge
    */
    double getMaxCostmapDist();

    /**
    * @brief Transform a pose to the global reference frame
    * @param pose Current pose
    * @return output poose in global reference frame
    */
    geometry_msgs::PoseStamped
    transformToGlobalPlanFrame(const geometry_msgs::PoseStamped & pose);

    /**
    * @brief Get global plan within window of the local costmap size
    * @param global_pose Robot pose
    * @return plan transformed in the costmap frame and iterator to the first pose of the global
    * plan (for pruning)
    */
    std::pair<nav_msgs::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
        const geometry_msgs::PoseStamped & global_pose);

    /**
    * @brief Prune a path to only interesting portions
    * @param plan Plan to prune
    * @param end Final path iterator
    */
    void prunePlan(nav_msgs::Path & plan, const PathIterator end);

    /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
    bool isWithinInversionTolerances(const geometry_msgs::PoseStamped & robot_pose);

    std::string name_;
    costmap_2d::Costmap2DROS* costmap_;
    tf2_ros::Buffer* tf_buffer_;
    ros::NodeHandle* nh_;

    nav_msgs::Path global_plan_;
    nav_msgs::Path global_plan_up_to_inversion_;

    double max_robot_pose_search_dist_{0};
    double prune_distance_{0};
    double transform_tolerance_{0};
    float inversion_xy_tolerance_{0.2};
    float inversion_yaw_tolerance{0.4};
    bool enforce_path_inversion_{false};
    unsigned int inversion_locale_{0u};
};
}  // namespace mppi

#endif // MPPI_LOCAL_PLANNER_TOOLS_PATH_HANDLER_HPP
