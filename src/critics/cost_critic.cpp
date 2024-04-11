#include <cmath>
#include <mppi_local_planner/critics/cost_critic.hpp>

namespace mppi::critics
{

void CostCritic::initialize(MPPIConfig& cfg)
{
    cfg_ = &cfg;
    enabled_ = cfg.cost_critic.enabled;
    power_ = cfg.cost_critic.cost_power;
    weight_ = cfg.cost_critic.cost_weight;
    consider_footprint_ = cfg.cost_critic.consider_footprint;
    critical_cost_ = cfg.cost_critic.critical_cost;
    collision_cost_ = cfg.cost_critic.collision_cost;
    near_goal_distance_ = cfg.cost_critic.near_goal_distance;
    //consider_footprint_ = true; // false
    //power_ = 1;
    //weight_ = 10.0; // 3.81 // 10
    //critical_cost_ = 300.0; // 300.0
    //collision_cost_ = 1000000.0; // 1000000.0
    //near_goal_distance_ = 0.5;
    inflation_layer_name_ = "inflation_layer";

    // Normalized by cost value to put in same regime as other weights
    weight_ /= 254.0f;


    ROS_INFO("cost critic plugin initialized");
    ROS_INFO(
        "InflationCostCritic instantiated with %d power and %f / %f weights. "
        "Critic will collision check based on %s cost.",
        power_, critical_cost_, weight_, consider_footprint_ ?
                            "footprint" : "circular");
}

void CostCritic::reconfigure(const MPPIConfig *cfg)
{
    enabled_ = cfg->cost_critic.enabled;
    power_ = cfg->cost_critic.cost_power;
    weight_ = cfg->cost_critic.cost_weight;
    consider_footprint_ = cfg->cost_critic.consider_footprint;
    critical_cost_ = cfg->cost_critic.critical_cost;
    collision_cost_ = cfg->cost_critic.collision_cost;
    near_goal_distance_ = cfg->cost_critic.near_goal_distance;

    weight_ /= 254.0f;
}

void CostCritic::score(CriticData & data)
{

    reconfigure(cfg_);

    /**
    using xt::evaluation_strategy::immediate;**/

    if (!enabled_) {
        ROS_WARN("Cost critic is disabled");
        return;
    }

    if (consider_footprint_) {
        // footprint may have changed since initialization if user has dynamic footprints
        //possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
    }

    // If near the goal, don't apply the preferential term since the goal is near obstacles
    bool near_goal = false;
    if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
        near_goal = true;
    }

    /**
    auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
    repulsive_cost.fill(0.0);

    const size_t traj_len = data.trajectories.x.shape(1);
    bool all_trajectories_collide = true;
    for (size_t i = 0; i < data.trajectories.x.shape(0); ++i) {
        bool trajectory_collide = false;
        const auto & traj = data.trajectories;
        float pose_cost;

        for (size_t j = 0; j < traj_len; j++) {
            // The costAtPose doesn't use orientation
            // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
            // So the center point has more information than the footprint
            pose_cost = costAtPose(traj.x(i, j), traj.y(i, j));
            if (pose_cost < 1.0f) {continue;}  // In free space

            if (inCollision(pose_cost, traj.x(i, j), traj.y(i, j), traj.yaws(i, j))) {
                ROS_WARN("in collision");
                trajectory_collide = true;
                break;
            }

            // Let near-collision trajectory points be punished severely
            // Note that we collision check based on the footprint actual,
            // but score based on the center-point cost regardless
            using namespace costmap_2d; // NOLINT
            if (pose_cost > INSCRIBED_INFLATED_OBSTACLE) {
                repulsive_cost[i] += 3 * critical_cost_;
            }
            else if (pose_cost > 235)
            {
                repulsive_cost[i] += 1.5 * critical_cost_;
            }
            else if (!near_goal) {  // Generally prefer trajectories further from obstacles
                //repulsive_cost[i] += pose_cost;
                repulsive_cost[i] += pose_cost; // gx modify
            }
        }

        if (!trajectory_collide) {
            all_trajectories_collide = false;
        } else {
            repulsive_cost[i] = collision_cost_;
        }
    }

    data.costs += xt::pow((weight_ * repulsive_cost / traj_len), power_);
    data.fail_flag = all_trajectories_collide;**/



    /**********************************************************************************/
    Eigen::ArrayXf arr_repulsive_cost(data.arr_costs_.rows());
    arr_repulsive_cost.fill(0.0f);

    const size_t arr_traj_len = data.trajectories.arr_x.cols();
    bool arr_all_trajectories_collide = true;
    for (size_t i = 0; i < data.trajectories.arr_x.rows(); ++i) {
        bool trajectory_collide = false;
        const auto & traj = data.trajectories;
        float pose_cost;

        for (size_t j = 0; j < arr_traj_len; j++) {
            // The costAtPose doesn't use orientation
            // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
            // So the center point has more information than the footprint
            pose_cost = costAtPose(traj.arr_x(i, j), traj.arr_y(i, j));
            if (pose_cost < 1.0f) {continue;}  // In free space

            if (inCollision(pose_cost, traj.arr_x(i, j), traj.arr_y(i, j), traj.arr_yaws(i, j))) {
                ROS_WARN("in collision");
                trajectory_collide = true;
                break;
            }

            // Let near-collision trajectory points be punished severely
            // Note that we collision check based on the footprint actual,
            // but score based on the center-point cost regardless
            using namespace costmap_2d; // NOLINT
            if (pose_cost > INSCRIBED_INFLATED_OBSTACLE) {
                arr_repulsive_cost[i] += 3 * critical_cost_;
            }
            else if (pose_cost > 235)
            {
                arr_repulsive_cost[i] += 1.5 * critical_cost_;
            }
            else if (!near_goal) {  // Generally prefer trajectories further from obstacles
                //repulsive_cost[i] += pose_cost;
                arr_repulsive_cost[i] += pose_cost; // gx modify
            }
        }

        if (!trajectory_collide) {
            arr_all_trajectories_collide = false;
        } else {
            arr_repulsive_cost[i] = collision_cost_;
        }
    }

    data.arr_costs_ += (weight_ * arr_repulsive_cost / arr_traj_len).pow(power_);
    data.fail_flag = arr_all_trajectories_collide;
    /****************************************************************************************/

}

/**
  * @brief Checks if cost represents a collision
  * @param cost Costmap cost
  * @return bool if in collision
  */
bool CostCritic::inCollision(float cost, float x, float y, float theta)
{
    bool is_tracking_unknown =
        costmap_ros_->getLayeredCostmap()->isTrackingUnknown();

    is_tracking_unknown = true;

    switch (static_cast<unsigned char>(cost)) {
        using namespace costmap_2d; // NOLINT
    case (LETHAL_OBSTACLE):
        return true;
    case (INSCRIBED_INFLATED_OBSTACLE):
        return consider_footprint_ ? false : true;
    case (NO_INFORMATION):
        return is_tracking_unknown ? false : true;
    }

    return false;
}

float CostCritic::costAtPose(float x, float y)
{
    using namespace costmap_2d;   // NOLINT
    unsigned int x_i, y_i;
    if (!costmap_->worldToMap(x, y, x_i, y_i)) {
        return costmap_2d::NO_INFORMATION;
    }

    return costmap_->getCost(x_i, y_i);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    mppi::critics::CostCritic,
    mppi::critics::CriticFunction)
