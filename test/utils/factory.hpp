// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "mppi_local_planner/motion_models.hpp"
#include "mppi_local_planner/optimizer.hpp"
#include "mppi_local_planner/mppi_local_planner_ros.hpp"

#include "models.hpp"

namespace detail
{

template<typename TMessage>
void setHeader(TMessage && msg, std::string frame)
{
  //auto time = node->get_clock()->now();
  msg.header.frame_id = frame;
  //msg.header.stamp = time;
  msg.header.stamp = ros::Time::now();
}

}  // namespace detail


/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
 */
//void setUpOptimizerParams(
//  const TestOptimizerSettings & s,
//  const std::vector<std::string> & critics)
//{
//  constexpr double dummy_freq = 50.0;
//  s.iteration_count = 1;
//  s.batch_size = 1800;
//  s.time_steps = 36;
//  s.lookahead_distance = 1.5;
//  s.motion_model = "DiffDrive"
//  critics ={"PathAlignLegacyCritic", "PathFollowCritic", "PreferForwardCritic", "GoalCritic",
//         "GoalAngleCritic", "PathAngleCritic", "TwirlingCritic", "CostCritic", "ConstraintCritic",
//         "VelocityCritic"};
//  dummy_freq = 10.0;
//}

void setUpControllerParams(
  bool visualize, std::string node_name = std::string("dummy"))
{
  double dummy_freq = 50.0;
  visualize = true;
  dummy_freq = 10.0;
}

//rclcpp::NodeOptions getOptimizerOptions(
//  TestOptimizerSettings s,
//  const std::vector<std::string> & critics)
//{
//  std::vector<rclcpp::Parameter> params;
//  rclcpp::NodeOptions options;
//  setUpOptimizerParams(s, critics, params);
//  options.parameter_overrides(params);
//  return options;
//}

geometry_msgs::Point getDummyPoint(double x, double y)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = 0;

  return point;
}

std::shared_ptr<costmap_2d::Costmap2DROS> getDummyCostmapRos()
{
  auto costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>("cost_map_node");
  //costmap_ros->on_configure(rclcpp_lifecycle::State{});
  return costmap_ros;
}

std::shared_ptr<costmap_2d::Costmap2D> getDummyCostmap(TestCostmapSettings s)
{
  auto costmap = std::make_shared<costmap_2d::Costmap2D>(
    s.cells_x, s.cells_y, s.resolution, s.origin_x, s.origin_y, s.cost_map_default_value);

  return costmap;
}

std::vector<geometry_msgs::Point> getDummySquareFootprint(double a)
{
  return {getDummyPoint(a, a), getDummyPoint(-a, -a), getDummyPoint(a, -a), getDummyPoint(-a, a)};
}

std::shared_ptr<costmap_2d::Costmap2DROS> getDummyCostmapRos(TestCostmapSettings s)
{
  auto costmap_ros = getDummyCostmapRos();
  auto costmap_ptr = costmap_ros->getCostmap();
  auto costmap = getDummyCostmap(s);
  *(costmap_ptr) = *costmap;


  //costmap_ros->setRobotFootprint(getDummySquareFootprint(s.footprint_size));

  return costmap_ros;
}

//std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
//getDummyNode(
//  TestOptimizerSettings s, std::vector<std::string> critics,
//  std::string node_name = std::string("dummy"))
//{
//  auto node =
//    std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, getOptimizerOptions(s, critics));
//  return node;
//}

//std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
//getDummyNode(rclcpp::NodeOptions options, std::string node_name = std::string("dummy"))
//{
//  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
//  return node;
//}

template<typename TCostMap, typename TConfig>
std::shared_ptr<mppi::Optimizer> getDummyOptimizer(TCostMap costmap_ros, TConfig cfg)
{
  std::shared_ptr<mppi::Optimizer> optimizer = std::make_shared<mppi::Optimizer>();
  ros::NodeHandle nh;
  std::string name = "test_optimizer";

  optimizer->initialize(
      &nh, name, costmap_ros, cfg);

  return optimizer;
}

// TODO
template<typename TCostMap, typename TFBuffer>
mppi::PathHandler getDummyPathHandler(TCostMap costmap_ros, TFBuffer tf_buffer)
{
  mppi::PathHandler path_handler;

  ros::NodeHandle nh;
  std::string name = "test_path_handler";

  path_handler.initialize(&nh, name, costmap_ros, tf_buffer);

  return path_handler;
}

// TODO
template<typename TCostMap, typename TFBuffer>
std::shared_ptr<mppi::MPPILocalPlannerROS> getDummyController(TFBuffer tf_buffer,
  TCostMap costmap_ros)
{
  auto controller = std::make_shared<mppi::MPPILocalPlannerROS>();

  std::string name = "test_controller";

  controller->initialize(name, tf_buffer, costmap_ros);

  return controller;
}

auto getDummyTwist()
{
  geometry_msgs::Twist twist;
  return twist;
}

geometry_msgs::PoseStamped
getDummyPointStamped(std::string frame = std::string("odom"))
{
  geometry_msgs::PoseStamped point;
  detail::setHeader(point, frame);

  return point;
}

geometry_msgs::PoseStamped getDummyPointStamped(TestPose pose)
{
  geometry_msgs::PoseStamped point = getDummyPointStamped();
  point.pose.position.x = pose.x;
  point.pose.position.y = pose.y;

  return point;
}

nav_msgs::Path getDummyPath(std::string frame = std::string("odom"))
{
  nav_msgs::Path path;
  detail::setHeader(path, frame);
  return path;
}

auto getDummyPath(size_t points_count)
{
  auto path = getDummyPath();

  for (size_t i = 0; i < points_count; i++) {
    path.poses.push_back(getDummyPointStamped());
  }

  return path;
}

nav_msgs::Path getIncrementalDummyPath(TestPathSettings s)
{
  auto path = getDummyPath();

  for (size_t i = 0; i < s.poses_count; i++) {
    double x = s.start_pose.x + static_cast<double>(i) * s.step_x;
    double y = s.start_pose.y + static_cast<double>(i) * s.step_y;
    path.poses.push_back(getDummyPointStamped(TestPose{x, y}));
  }

  return path;
}
