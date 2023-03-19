/*
Copyright (c) 2023 Omar Elmofty

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "nav2_ompl/ompl_planner.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav2_util/node_utils.hpp>

namespace nav2_ompl {
OMPLPlanner::OMPLPlanner() {}

OMPLPlanner::~OMPLPlanner() {}

void OMPLPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  std::ignore = tf;  // We won't be using tf

  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  name_ = name;
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(logger_, "Configuring plugin %s of type OMPLPlanner",
              name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  nav2_util::declare_parameter_if_not_declared(node, name + ".dubins_radius",
                                               rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".dubins_radius", dubins_radius_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".planning_time",
                                               rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".planning_time", planning_time_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".simplify_time",
                                               rclcpp::ParameterValue(1.0));
  node->get_parameter(name + ".simplify_time", simplify_time_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".x_bound",
                                               rclcpp::ParameterValue(12.5));
  node->get_parameter(name + ".x_bound", x_bound_);
  nav2_util::declare_parameter_if_not_declared(node, name + ".y_bound",
                                               rclcpp::ParameterValue(7.5));
  node->get_parameter(name + ".y_bound", y_bound_);

  // Create dubins state space with turning radius of 1.0 m
  state_space_ = std::make_shared<ob::DubinsStateSpace>(dubins_radius_);

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, -x_bound_);
  bounds.setHigh(0, x_bound_);
  bounds.setLow(1, -y_bound_);
  bounds.setHigh(1, y_bound_);

  state_space_->setBounds(bounds);

  // Create the simple setup object.
  simple_setup_ = std::make_unique<og::SimpleSetup>(state_space_);

  // We now set the state validity checker, which is the function that will be
  // used to check if the robots footprint is in collision or not.
  simple_setup_->setStateValidityChecker(
      [costmap_2d = costmap_ros->getCostmap()](const ob::State *state) -> bool {
        // First convert the state to the desired type
        const auto se2_state = state->as<ob::DubinsStateSpace::StateType>();
        // Get the costmap coordinates at this state.
        unsigned int mx{0};
        unsigned int my{0};
        if (!costmap_2d->worldToMap(se2_state->getX(), se2_state->getY(), mx,
                                    my)) {
          return false;
        }
        // Check if state is collision free.
        return costmap_2d->getCost(mx, my) == nav2_costmap_2d::FREE_SPACE;
      });

  // Now setup planner
  planner_ =
      std::make_shared<og::BITstar>(simple_setup_->getSpaceInformation());
  simple_setup_->setPlanner(planner_);
}

void OMPLPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type OMPLPlanner",
              name_.c_str());
  simple_setup_.reset();
  planner_.reset();
  state_space_.reset();
}

void OMPLPlanner::activate() {
  RCLCPP_INFO(logger_, "Activating plugin %s of type OMPLPlanner",
              name_.c_str());
}

void OMPLPlanner::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type OMPLPlanner",
              name_.c_str());
}

nav_msgs::msg::Path OMPLPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) {
  // Clear stale data from previous plan
  simple_setup_->clear();

  // Create start scoped state.
  auto si = simple_setup_->getSpaceInformation();
  auto start_scoped = ob::ScopedState<ob::DubinsStateSpace>(si);
  start_scoped->setX(start.pose.position.x);
  start_scoped->setY(start.pose.position.y);
  tf2::Quaternion q_start;
  tf2::convert(start.pose.orientation, q_start);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_start).getRPY(roll, pitch, yaw);
  start_scoped->setYaw(yaw);

  // Create goal scoped state.
  auto goal_scoped = ob::ScopedState<ob::DubinsStateSpace>(si);
  goal_scoped->setX(goal.pose.position.x);
  goal_scoped->setY(goal.pose.position.y);
  tf2::Quaternion q_goal;
  tf2::convert(goal.pose.orientation, q_goal);
  tf2::Matrix3x3(q_goal).getRPY(roll, pitch, yaw);
  goal_scoped->setYaw(yaw);

  simple_setup_->setStartAndGoalStates(start_scoped, goal_scoped);

  // Create plan
  const auto status = simple_setup_->solve(planning_time_);
  if (status != ob::PlannerStatus::EXACT_SOLUTION) {
    RCLCPP_WARN(logger_, "%s: failed to create plan.", name_.c_str());
    return nav_msgs::msg::Path();
  }
  simple_setup_->simplifySolution(simplify_time_);

  // Create path message
  auto solution_path = simple_setup_->getSolutionPath();
  solution_path.interpolate();

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  for (const auto &state : solution_path.getStates()) {
    const auto se2_state = state->as<ob::DubinsStateSpace::StateType>();
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = se2_state->getX();
    pose.pose.position.y = se2_state->getY();
    tf2::Quaternion q_pose;
    q_pose.setRPY(0, 0, se2_state->getYaw());
    q_pose.normalize();
    pose.pose.orientation.x = q_pose.getX();
    pose.pose.orientation.y = q_pose.getY();
    pose.pose.orientation.z = q_pose.getZ();
    pose.pose.orientation.w = q_pose.getW();

    path.poses.push_back(pose);
  }

  return path;
}
}  // namespace nav2_ompl

// Register plugin
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nav2_ompl::OMPLPlanner, nav2_core::GlobalPlanner)
