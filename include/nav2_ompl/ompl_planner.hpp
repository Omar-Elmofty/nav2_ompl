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

#pragma once

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_ompl {

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief Global planner plugin that uses OMPL for plan generation.
 */
class OMPLPlanner : public nav2_core::GlobalPlanner {
 public:
  /**
   * @brief constructor
   */
  OMPLPlanner();

  /**
   * @brief destructor
   */
  ~OMPLPlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal) override;

 private:
  //! Pointer to the simple setup object.
  std::unique_ptr<og::SimpleSetup> simple_setup_;

  //! Clock.
  rclcpp::Clock::SharedPtr clock_;

  //! Logger.
  rclcpp::Logger logger_{rclcpp::get_logger("OMPLPlanner")};

  //! The name of this node.
  std::string name_;

  //! Name of the global frame.
  std::string global_frame_;

  //! Pointer to the dubins state space.
  std::shared_ptr<ob::DubinsStateSpace> state_space_;

  //! Pointer to the planner.
  std::shared_ptr<og::BITstar> planner_;

  //! The dubins radius to use [m].
  double dubins_radius_{0.25};

  //! Time to spend on planning [s].
  double planning_time_{1.0};

  //! Time to spend on simplifying the solution [s].
  double simplify_time_{1.0};

  //! X-bound [m].
  double x_bound_{12.5};

  //! Y-bound [m].
  double y_bound_{7.5};
};

}  // namespace nav2_ompl
