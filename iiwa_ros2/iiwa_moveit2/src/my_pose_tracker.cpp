// Copyright 2022, ICube Laboratory, University of Strasbourg
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
//
// Adapted from https://github.com/ros-planning/moveit2/blob/foxy/moveit_ros/moveit_servo/src/cpp_interface_demo/pose_tracking_demo.cpp

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_tracking_servo_node");

class PoseTracker
{
public:
  PoseTracker(const rclcpp::Node::SharedPtr& node, const std::string& command_topic, const std::string& target_topic)
  {
    node_ = node;
    sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(command_topic, 1, std::bind(&PoseTracker::new_target_pose_callback, this, std::placeholders::_1));
    target_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(target_topic, 1 /* queue */);
    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
    if (servo_parameters == nullptr)
    {
      RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
      exit(EXIT_FAILURE);
    }
    
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }

    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // Wait for Planning Scene Monitor to setup
    if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
      exit(EXIT_FAILURE);
    }
    tracker = new moveit_servo::PoseTracking(node_, servo_parameters, planning_scene_monitor);

  }


private:

  void new_target_pose_callback(geometry_msgs::msg::PoseStamped msg)
  {

      msg.header.stamp = node_->now();
      std::cout<<"PRINT HERE"<<std::endl;
      tracker->resetTargetPose();

      std::cout<<"PRINT HERE2"<<std::endl;
      target_pose_pub->publish(msg);
      std::atomic_bool done;
      done = false;
      std::thread move_to_pose_thread([this, &done] {
        moveit_servo::PoseTrackingStatusCode tracking_status =
            tracker->moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
            done = true;
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                      << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
      });
    
    std::cout<<"PRINT HERe3"<<std::endl;
    rclcpp::Rate loop_rate(1000);
    while (done == false)
    {
      // Modify the pose target a little bit each cycle
      // This is a dynamic pose target
       std::cout<<"PRINT HERe5"<<std::endl;
      msg.header.stamp = node_->now();
      target_pose_pub->publish(msg);
      loop_rate.sleep();
    }


      std::cout<<"PRINT HERe4"<<std::endl;

      



  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub;
  moveit_servo::ServoParameters::SharedConstPtr servo_parameters;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  moveit_servo::PoseTracking* tracker;
  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  double rot_tol = 0.01;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_tracking_servo_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  PoseTracker pose_tracker = PoseTracker(node, "cmd_pose", "target_pose");
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
