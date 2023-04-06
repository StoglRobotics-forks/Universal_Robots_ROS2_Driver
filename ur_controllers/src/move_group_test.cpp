/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "yaml-cpp/yaml.h"
#include <filesystem>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");


void wrtieTrajectoryMsgToYaml(const moveit_msgs::msg::RobotTrajectory &traj,
                                YAML::Node &out_node) {
  YAML::Node trajectory_node;
  YAML::Node header_node;

  // Set header of the joint_trajectory msgs
  YAML::Node stamp_node;
  stamp_node["secs"] = traj.joint_trajectory.header.stamp.sec;
  stamp_node["nanosec"] = traj.joint_trajectory.header.stamp.nanosec;
  header_node["stamp"] = stamp_node;
  header_node["frame_id"] = traj.joint_trajectory.header.frame_id;
  trajectory_node["header"] = header_node;

  // Set the joint names
  YAML::Node joint_names_node;
  for (size_t i = 0; i < traj.joint_trajectory.joint_names.size(); i++) {
    joint_names_node.push_back(traj.joint_trajectory.joint_names[i]);
  }
  trajectory_node["joint_names"] = joint_names_node;

  // Set the list of joint trajectory points
  YAML::Node points_node;
  for (size_t i = 0; i < traj.joint_trajectory.points.size(); i++) {
    YAML::Node point_node;
    point_node["positions"] = traj.joint_trajectory.points[i].positions;
    point_node["velocities"] = traj.joint_trajectory.points[i].velocities;
    point_node["accelerations"] = traj.joint_trajectory.points[i].accelerations;
    point_node["effort"] = traj.joint_trajectory.points[i].effort;
    YAML::Node tfs_node;
    tfs_node["secs"] = traj.joint_trajectory.points[i].time_from_start.sec;
    tfs_node["nanosec"] = traj.joint_trajectory.points[i].time_from_start.nanosec;
    point_node["time_from_start"] = tfs_node;
    points_node[std::to_string(i)] = point_node;
  }
  trajectory_node["points"] = points_node;

  // Set the trajectory to the output yaml node
  out_node["trajectory"] = trajectory_node;
}

void writeTrajectoy(const moveit_msgs::msg::RobotTrajectory &traj, std::filesystem::path &save_path){
  std::ofstream sav;
  sav.open(save_path.c_str());
  YAML::Emitter out;
  YAML::Node node;

  wrtieTrajectoryMsgToYaml(traj, node);
  out << node;
  sav << out.c_str();
  sav.close();
}

moveit_msgs::msg::RobotTrajectory getTraj(std::filesystem::path &traj_path) {

  YAML::Node file_node = YAML::LoadFile(traj_path);
  moveit_msgs::msg::RobotTrajectory traj_msg;
  auto traj_node = file_node["trajectory"].as<YAML::Node>();

  traj_msg.joint_trajectory.header.frame_id =
      traj_node["header"]["frame_id"].as<std::string>();
  traj_msg.joint_trajectory.header.stamp.sec =
      traj_node["header"]["stamp"]["secs"].as<uint32_t>();
  traj_msg.joint_trajectory.header.stamp.nanosec =
      traj_node["header"]["stamp"]["nanosec"].as<uint32_t>();


    for (size_t i = 0; i < traj_node["joint_names"].size(); i++)
    {
        /* code */
        traj_msg.joint_trajectory.joint_names.push_back(traj_node["joint_names"][i].as<std::string>());
    }

  for (size_t i = 0; i < traj_node["points"].size(); i++) {
    auto point_node = traj_node["points"][std::to_string(i)].as<YAML::Node>();
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = point_node["positions"].as<std::vector<double>>();
    point.velocities = point_node["velocities"].as<std::vector<double>>();
    point.accelerations = point_node["accelerations"].as<std::vector<double>>();
    point.effort = point_node["effort"].as<std::vector<double>>();
    point.time_from_start.sec =
        point_node["time_from_start"].as<YAML::Node>()["secs"].as<int32_t>();
    point.time_from_start.nanosec =
        point_node["time_from_start"].as<YAML::Node>()["nanosec"].as<int32_t>();
    traj_msg.joint_trajectory.points.push_back(point);
  }

  return traj_msg;
}

int main(int argc, char** argv)
{
  std::cout << "Starting the node...." << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_test", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "ur_manipulator";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose target_pose1 = current_pose.pose;
  target_pose1.position.z -= 0.1;
  move_group.setPoseTarget(target_pose1);


  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Adding a obtject in the planning scene
  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.075;  // x
  primitive.dimensions[1] = 1.075;  // y
  primitive.dimensions[2] = 0.005;  // z
  
  moveit_msgs::msg::CollisionObject collision_object;
  
  // Make the object moving with the robot 
  collision_object.header.frame_id = "base_link";
  /* The id of the object */
  collision_object.id = "Ceiling";
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Define a pose related to the given frame
  geometry_msgs::msg::Pose pose;
  pose.position.z = 1.11;
  pose.orientation.w = 1.0;

  // Assgin the pose and the shape to the collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);

  // Update the planning scene
  planning_scene_interface.applyCollisionObject(collision_object);


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Get the trajectory - moveit_msgs/RobotTrajectory.msg
  auto traj = plan.trajectory_;

  // Define place to save trajectory
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("ur_moveit_config");
  std::cout << package_share_directory.c_str() << std::endl;
  std::filesystem::path pkg_path(package_share_directory);
  std::filesystem::path file_path = pkg_path / "trajectory.yaml";

  writeTrajectoy(traj, file_path);
  
  std::cout << "Sleeping for 3 seconds..." << std::endl;
  std::cout << "3" << std::endl;
  rclcpp::Rate rate(1.0);
  rate.sleep();
  std::cout << "2" << std::endl;
  rate.sleep();
  std::cout << "1" << std::endl;
  rate.sleep();

  std::cout << "I am done with sleeping, I will now read the trajectory" << std::endl;

  // Set the trajectory to be empty
  auto read_traj = getTraj(file_path);

  // Execute the plan
  // move_group.execute(new_plan);
  move_group.execute(read_traj);


  // Moving to a pose goal
  // move_group.move();

  // 

  rclcpp::shutdown();
  return 0;
}
