// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "gravity_torque/gravity_torque.hpp"

GravityTorque::GravityTorque(
  bool & success,
  const rclcpp::NodeOptions & options)
: Node("gravity_torque")
{
  // Declare parameters
  this->declare_parameter("motor_specs", "");
  this->declare_parameter("arm_group_name", "arm");
  this->declare_parameter("gripper_joint_name", "gripper");

  // Get parameters
  std::string motor_specs;
  this->get_parameter("motor_specs", motor_specs);
  this->get_parameter("arm_group_name", arm_group_name_);
  this->get_parameter("gripper_joint_name", gripper_joint_name_);

  // Create a reentrant callback group
  auto reentrant_callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  // Create the Subscription for the JointState message
  rclcpp::SubscriptionOptions joint_state_sub_options;
  joint_state_sub_options.callback_group = reentrant_callback_group;
  // When a message arrives on "joint_states", ROS calls the bound function and passes the message as the first argument
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&GravityTorque::joint_state_cb, this, std::placeholders::_1),
    joint_state_sub_options);


  // Main publisher that publishes Gravity-induced Torque readings
  gravity_torques_pub_ = this->create_publisher<gravity_torque::msg::GravityTorques>("gravity_torques", 10);

  // Create the client for the 'RobotInfo' service
  robot_info_client_ = this->create_client<interbotix_xs_msgs::srv::RobotInfo>(
    "get_robot_info", rmw_qos_profile_services_default, reentrant_callback_group);

  // Wait for the 'RobotInfo' service to be available
  while (!robot_info_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for %s. Exiting.",
        robot_info_client_->get_service_name());
      success = false;
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "%s not available, waiting again...",
      robot_info_client_->get_service_name());
  }

  // Get the joint names
  if (!get_joint_names()) {
    success = false;
    return;
  }

  // Load the motor specs
  if (!load_motor_specs(motor_specs)) {
    success = false;
    return;
  }

  // Prepare for the inverse dynamics solver
  if (!prepare_tree()) {
    success = false;
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Gravity Torque Publisher node is up"  // Check Subscriber-Publisher
  );
}

void GravityTorque::joint_state_cb(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // IMPORTANT
  // IMPLEMENTATION DETAIL :
  // ------------------------------------------------------ 
  // In ROS2 when using mutually exclusive callback groups,
  // The CALLBACK owns the Msg.
  // This means mutex locks are not required for safe access
  // For consistency, choose and follow through with one pattern
  // Either do not store the Msg
  // Or Store the Msg with a copied value

  // Store the latest joint state for the get_gravity_torques service
  joint_state_mutex_.lock();
  latest_joint_state_ = msg;
  joint_state_mutex_.unlock();

  // Create stuffs needing read/write access
  KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));
  KDL::JntArray q(tree_.getNrOfJoints());
  KDL::JntArray q_dot(tree_.getNrOfJoints());
  KDL::JntArray torques(tree_.getNrOfJoints());

  // Set the joint positions and velocities
  for (size_t i = 0; i < latest_joint_state_->position.size(); i++) {
    q(i) = latest_joint_state_->position[i];
    q_dot(i) = latest_joint_state_->velocity[i];
  }

  // Compute the torques
  idsolver.CartToJnt(q, q_dot, q_ddot_, f_ext_, torques);

  // Create a GravityTorques msg
  gravity_torque::msg::GravityTorques gravity_msg;
  gravity_msg.header.stamp = this->now();
  gravity_msg.header.frame_id = "base_link";
  gravity_msg.group_name = arm_group_name_;

  // Extract torques for arm joints only (in Nm, before conversion to current)
  gravity_msg.torques.resize(num_joints_arm_);
  gravity_msg.joint_positions.resize(num_joints_arm_);
  gravity_msg.joint_velocities.resize(num_joints_arm_);

  // get joint pos and vel
  for (size_t i = 0; i < num_joints_arm_; i++) {
    gravity_msg.joint_positions[i] = latest_joint_state_->position[i];
    gravity_msg.joint_velocities[i] = latest_joint_state_->velocity[i];
  }

  // Get gravity torques
  for (size_t i = 0; i < num_joints_arm_; i++) {
    gravity_msg.torques[i] = static_cast<double>(torques(i));  // save as Nm
  }
  
  // Extra Notes:
  // ----------------------------------------------------
  // More options available for kinematic, Static frictions from the original script.
  // Add to the ROS message if required.

  // Publish the Gravity Torques message
  // no race condition
  gravity_torques_pub_->publish(gravity_msg);

}

bool GravityTorque::load_motor_specs(const std::string & motor_specs)
{
  // Load the torque constants and current units from the motor_specs file
  YAML::Node motor_specs_node;
  try {
    motor_specs_node = YAML::LoadFile(motor_specs);
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load the motor specs file '%s'! Error: %s",
      motor_specs.c_str(), e.what());
    return false;
  }

  // Warn if dither is enabled
  bool dither = motor_specs_node["dither"].as<bool>(false);
  if (dither) {
    RCLCPP_WARN(
      this->get_logger(),
      "Dither is enabled. Excessive dithering WILL cause heat and wear on the joints.");
  }

  // Load the torque constants, current units, and no load currents
  float all = motor_specs_node["motor_assist"]["all"].as<float>(-1.0);
  float single = 0.0;
  for (const auto & joint_name : joint_names_) {
    if (motor_specs_node["motor_assist"][joint_name].IsDefined()) {
      // Load the motor specs
      torque_constants_.push_back(
        motor_specs_node["motor_specs"][joint_name]["torque_constant"].as<float>()
      );
      current_units_.push_back(
        motor_specs_node["motor_specs"][joint_name]["current_unit"].as<float>()
      );
      no_load_currents_.push_back(
        motor_specs_node["motor_specs"][joint_name]["no_load_current"].as<float>()
      );
      kinetic_friction_coefficients_.push_back(
        motor_specs_node["motor_specs"][joint_name]["kinetic_friction_coefficient"].as<float>()
      );
      static_friction_coefficients_.push_back(
        motor_specs_node["motor_specs"][joint_name]["static_friction_coefficient"].as<float>()
      );
      dither_speeds_.push_back(
        motor_specs_node["motor_specs"][joint_name]["dither_speed"].as<float>()
      );

      // Enable/disable the dither
      if (!dither) {
        kinetic_friction_coefficients_.back() = static_friction_coefficients_.back();
        static_friction_coefficients_.back() = 0;
        dither_speeds_.back() = 0;
      }

      // Scale the no load current according to the motor assist setting
      if (all == -1.0) {
        single = motor_specs_node["motor_assist"][joint_name].as<float>(0.5);
        if (0 <= single && single <= 1) {
          no_load_currents_.back() *= single;
          kinetic_friction_coefficients_.back() *= single;
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Motor assist value not in the range [0, 1] for joint %s. Setting it to 0.",
            joint_name.c_str());
          no_load_currents_.back() = 0;
          kinetic_friction_coefficients_.back() = 0;
        }
      } else if (0 <= all && all <= 1) {
        no_load_currents_.back() *= all;
        kinetic_friction_coefficients_.back() *= all;
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Motor assist value not in the range [0, 1] or -1 for all joints. Setting it to 0.");
        no_load_currents_.back() = 0;
        kinetic_friction_coefficients_.back() = 0;
      }

    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Motor specs not found for joint %s in '%s', "
        "assuming it does not support current control. "
        "Setting all motor specs to -1. "
        "Its torque will be disabled when the gravity compensation is enabled.",
        joint_name.c_str(), motor_specs.c_str());
      torque_constants_.push_back(-1);
      current_units_.push_back(-1);
      no_load_currents_.push_back(-1);
      kinetic_friction_coefficients_.push_back(-1);
      static_friction_coefficients_.push_back(-1);
      dither_speeds_.push_back(-1);
    }
  }

  return true;
}

bool GravityTorque::get_joint_names()
{
  // Create a request message for the 'RobotInfo' service
  auto request = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  request->cmd_type = "group";
  request->name = "all";

  // Call the 'RobotInfo' service
  auto future = robot_info_client_->async_send_request(request);

  // Wait for the future to be ready
  // Use spin_until_future_complete because it's not called in a callback
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to call the %s service",
      robot_info_client_->get_service_name());
    return false;
  }

  // Get the response message
  auto response = future.get();

  // Get the joint names
  joint_names_ = response->joint_names;

  // Replace "left_finger" with "gripper"
  // Please refer to the note in the 'RobotInfo' service documentation for why
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == "left_finger") {
      joint_names_[i] = gripper_joint_name_;
    }
  }

  // Get the number of joints in the 'arm' group: all joints except the gripper joint
  num_joints_arm_ = joint_names_.size() - 1;

  return true;
}


bool GravityTorque::prepare_tree()
{
  // Create the client to get the robot description string
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    this, "robot_state_publisher"
  );

  // Wait for the robot description string to be available
  while (!param_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for robot_state_publisher. Exiting.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "robot_state_publisher not available, waiting again...");
  }

  // Get the robot description string
  std::string robot_desc_string = param_client->get_parameter<std::string>("robot_description");

  // Parse the robot description string to get the KDL tree
  if (!kdl_parser::treeFromString(robot_desc_string, tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse the robot description string!");
    return false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully parsed the robot description string!");
  }

  // Resize the read-only joint arrays
  q_ddot_.resize(tree_.getNrOfJoints());

  return true;
}
