#include <serl_franka_controllers/real_time_joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace serl_franka_controllers {

bool RealTimeJointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "RealTimeJointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("RealTimeJointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("RealTimeJointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "RealTimeJointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, 0, 0, -1.57, -1.57, 3.14, 0}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "RealTimeJointPositionController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  // Initialize the subscriber
  joint_state_subscriber_ = node_handle.subscribe("/panda_commands", 1, &RealTimeJointPositionController::jointStateCallback, this);

  return true;
}

void RealTimeJointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void RealTimeJointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // Check if we have received joint states
  if (current_joint_values_.size() == 7) {
    for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(current_joint_values_[i]);
    }
  }
  else {
    ROS_ERROR("RealTimeJointPositionController: Did not receive joint states");
  }
}

void RealTimeJointPositionController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (msg->position.size() == 7) {
    current_joint_values_ = msg->position;
  } else {
    ROS_ERROR("RealTimeJointPositionController: Received joint states with incorrect number of joints");
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::RealTimeJointPositionController,
                       controller_interface::ControllerBase)
