/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include "hsrb_gripper_controller/hrh_gripper_follow_trajectory_action.hpp"
#include <limits>
#include <hsrb_servomotor_protocol/exxx_common.hpp>
#include "hsrb_gripper_controller/hrh_gripper_controller.hpp"

namespace {

// Default position goal tolerance error [RAD]
const double kDefaultPositionGoalTolerance = 0.05;
// Default goal reach accepted time [S]
const double kDefaultPositionGoalTimeTolerance = 0.05;

bool ValidateTrajectory(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                        const trajectory_msgs::msg::JointTrajectory& trajectory, const std::string& joint_name) {
  // Grippers are assumed 1 -axis
  if (trajectory.joint_names.size() != 1) {
    RCLCPP_ERROR(node->get_logger(), "Can't accept new action goals. joint_names' size is invalid.");
    return false;
  }

  if (trajectory.joint_names.at(0) != joint_name) {
    RCLCPP_ERROR(node->get_logger(), "Can't accept new action goals. joint_names[0]=%s is invalid.",
                 trajectory.joint_names.at(0).c_str());
    return false;
  }

  if (trajectory.points.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory is empty.");
    return false;
  }

  double last_time = -std::numeric_limits<double>::max();
  for (const auto& point : trajectory.points) {
    if (point.positions.size() != 1) {
      RCLCPP_ERROR(node->get_logger(), "Can't accept new action goals. positions' size is invalid.");
      return false;
    }
    double time_from_start = rclcpp::Duration(point.time_from_start).seconds();
    if (time_from_start - last_time <= 0.0) {
      RCLCPP_ERROR(node->get_logger(), "Trajectory's time_from_start is going reverse.");
      return false;
    }
    last_time = time_from_start;
  }

  if (rclcpp::Time(trajectory.header.stamp) == rclcpp::Time(0, 0, RCL_ROS_TIME) && last_time > 0.0) {
    return true;
  }
  if (rclcpp::Time(trajectory.header.stamp) + rclcpp::Duration::from_seconds(last_time) < node->now()) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory is past. Ignoring the goal...");
    return false;
  } else {
    return true;
  }
}

}  // unnamed namespace

namespace hsrb_gripper_controller {

HrhGripperFollowTrajectoryAction::HrhGripperFollowTrajectoryAction(HrhGripperController* controller)
    : HrhGripperAction(controller, "~/follow_joint_trajectory", hsrb_servomotor_protocol::kDriveModeHandPosition),
      default_goal_tolerance_(kDefaultPositionGoalTolerance),
      default_goal_time_tolerance_(kDefaultPositionGoalTimeTolerance) {}

bool HrhGripperFollowTrajectoryAction::Activate() {
  last_command_state_.positions = { controller_->GetCurrentPosition() };
  last_command_state_.velocities = { controller_->GetCurrentVelocity() };
  return true;
}

void HrhGripperFollowTrajectoryAction::Update(const rclcpp::Time& time) {
  // Check if there is orbit, update should only be called in orbit follow -up mode, so it may not be necessary.
  auto current_msg = trajectory_ptr_->get_trajectory_msg();
  auto new_msg = trajectory_msg_buffer_.readFromRT();
  if (current_msg != *new_msg) {
    trajectory_ptr_->update(*new_msg);
  }
  if (!trajectory_active_ptr_ || !(*trajectory_active_ptr_)->has_trajectory_msg() ||
      (*trajectory_active_ptr_)->get_trajectory_msg()->points.empty()) {
    return;
  }

  // Target position sample
  if (!(*trajectory_active_ptr_)->is_sampled_already()) {
    if (open_loop_control_) {
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, last_command_state_);
    } else {
      trajectory_msgs::msg::JointTrajectoryPoint current_state;
      current_state.positions = { controller_->GetCurrentPosition() };
      current_state.velocities = { controller_->GetCurrentVelocity() };
      (*trajectory_active_ptr_)->set_point_before_trajectory_msg(time, current_state);
    }
  }
  trajectory_msgs::msg::JointTrajectoryPoint desired_state;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator start_segment_it;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator end_segment_it;
  (*trajectory_active_ptr_)->sample(time, joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,
                                    desired_state, start_segment_it, end_segment_it);
  last_command_state_ = desired_state;

  controller_->SetComandPosition(desired_state.positions[0]);

  // Judgment and failure, check only GOAL TOLERANCE in the gripper
  const auto active_goal = *goal_handle_buffer_.readFromNonRT();
  if (!active_goal) {
    return;
  }

  auto goal_condition = *(goal_condition_buffer_.readFromRT());
  if (time < goal_condition.expected_arrival_time) {
    return;
  }

  if (std::abs(controller_->GetCurrentPosition() - goal_condition.position) < goal_condition.goal_tolerance) {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL);
    active_goal->setSucceeded(result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  } else if (time >= goal_condition.abort_time) {
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_code(control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED);
    active_goal->setAborted(result);
    goal_handle_buffer_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

void HrhGripperFollowTrajectoryAction::PreemptActiveGoal() {
  HrhGripperAction::PreemptActiveGoal();
  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>());
}

/// Implementation of initialization of action
bool HrhGripperFollowTrajectoryAction::InitImpl(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
  default_goal_tolerance_ = GetPositiveParameter(node, "position_goal_tolerance", kDefaultPositionGoalTolerance);
  default_goal_time_tolerance_ =
      GetNonNegativeParameter(node, "position_goal_time_tolerance", kDefaultPositionGoalTimeTolerance);
  open_loop_control_ = GetParameter(node, "open_loop_control", false);

  goal_condition_buffer_.initRT(GoalCondition());

  trajectory_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  trajectory_active_ptr_ = &trajectory_ptr_;
  trajectory_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  trajectory_command_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", 1,
      std::bind(&HrhGripperFollowTrajectoryAction::TrajectoryCommandCallback, this, std::placeholders::_1));
  return true;
}

/// Check if the goal is acceptable
bool HrhGripperFollowTrajectoryAction::ValidateGoal(const control_msgs::action::FollowJointTrajectory::Goal& goal) {
  return ValidateTrajectory(node_, goal.trajectory, controller_->joint_name());
}

/// Update action goals
void HrhGripperFollowTrajectoryAction::UpdateActionImpl(const control_msgs::action::FollowJointTrajectory::Goal& goal) {
  // PATH_TOLERANCE is not supported
  if (!goal.path_tolerance.empty()) {
    RCLCPP_WARN(node_->get_logger(), "path_tolerance is not supported. Ignoring the parameter...");
  }

  GoalCondition goal_condition;
  goal_condition.position = goal.trajectory.points.back().positions[0];

  auto start_time = rclcpp::Time(goal.trajectory.header.stamp);
  if (start_time == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
    start_time = node_->now();
  }
  goal_condition.expected_arrival_time = start_time + rclcpp::Duration(goal.trajectory.points.back().time_from_start);

  rclcpp::Duration goal_time_tolerance(0, 0);
  if (goal.goal_tolerance.size() == 1 && goal.goal_tolerance[0].name == controller_->joint_name()) {
    goal_condition.goal_tolerance = goal.goal_tolerance[0].position;
    goal_time_tolerance = goal.goal_time_tolerance;
  } else {
    goal_condition.goal_tolerance = default_goal_tolerance_;
    goal_time_tolerance = rclcpp::Duration::from_seconds(default_goal_time_tolerance_);
  }
  if (goal_time_tolerance == rclcpp::Duration(0, 0)) {
    goal_condition.abort_time = rclcpp::Time(std::numeric_limits<int64_t>::max());
  } else {
    goal_condition.abort_time = goal_condition.expected_arrival_time + goal_time_tolerance;
  }
  goal_condition_buffer_.writeFromNonRT(goal_condition);

  trajectory_msg_buffer_.writeFromNonRT(std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal.trajectory));
}

void HrhGripperFollowTrajectoryAction::TrajectoryCommandCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  if (ValidateTrajectory(node_, *msg, controller_->joint_name())) {
    controller_->PreemptActiveGoal();
    controller_->ChangeControlMode(shared_from_this());
    trajectory_msg_buffer_.writeFromNonRT(msg);
  }
}

}  // namespace hsrb_gripper_controller
