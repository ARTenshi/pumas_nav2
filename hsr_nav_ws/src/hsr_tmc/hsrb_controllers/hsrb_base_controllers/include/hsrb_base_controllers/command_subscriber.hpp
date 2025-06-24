/*
Copyright (c) 2019 TOYOTA MOTOR CORPORATION
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
/// @file command_subscriber.hpp
/// @brief Input command control class for controlling all -level bogie control
#ifndef HSRB_BASE_CONTROLLERS_COMMAND_SUBSCRIBER_HPP_
#define HSRB_BASE_CONTROLLERS_COMMAND_SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <hsrb_base_controllers/controller_command_interface.hpp>
#include <hsrb_base_controllers/omni_base_state.hpp>

namespace hsrb_base_controllers {

/// Input command class
class CommandSubscriber : private boost::noncopyable {
 public:
  CommandSubscriber(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                    IControllerCommandInterface* controller);
  virtual ~CommandSubscriber() {}

 protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  IControllerCommandInterface* controller_;
};

/// Input speed command class
class CommandVelocitySubscriber : public CommandSubscriber {
 public:
  using Ptr = std::shared_ptr<CommandVelocitySubscriber>;

  CommandVelocitySubscriber(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                            IControllerCommandInterface* controller);
  virtual ~CommandVelocitySubscriber() {}

 private:
  // Input command speed callback
  void CommandVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Input speed sub slider
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
};

/// Input orbital command class
class CommandTrajectorySubscriber : public CommandSubscriber {
 public:
  using Ptr = std::shared_ptr<CommandTrajectorySubscriber>;

  CommandTrajectorySubscriber(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                              IControllerCommandInterface* controller);
  virtual ~CommandTrajectorySubscriber() {}

 private:
  // Input command trajectory callback
  void CommandTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  // Input orbital sub -scriver
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
};


/// Input orbital action command class
class TrajectoryActionServer : public CommandSubscriber {
 public:
  using Ptr = std::shared_ptr<TrajectoryActionServer>;

  TrajectoryActionServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                         const std::vector<std::string>& cordinates,
                         IControllerCommandInterface* controller);
  virtual ~TrajectoryActionServer() {}

  // Update the results of action
  void UpdateActionResult(int32_t error_code);
  // Issuance of feedback
  void SetFeedback(const ControllerBaseState& state, const rclcpp::Time& stamp);
  // Clear the goal currently following
  void PreemptActiveGoal();

 private:
  // Action status update cycle
  double action_monitor_period_;
  // Bogie joint name
  std::vector<std::string> cordinates_;
  // Goalhandle for action
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> goal_handle_buffer_;

  // Action server and callback function group
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
  using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_action::GoalResponse GoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse CancelCallback(const ServerGoalHandlePtr goal_handle);
  void FeedbackSetupCallback(ServerGoalHandlePtr goal_handle);

  // Timer for action execution
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
};

}  // namespace hsrb_base_controllers

#endif /*HSRB_BASE_CONTROLLERS_COMMAND_SUBSCRIBER_HPP_*/
