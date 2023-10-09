/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Groundtruth Plugin
 *
 * This plugin gets and publishes ground-truth data
 *
 * @author Abhishek Goudar <deloney21@gmail.com>
 */

#include <gazebo_groundtruth_plugin_ros2.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GroundtruthPlugin)

GroundtruthPlugin::GroundtruthPlugin() : ModelPlugin()
{ }

GroundtruthPlugin::~GroundtruthPlugin()
{
  if (updateConnection_)
    updateConnection_->~Connection();
}

void GroundtruthPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the ptr to the model
  model_ = _model;
  model_name_ = model_->GetName();
  // Store the ptr to the world
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_groundtruth_plugin] Please specify a robotNamespace.\n";
  }

  if (!rclcpp::is_initialized()) {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  this->ros_node_ = rclcpp::Node::make_shared("gazeob_ground_truth_plugin_ros");
  
  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GroundtruthPlugin::OnUpdate, this, _1));

  gt_pub_ros_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/" + model_->GetName() + "/groundtruth/odom",
    rclcpp::SystemDefaultsQoS());

}

void GroundtruthPlugin::OnUpdate(const common::UpdateInfo&)
{
  #if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time = world_->SimTime();
  #else
    common::Time current_time = world_->GetSimTime();
  #endif

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d T_W_I = model_->WorldPose();
  #else
    ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
  #endif

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d velocity_current_B = model_->RelativeLinearVel();
  #else
    ignition::math::Vector3d velocity_current_B = ignitionFromGazeboMath(model_->GetRelativeLinearVel());
  #endif

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d rate_current_B = model_->RelativeAngularVel();
  #else
    ignition::math::Vector3d rate_current_B = ignitionFromGazeboMath(model_->GetRelativeAngularVel());
  #endif
  // TODO: Fix frame ids
  nav_msgs::msg::Odometry msg;
  // msg.header.stamp.sec = current_time.sec;
  // msg.header.stamp.nanosec = current_time.nsec;
  msg.header.stamp = ros_node_->get_clock()->now();
  msg.header.frame_id = "world";
  msg.child_frame_id = model_name_;
  msg.pose.pose.position.x = T_W_I.Pos().X();
  msg.pose.pose.position.y = T_W_I.Pos().Y();
  msg.pose.pose.position.z = T_W_I.Pos().Z();
  msg.pose.pose.orientation.x = T_W_I.Rot().X();
  msg.pose.pose.orientation.y = T_W_I.Rot().Y();
  msg.pose.pose.orientation.z = T_W_I.Rot().Z();
  msg.pose.pose.orientation.w = T_W_I.Rot().W();
  msg.pose.covariance[0] = 1e-6;
  msg.pose.covariance[7] = 1e-6;
  msg.pose.covariance[14] = 1e-6;
  msg.pose.covariance[21] = 1e-6;
  msg.pose.covariance[28] = 1e-6;
  msg.pose.covariance[35] = 1e-6;
  msg.twist.twist.linear.x = velocity_current_B.X();
  msg.twist.twist.linear.y = velocity_current_B.Y();
  msg.twist.twist.linear.z = velocity_current_B.Z();
  msg.twist.twist.angular.x = rate_current_B.X();
  msg.twist.twist.angular.y = rate_current_B.Y();
  msg.twist.twist.angular.z = rate_current_B.Z();
  msg.twist.covariance[0] = 1e-6;
  msg.twist.covariance[7] = 1e-6;
  msg.twist.covariance[14] = 1e-6;
  msg.twist.covariance[21] = 1e-6;
  msg.twist.covariance[28] = 1e-6;
  msg.twist.covariance[35] = 1e-6;
  gt_pub_ros_->publish(msg);
}

} // namespace gazebo
