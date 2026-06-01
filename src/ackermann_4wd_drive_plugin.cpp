#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mower_gazebo
{
class Ackermann4wdDrivePlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    cmd_vel_topic_ = getSdfString(sdf, "cmd_vel_topic", "cmd_vel");
    odom_topic_ = getSdfString(sdf, "odom_topic", "odom");
    odom_frame_ = getSdfString(sdf, "odom_frame", "odom");
    base_frame_ = getSdfString(sdf, "base_frame", "base_link");
    wheel_base_ = getSdfDouble(sdf, "wheel_base", 0.72);
    wheel_track_ = getSdfDouble(sdf, "wheel_track", 0.64);
    wheel_radius_ = getSdfDouble(sdf, "wheel_radius", 0.12);
    max_steer_angle_ = getSdfDouble(sdf, "max_steer_angle", 0.65);
    command_timeout_ = getSdfDouble(sdf, "command_timeout", 0.3);
    publish_odom_tf_ = getSdfBool(sdf, "publish_odom_tf", true);

    front_left_steer_joint_ = getJoint(sdf, "front_left_steer_joint", "front_left_steer_joint");
    front_right_steer_joint_ = getJoint(sdf, "front_right_steer_joint", "front_right_steer_joint");
    front_left_wheel_joint_ = getJoint(sdf, "front_left_wheel_joint", "front_left_wheel_joint");
    front_right_wheel_joint_ = getJoint(sdf, "front_right_wheel_joint", "front_right_wheel_joint");
    rear_left_wheel_joint_ = getJoint(sdf, "rear_left_wheel_joint", "rear_left_wheel_joint");
    rear_right_wheel_joint_ = getJoint(sdf, "rear_right_wheel_joint", "rear_right_wheel_joint");

    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_cmd_ = msg->linear.x;
        angular_cmd_ = msg->angular.z;
        last_cmd_time_ = world_->SimTime();
      });

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&Ackermann4wdDrivePlugin::OnUpdate, this, std::placeholders::_1));

    last_cmd_time_ = world_->SimTime();
    RCLCPP_INFO(
      node_->get_logger(),
      "Ackermann 4WD plugin loaded for model [%s], cmd_vel [%s], odom [%s]",
      model_->GetName().c_str(), cmd_vel_topic_.c_str(), odom_topic_.c_str());
  }

private:
  static std::string getSdfString(
    const sdf::ElementPtr & sdf, const std::string & name, const std::string & default_value)
  {
    return sdf->HasElement(name) ? sdf->Get<std::string>(name) : default_value;
  }

  static double getSdfDouble(
    const sdf::ElementPtr & sdf, const std::string & name, const double default_value)
  {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : default_value;
  }

  static bool getSdfBool(
    const sdf::ElementPtr & sdf, const std::string & name, const bool default_value)
  {
    return sdf->HasElement(name) ? sdf->Get<bool>(name) : default_value;
  }

  gazebo::physics::JointPtr getJoint(
    const sdf::ElementPtr & sdf, const std::string & tag_name, const std::string & default_name)
  {
    const auto joint_name = getSdfString(sdf, tag_name, default_name);
    auto joint = model_->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(node_->get_logger(), "Joint [%s] was not found", joint_name.c_str());
    }
    return joint;
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double command_age = (info.simTime - last_cmd_time_).Double();
    const double linear = command_age <= command_timeout_ ? linear_cmd_ : 0.0;
    const double angular = command_age <= command_timeout_ ? angular_cmd_ : 0.0;

    const auto pose = model_->WorldPose();
    const double yaw = pose.Rot().Yaw();
    model_->SetLinearVel({linear * std::cos(yaw), linear * std::sin(yaw), 0.0});
    model_->SetAngularVel({0.0, 0.0, angular});

    updateSteeringAndWheels(linear, angular);
    publishOdometry(info.simTime, pose, linear, angular);
  }

  void updateSteeringAndWheels(const double linear, const double angular)
  {
    double left_steer = 0.0;
    double right_steer = 0.0;

    if (std::abs(linear) > 1e-3 && std::abs(angular) > 1e-3) {
      const double turn_radius = linear / angular;
      left_steer = std::atan2(wheel_base_, turn_radius - std::copysign(wheel_track_ * 0.5, angular));
      right_steer = std::atan2(wheel_base_, turn_radius + std::copysign(wheel_track_ * 0.5, angular));
    }

    left_steer = std::clamp(left_steer, -max_steer_angle_, max_steer_angle_);
    right_steer = std::clamp(right_steer, -max_steer_angle_, max_steer_angle_);
    if (front_left_steer_joint_) {
      front_left_steer_joint_->SetPosition(0, left_steer);
    }
    if (front_right_steer_joint_) {
      front_right_steer_joint_->SetPosition(0, right_steer);
    }

    const double left_wheel_velocity = (linear - angular * wheel_track_ * 0.5) / wheel_radius_;
    const double right_wheel_velocity = (linear + angular * wheel_track_ * 0.5) / wheel_radius_;
    setWheelVelocity(front_left_wheel_joint_, left_wheel_velocity);
    setWheelVelocity(rear_left_wheel_joint_, left_wheel_velocity);
    setWheelVelocity(front_right_wheel_joint_, right_wheel_velocity);
    setWheelVelocity(rear_right_wheel_joint_, right_wheel_velocity);
  }

  static void setWheelVelocity(const gazebo::physics::JointPtr & joint, const double velocity)
  {
    if (joint) {
      joint->SetVelocity(0, velocity);
    }
  }

  void publishOdometry(
    const gazebo::common::Time & sim_time,
    const ignition::math::Pose3d & pose,
    const double linear,
    const double angular)
  {
    const rclcpp::Time stamp(sim_time.sec, sim_time.nsec, RCL_ROS_TIME);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = linear;
    odom.twist.twist.angular.z = angular;
    odom_publisher_->publish(odom);

    if (!publish_odom_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pose.Pos().X();
    transform.transform.translation.y = pose.Pos().Y();
    transform.transform.translation.z = pose.Pos().Z();
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_connection_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  gazebo::physics::JointPtr front_left_steer_joint_;
  gazebo::physics::JointPtr front_right_steer_joint_;
  gazebo::physics::JointPtr front_left_wheel_joint_;
  gazebo::physics::JointPtr front_right_wheel_joint_;
  gazebo::physics::JointPtr rear_left_wheel_joint_;
  gazebo::physics::JointPtr rear_right_wheel_joint_;

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double wheel_base_{0.72};
  double wheel_track_{0.64};
  double wheel_radius_{0.12};
  double max_steer_angle_{0.65};
  double command_timeout_{0.3};
  bool publish_odom_tf_{true};

  double linear_cmd_{0.0};
  double angular_cmd_{0.0};
  gazebo::common::Time last_cmd_time_;
};
}  // namespace mower_gazebo

GZ_REGISTER_MODEL_PLUGIN(mower_gazebo::Ackermann4wdDrivePlugin)
