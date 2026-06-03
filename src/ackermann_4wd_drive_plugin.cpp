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
struct WheelMotion
{
  double steer_angle{0.0};
  double linear_speed{0.0};
};

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
    front_wheel_radius_ = getSdfDouble(sdf, "front_wheel_radius", wheel_radius_);
    rear_wheel_radius_ = getSdfDouble(sdf, "rear_wheel_radius", wheel_radius_);
    max_steer_angle_ = getSdfDouble(sdf, "max_steer_angle", 0.65);
    rear_axle_to_base_ = getSdfDouble(sdf, "rear_axle_to_base", wheel_base_ * 0.5);
    max_wheel_torque_ = getSdfDouble(sdf, "max_wheel_torque", 20.0);
    max_wheel_acceleration_ = getSdfDouble(sdf, "max_wheel_acceleration", 8.0);
    max_steer_rate_ = getSdfDouble(sdf, "max_steer_rate", 2.0);
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
    last_update_time_ = world_->SimTime();
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
    const double dt = std::max(0.0, (info.simTime - last_update_time_).Double());
    last_update_time_ = info.simTime;
    const double linear = command_age <= command_timeout_ ? linear_cmd_ : 0.0;
    const double angular = command_age <= command_timeout_ ? angular_cmd_ : 0.0;

    updateSteeringAndWheels(linear, angular, dt);
    publishOdometry(info.simTime);
  }

  void updateSteeringAndWheels(const double linear, const double angular, const double dt)
  {
    const double half_track = wheel_track_ * 0.5;
    const auto front_left = wheelMotionFromRearAxleCenter(wheel_base_, half_track, linear, angular);
    const auto front_right = wheelMotionFromRearAxleCenter(wheel_base_, -half_track, linear, angular);
    const auto rear_left = wheelMotionFromRearAxleCenter(0.0, half_track, linear, angular);
    const auto rear_right = wheelMotionFromRearAxleCenter(0.0, -half_track, linear, angular);

    // RCLCPP_INFO(
    //   node_->get_logger(),
    //   "linear: %.2f m/s, angular: %.2f rad/s, left_steer: %.2f deg, right_steer: %.2f deg",
    //   linear, angular,
    //   front_left.steer_angle * 180.0 / M_PI,
    //   front_right.steer_angle * 180.0 / M_PI);

    front_left_steer_cmd_ =
      limitRate(front_left.steer_angle, front_left_steer_cmd_, max_steer_rate_, dt);
    front_right_steer_cmd_ =
      limitRate(front_right.steer_angle, front_right_steer_cmd_, max_steer_rate_, dt);
    if (front_left_steer_joint_) {
      front_left_steer_joint_->SetPosition(0, front_left_steer_cmd_);
    }
    if (front_right_steer_joint_) {
      front_right_steer_joint_->SetPosition(0, front_right_steer_cmd_);
    }

    front_left_wheel_velocity_cmd_ = limitRate(
      front_left.linear_speed / front_wheel_radius_, front_left_wheel_velocity_cmd_,
      max_wheel_acceleration_, dt);
    front_right_wheel_velocity_cmd_ = limitRate(
      front_right.linear_speed / front_wheel_radius_, front_right_wheel_velocity_cmd_,
      max_wheel_acceleration_, dt);
    rear_left_wheel_velocity_cmd_ = limitRate(
      rear_left.linear_speed / rear_wheel_radius_, rear_left_wheel_velocity_cmd_,
      max_wheel_acceleration_, dt);
    rear_right_wheel_velocity_cmd_ = limitRate(
      rear_right.linear_speed / rear_wheel_radius_, rear_right_wheel_velocity_cmd_,
      max_wheel_acceleration_, dt);

    setWheelVelocity(front_left_wheel_joint_, front_left_wheel_velocity_cmd_);
    setWheelVelocity(front_right_wheel_joint_, front_right_wheel_velocity_cmd_);
    setWheelVelocity(rear_left_wheel_joint_, rear_left_wheel_velocity_cmd_);
    setWheelVelocity(rear_right_wheel_joint_, rear_right_wheel_velocity_cmd_);
  }

  WheelMotion wheelMotionFromRearAxleCenter(
    const double wheel_x,
    const double wheel_y,
    const double rear_axle_linear,
    const double angular) const
  {
    const double velocity_x = rear_axle_linear - angular * wheel_y;
    const double velocity_y = angular * wheel_x;

    WheelMotion motion;
    if (std::abs(wheel_x) < 1e-6) {
      motion.linear_speed = velocity_x;
      return motion;
    }

    motion.steer_angle = std::atan2(velocity_y, velocity_x);
    motion.linear_speed = std::hypot(velocity_x, velocity_y);

    if (motion.steer_angle > M_PI_2) {
      motion.steer_angle -= M_PI;
      motion.linear_speed = -motion.linear_speed;
    } else if (motion.steer_angle < -M_PI_2) {
      motion.steer_angle += M_PI;
      motion.linear_speed = -motion.linear_speed;
    }

    motion.steer_angle = std::clamp(motion.steer_angle, -max_steer_angle_, max_steer_angle_);
    motion.linear_speed =
      velocity_x * std::cos(motion.steer_angle) + velocity_y * std::sin(motion.steer_angle);
    return motion;
  }

  static double limitRate(
    const double target,
    const double current,
    const double max_rate,
    const double dt)
  {
    if (dt <= 0.0 || max_rate <= 0.0) {
      return target;
    }
    const double max_delta = max_rate * dt;
    return current + std::clamp(target - current, -max_delta, max_delta);
  }

  void setWheelVelocity(const gazebo::physics::JointPtr & joint, const double velocity)
  {
    if (joint) {
      joint->SetParam("fmax", 0, max_wheel_torque_);
      joint->SetParam("vel", 0, velocity);
    }
  }

  void publishOdometry(const gazebo::common::Time & sim_time)
  {
    const rclcpp::Time stamp(sim_time.sec, sim_time.nsec, RCL_ROS_TIME);
    const auto pose = model_->WorldPose();
    const auto world_linear = model_->WorldLinearVel();
    const auto world_angular = model_->WorldAngularVel();
    const double yaw = pose.Rot().Yaw();
    const double body_linear_x =
      world_linear.X() * std::cos(yaw) + world_linear.Y() * std::sin(yaw);
    const double body_linear_y =
      -world_linear.X() * std::sin(yaw) + world_linear.Y() * std::cos(yaw);

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
    odom.twist.twist.linear.x = body_linear_x;
    odom.twist.twist.linear.y = body_linear_y;
    odom.twist.twist.angular.z = world_angular.Z();
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
  double front_wheel_radius_{0.12};
  double rear_wheel_radius_{0.12};
  double max_steer_angle_{0.65};
  double rear_axle_to_base_{0.36};
  double max_wheel_torque_{20.0};
  double max_wheel_acceleration_{8.0};
  double max_steer_rate_{2.0};
  double command_timeout_{0.3};
  bool publish_odom_tf_{true};

  double linear_cmd_{0.0};
  double angular_cmd_{0.0};
  double front_left_steer_cmd_{0.0};
  double front_right_steer_cmd_{0.0};
  double front_left_wheel_velocity_cmd_{0.0};
  double front_right_wheel_velocity_cmd_{0.0};
  double rear_left_wheel_velocity_cmd_{0.0};
  double rear_right_wheel_velocity_cmd_{0.0};
  gazebo::common::Time last_cmd_time_;
  gazebo::common::Time last_update_time_;
};
}  // namespace mower_gazebo

GZ_REGISTER_MODEL_PLUGIN(mower_gazebo::Ackermann4wdDrivePlugin)
