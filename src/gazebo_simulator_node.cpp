#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>

#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mower_msgs/msg/dock_pole.hpp"
#include "mower_msgs/msg/gps.hpp"
#include "mower_msgs/msg/location.hpp"
#include "mower_msgs/msg/motor.hpp"
#include "mower_msgs/msg/planning.hpp"
#include "mower_msgs/msg/position_wgs.hpp"
#include "mower_msgs/msg/safety_ctrl_evt.hpp"
#include "mower_wrapper_client/TopicName.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace
{
constexpr double kEarthRadius = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257;

double normalizeAngle(double angle)
{
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

double getYaw(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion quaternion(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 matrix(quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  matrix.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quat);
}

double radiansToDDMMmmm(double radians)
{
  const double degrees = radians * 180.0 / M_PI;
  const int whole_degrees = static_cast<int>(degrees);
  const double minutes = (degrees - whole_degrees) * 60.0;
  return whole_degrees * 100.0 + minutes;
}

void radiiOfCurvature(double latitude, double & rm, double & rn)
{
  const double sin_lat = std::sin(latitude);
  rm = kEarthRadius * (1.0 - 2.0 * kFlattening + 3.0 * kFlattening * sin_lat * sin_lat);
  rn = kEarthRadius * (1.0 + kFlattening * sin_lat * sin_lat);
}

void enuToWgs(
  double east, double north, double up,
  double anchor_latitude_deg, double anchor_longitude_deg,
  double & latitude_rad, double & longitude_rad, double & height)
{
  const double anchor_latitude = anchor_latitude_deg / 180.0 * M_PI;
  const double anchor_longitude = anchor_longitude_deg / 180.0 * M_PI;

  double rm = 0.0;
  double rn = 0.0;
  radiiOfCurvature(anchor_latitude, rm, rn);

  latitude_rad = north / (rm + up) + anchor_latitude;
  longitude_rad = east / ((rn + up) * std::cos(anchor_latitude)) + anchor_longitude;
  height = up;
}
}  // namespace

class GazeboSimulator : public rclcpp::Node
{
public:
  GazeboSimulator()
  : Node("mower_gazebo_simulator")
  {
    loadParameters();

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    location_publisher_ = create_publisher<mower_msgs::msg::Location>(TOPIC_LOCATION, 10);
    gps_publisher_ = create_publisher<mower_msgs::msg::Gps>(TOPIC_RTK, 10);
    left_motor_publisher_ = create_publisher<mower_msgs::msg::Motor>(TOPIC_MOTOR_LEFT, 10);
    right_motor_publisher_ = create_publisher<mower_msgs::msg::Motor>(TOPIC_MOTOR_RIGHT, 10);
    cut_motor_publisher_ = create_publisher<mower_msgs::msg::Motor>(TOPIC_MOTOR_CUT, 10);
    safety_publisher_ = create_publisher<mower_msgs::msg::SafetyCtrlEvt>(TOPIC_SAFETY_EVENT, 10);
    dock_pole_publisher_ = create_publisher<mower_msgs::msg::DockPole>(TOPIC_DOCK_POLE, 10);

    set_entity_state_client_ =
      create_client<gazebo_msgs::srv::SetEntityState>(set_entity_state_service_);

    planning_subscription_ = create_subscription<mower_msgs::msg::Planning>(
      TOPIC_PLANNING, 10,
      [this](mower_msgs::msg::Planning::SharedPtr msg) {
        onPlanning(*msg);
      });

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        onOdometry(*msg);
      });

    reset_pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
      "reset_mower_pose", 10,
      [this](geometry_msgs::msg::Pose::SharedPtr msg) {
        resetGazeboPose(*msg);
      });

    anchor_subscription_ = create_subscription<mower_msgs::msg::PositionWGS>(
      TOPIC_LOCATION_ANCHOR, 10,
      [this](mower_msgs::msg::PositionWGS::SharedPtr msg) {
        origin_wgs_ = *msg;
        has_origin_wgs_ = !std::isnan(origin_wgs_.latitude) && !std::isnan(origin_wgs_.longitude);
      });

    slip_flag_subscription_ = create_subscription<std_msgs::msg::Bool>(
      TOPIC_SLIP_FLAG, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        slip_flag_ = msg->data;
      });

    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(status_period_ms_),
      [this]() {
        publishStatus();
      });

    command_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        stopOnCommandTimeout();
      });

    RCLCPP_INFO(
      get_logger(),
      "mower_gazebo_simulator bridges %s -> %s and %s -> %s/%s",
      TOPIC_PLANNING, cmd_vel_topic_.c_str(), odom_topic_.c_str(), TOPIC_LOCATION, TOPIC_RTK);
  }

private:
  void loadParameters()
  {
    declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    declare_parameter("odom_topic", std::string("/odom"));
    declare_parameter("model_name", std::string("mower_robot"));
    declare_parameter("set_entity_state_service", std::string("/set_entity_state"));
    declare_parameter("wheel_base", 0.68);
    declare_parameter("wheel_radius", 0.12);
    declare_parameter("command_timeout_ms", 300.0);
    declare_parameter("status_period_ms", 200);
    declare_parameter("drive_linear_noise_stddev", 0.0);
    declare_parameter("drive_angular_noise_stddev", 0.0);
    declare_parameter("drive_linear_noise_bias", 0.0);
    declare_parameter("drive_angular_noise_bias", 0.0);

    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("model_name", model_name_);
    get_parameter("set_entity_state_service", set_entity_state_service_);
    get_parameter("wheel_base", wheel_base_);
    get_parameter("wheel_radius", wheel_radius_);
    get_parameter("command_timeout_ms", command_timeout_ms_);
    get_parameter("status_period_ms", status_period_ms_);
    get_parameter("drive_linear_noise_stddev", drive_linear_noise_stddev_);
    get_parameter("drive_angular_noise_stddev", drive_angular_noise_stddev_);
    get_parameter("drive_linear_noise_bias", drive_linear_noise_bias_);
    get_parameter("drive_angular_noise_bias", drive_angular_noise_bias_);
  }

  void onPlanning(const mower_msgs::msg::Planning & planning)
  {
    const double linear_cmd = applyGaussianNoise(
      planning.line_speed, drive_linear_noise_bias_, drive_linear_noise_stddev_);
    const double angular_cmd = applyGaussianNoise(
      planning.angular_speed, drive_angular_noise_bias_, drive_angular_noise_stddev_);

    last_linear_speed_cmd_ = linear_cmd;
    last_angular_speed_cmd_ = angular_cmd;
    last_command_time_ = now();

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_cmd;
    cmd_vel.angular.z = angular_cmd;
    cmd_vel_publisher_->publish(cmd_vel);
  }

  void onOdometry(const nav_msgs::msg::Odometry & odom)
  {
    const auto & pose = odom.pose.pose;
    const auto & twist = odom.twist.twist;
    const double yaw = getYaw(pose.orientation);

    mower_msgs::msg::Location location;
    location.timestamp = now().nanoseconds();
    location.gps_status = 4;
    location.rtk_status = 4;
    location.vio_status = 0;
    location.error_distance = 0.0;
    location.slip_status = slip_flag_ ? 1 : 0;
    location.fall_status = 0;
    location.mower_sat_num = 20;
    location.mean_cno = 40.0;
    location.odom_speed = static_cast<float>(twist.linear.x);

    // Existing rviz_wrapper simulator maps Gazebo/XYZ to mower ENU as:
    // east = -y, north = x.
    location.pos_enu.east = static_cast<float>(-pose.position.y);
    location.pos_enu.north = static_cast<float>(pose.position.x);
    location.pos_enu.up = static_cast<float>(pose.position.z);
    location.rtk_enu = location.pos_enu;
    location.att.head = static_cast<float>(normalizeAngle(yaw));
    location.att.pitch = 0.0;
    location.att.roll = 0.0;
    location.pose_oi.position = location.pos_enu;
    location.pose_oi.attitude = location.att;

    if (has_origin_wgs_) {
      double latitude_rad = 0.0;
      double longitude_rad = 0.0;
      double height = 0.0;
      enuToWgs(
        location.pos_enu.east, location.pos_enu.north, location.pos_enu.up,
        origin_wgs_.latitude, origin_wgs_.longitude,
        latitude_rad, longitude_rad, height);

      location.pos_wgs.latitude = latitude_rad * 180.0 / M_PI;
      location.pos_wgs.longitude = longitude_rad * 180.0 / M_PI;
      location.pos_wgs.height = static_cast<float>(height);

      mower_msgs::msg::Gps gps;
      gps.timestamp = location.timestamp;
      gps.ppstimestamp = location.timestamp;
      gps.status = 4;
      gps.heading = static_cast<float>(yaw);
      gps.latitude = radiansToDDMMmmm(latitude_rad);
      gps.longitude = radiansToDDMMmmm(longitude_rad);
      gps.elevation = static_cast<float>(height);
      gps.speed = static_cast<float>(std::hypot(twist.linear.x, twist.linear.y));
      gps.vertspeed = static_cast<float>(twist.linear.z);
      gps.pdop = 1.0;
      gps.hdop = 0.8;
      gps.vdop = 1.2;
      gps.satenum = 20;
      gps.stdlat = 0.02;
      gps.stdlong = 0.02;
      gps.stdalt = 0.05;
      gps_publisher_->publish(gps);
    }

    location_publisher_->publish(location);
  }

  void resetGazeboPose(const geometry_msgs::msg::Pose & enu_pose)
  {
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = model_name_;
    request->state.reference_frame = "world";

    // Match rviz_wrapper simulator conversion: ENU east/north -> Gazebo x/y.
    request->state.pose.position.x = enu_pose.position.y;
    request->state.pose.position.y = -enu_pose.position.x;
    request->state.pose.position.z = enu_pose.position.z;

    const double enu_yaw = getYaw(enu_pose.orientation);
    request->state.pose.orientation = yawToQuaternion(normalizeAngle(enu_yaw - M_PI_2));

    request->state.twist = geometry_msgs::msg::Twist();

    if (!set_entity_state_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(
        get_logger(), "Gazebo service %s is not available; cannot reset mower pose",
        set_entity_state_service_.c_str());
      return;
    }

    (void)set_entity_state_client_->async_send_request(request);
    publishZeroCommand();
  }

  void publishStatus()
  {
    const auto timestamp = now().nanoseconds();

    mower_msgs::msg::Motor left_motor;
    mower_msgs::msg::Motor right_motor;
    mower_msgs::msg::Motor cut_motor;
    left_motor.timestamp = timestamp;
    right_motor.timestamp = timestamp;
    cut_motor.timestamp = timestamp;
    left_motor.status = 0x00;
    right_motor.status = 0x00;
    cut_motor.status = 0x00;

    const double left_speed =
      last_linear_speed_cmd_ - last_angular_speed_cmd_ * wheel_base_ / 2.0;
    const double right_speed =
      last_linear_speed_cmd_ + last_angular_speed_cmd_ * wheel_base_ / 2.0;
    left_motor.rpm = speedToRpm(left_speed);
    right_motor.rpm = speedToRpm(right_speed);
    cut_motor.rpm = 0;

    left_motor_publisher_->publish(left_motor);
    right_motor_publisher_->publish(right_motor);
    cut_motor_publisher_->publish(cut_motor);

    mower_msgs::msg::SafetyCtrlEvt safety;
    safety.evt = 0;
    std::fill(safety.source.begin(), safety.source.end(), mower_msgs::msg::SafetyCtrlEvt::E_SAFE_ST_NONE);
    safety_publisher_->publish(safety);

    mower_msgs::msg::DockPole dock_pole;
    dock_pole.on_dock_state = 3;
    dock_pole.dock_pole_state = 3;
    dock_pole.resistance = 0;
    dock_pole_publisher_->publish(dock_pole);
  }

  void stopOnCommandTimeout()
  {
    if (last_command_time_.nanoseconds() == 0) {
      return;
    }

    const double elapsed_ms = (now() - last_command_time_).seconds() * 1000.0;
    if (elapsed_ms <= command_timeout_ms_) {
      return;
    }

    if (last_linear_speed_cmd_ != 0.0 || last_angular_speed_cmd_ != 0.0) {
      publishZeroCommand();
    }
  }

  void publishZeroCommand()
  {
    last_linear_speed_cmd_ = 0.0;
    last_angular_speed_cmd_ = 0.0;
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel_publisher_->publish(cmd_vel);
  }

  int16_t speedToRpm(double speed) const
  {
    const double rpm = speed / (2.0 * M_PI * wheel_radius_) * 60.0;
    const double clamped = std::clamp(
      rpm,
      static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max()));
    return static_cast<int16_t>(std::lround(clamped));
  }

  double applyGaussianNoise(double value, double bias, double stddev)
  {
    if (stddev <= 0.0 && bias == 0.0) {
      return value;
    }

    const double noise = (stddev > 0.0) ?
      std::normal_distribution<double>(0.0, stddev)(rand_engine_) : 0.0;
    return value + bias + noise;
  }

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string model_name_;
  std::string set_entity_state_service_;

  double wheel_base_ = 0.68;
  double wheel_radius_ = 0.12;
  double command_timeout_ms_ = 300.0;
  int status_period_ms_ = 200;
  double drive_linear_noise_stddev_ = 0.0;
  double drive_angular_noise_stddev_ = 0.0;
  double drive_linear_noise_bias_ = 0.0;
  double drive_angular_noise_bias_ = 0.0;

  double last_linear_speed_cmd_ = 0.0;
  double last_angular_speed_cmd_ = 0.0;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::default_random_engine rand_engine_{std::random_device{}()};

  bool slip_flag_ = false;
  bool has_origin_wgs_ = false;
  mower_msgs::msg::PositionWGS origin_wgs_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<mower_msgs::msg::Location>::SharedPtr location_publisher_;
  rclcpp::Publisher<mower_msgs::msg::Gps>::SharedPtr gps_publisher_;
  rclcpp::Publisher<mower_msgs::msg::Motor>::SharedPtr left_motor_publisher_;
  rclcpp::Publisher<mower_msgs::msg::Motor>::SharedPtr right_motor_publisher_;
  rclcpp::Publisher<mower_msgs::msg::Motor>::SharedPtr cut_motor_publisher_;
  rclcpp::Publisher<mower_msgs::msg::SafetyCtrlEvt>::SharedPtr safety_publisher_;
  rclcpp::Publisher<mower_msgs::msg::DockPole>::SharedPtr dock_pole_publisher_;

  rclcpp::Subscription<mower_msgs::msg::Planning>::SharedPtr planning_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr reset_pose_subscription_;
  rclcpp::Subscription<mower_msgs::msg::PositionWGS>::SharedPtr anchor_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr slip_flag_subscription_;

  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr command_timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboSimulator>());
  rclcpp::shutdown();
  return 0;
}
