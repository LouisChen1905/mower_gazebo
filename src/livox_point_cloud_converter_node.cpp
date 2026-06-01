#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "mower_msgs/msg/livox_point.hpp"
#include "mower_msgs/msg/livox_point_cloud.hpp"
#include "mower_wrapper_client/TopicName.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
int16_t clampToInt16(double value)
{
  if (!std::isfinite(value)) {
    return 0;
  }

  const auto rounded = std::llround(value);
  return static_cast<int16_t>(std::clamp<long long>(
    rounded,
    std::numeric_limits<int16_t>::min(),
    std::numeric_limits<int16_t>::max()));
}

uint64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<uint64_t>(stamp.nanosec);
}
}  // namespace

class LivoxPointCloudConverter : public rclcpp::Node
{
public:
  LivoxPointCloudConverter()
  : Node("livox_point_cloud_converter")
  {
    declare_parameter("input_topic", std::string("/sensor_lidar"));
    declare_parameter("output_topic", std::string(TOPIC_LIVOX_CLOUD));
    declare_parameter("coordinate_scale", 1000.0);
    declare_parameter("default_intensity", 0);
    declare_parameter("default_tag", 0);

    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("coordinate_scale", coordinate_scale_);
    get_parameter("default_intensity", default_intensity_);
    get_parameter("default_tag", default_tag_);

    coordinate_scale_ = std::max(1.0, coordinate_scale_);
    default_intensity_ = std::clamp(default_intensity_, 0, 255);
    default_tag_ = std::clamp(default_tag_, 0, 255);

    publisher_ = create_publisher<mower_msgs::msg::LivoxPointCloud>(output_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        publishConvertedCloud(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "converting %s PointCloud2 to %s LivoxPointCloud",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  bool hasField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const
  {
    return std::any_of(
      cloud.fields.begin(), cloud.fields.end(),
      [&name](const sensor_msgs::msg::PointField & field) {
        return field.name == name;
      });
  }

  void publishConvertedCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    mower_msgs::msg::LivoxPointCloud output;
    output.timestamp = stampToNanoseconds(cloud.header.stamp);
    output.points.reserve(static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height));

    const bool has_intensity = hasField(cloud, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud, has_intensity ? "intensity" : "x");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        if (has_intensity) {
          ++iter_intensity;
        }
        continue;
      }

      mower_msgs::msg::LivoxPoint point;
      point.x = clampToInt16(static_cast<double>(x) * coordinate_scale_);
      point.y = clampToInt16(static_cast<double>(y) * coordinate_scale_);
      point.z = clampToInt16(static_cast<double>(z) * coordinate_scale_);
      point.intensity = static_cast<uint8_t>(default_intensity_);
      point.tag = static_cast<uint8_t>(default_tag_);

      if (has_intensity) {
        point.intensity = static_cast<uint8_t>(std::clamp(
          static_cast<int>(std::llround(*iter_intensity)), 0, 255));
        ++iter_intensity;
      }

      output.points.push_back(point);
    }

    output.points_num = output.points.size();
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  double coordinate_scale_ = 1000.0;
  int default_intensity_ = 0;
  int default_tag_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<mower_msgs::msg::LivoxPointCloud>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPointCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
