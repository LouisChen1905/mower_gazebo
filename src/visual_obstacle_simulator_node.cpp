#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "mower_msgs/msg/obs_info.hpp"
#include "mower_msgs/msg/vertex_info.hpp"
#include "mower_msgs/msg/visual_obs.hpp"
#include "mower_wrapper_client/TopicName.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
constexpr uint8_t kObjectIdOther = 255;

struct Vec3
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct VoxelKey
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  size_t operator()(const VoxelKey & key) const
  {
    const size_t hx = std::hash<int>{}(key.x);
    const size_t hy = std::hash<int>{}(key.y);
    const size_t hz = std::hash<int>{}(key.z);
    return hx ^ (hy << 1) ^ (hz << 2);
  }
};

mower_msgs::msg::VertexInfo makeVertex(float x, float y, float z)
{
  mower_msgs::msg::VertexInfo vertex;
  vertex.x = x;
  vertex.y = y;
  vertex.z = z;
  return vertex;
}
}  // namespace

class VisualObstacleSimulator : public rclcpp::Node
{
public:
  VisualObstacleSimulator()
  : Node("mower_visual_obstacle_simulator")
  {
    loadParameters();

    visual_obs_publisher_ =
      create_publisher<mower_msgs::msg::VisualObs>(visual_obs_topic_, 10);
    point_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        publishVisualObstacle(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "visual obstacle simulator converts %s to %s",
      point_cloud_topic_.c_str(), visual_obs_topic_.c_str());
  }

private:
  void loadParameters()
  {
    declare_parameter("point_cloud_topic", std::string("/mower_tof/points"));
    declare_parameter("visual_obs_topic", std::string(TOPIC_VISUAL_OBS_FRONT));
    declare_parameter("sensor_id", 0);
    declare_parameter("object_id", static_cast<int>(kObjectIdOther));
    declare_parameter("dynamic", 0);
    declare_parameter("min_depth", 0.05);
    declare_parameter("max_depth", 2.5);
    declare_parameter("min_points", 3);
    declare_parameter("cluster_tolerance", 0.12);
    declare_parameter("max_objects", 8);

    get_parameter("point_cloud_topic", point_cloud_topic_);
    get_parameter("visual_obs_topic", visual_obs_topic_);
    get_parameter("sensor_id", sensor_id_);
    get_parameter("object_id", object_id_);
    get_parameter("dynamic", dynamic_);
    get_parameter("min_depth", min_depth_);
    get_parameter("max_depth", max_depth_);
    get_parameter("min_points", min_points_);
    get_parameter("cluster_tolerance", cluster_tolerance_);
    get_parameter("max_objects", max_objects_);

    sensor_id_ = std::clamp(sensor_id_, 0, 255);
    object_id_ = std::clamp(object_id_, 0, 255);
    dynamic_ = std::clamp(dynamic_, 0, 255);
    min_depth_ = std::max(0.0, min_depth_);
    max_depth_ = std::max(min_depth_ + 0.01, max_depth_);
    min_points_ = std::max(1, min_points_);
    cluster_tolerance_ = std::max(0.01, cluster_tolerance_);
    max_objects_ = std::clamp(max_objects_, 1, 255);
  }

  void publishVisualObstacle(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    mower_msgs::msg::VisualObs visual_obs;
    visual_obs.timestamp = now().nanoseconds() / 1000000;
    visual_obs.sensor_id = static_cast<uint8_t>(sensor_id_);

    const auto points = extractValidPoints(cloud);
    const auto clusters = clusterPoints(points);
    if (!clusters.empty()) {
      visual_obs.state = 1;
      visual_obs.sum = static_cast<uint8_t>(
        std::min<size_t>(clusters.size(), std::numeric_limits<uint8_t>::max()));
      for (const auto & cluster : clusters) {
        visual_obs.obs.push_back(makeObstacle(points, cluster));
      }
    } else {
      visual_obs.state = 0;
      visual_obs.sum = 0;
    }

    visual_obs_publisher_->publish(visual_obs);
  }

  std::vector<Vec3> extractValidPoints(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    std::vector<Vec3> points;
    points.reserve(static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Vec3 point{*iter_x, *iter_y, *iter_z};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < min_depth_ || point.z > max_depth_) {
        continue;
      }

      const Vec3 point_in_camera({-*iter_y, -*iter_z, *iter_x});
      points.push_back(point_in_camera);
    }

    return points;
  }

  std::vector<std::vector<size_t>> clusterPoints(const std::vector<Vec3> & points) const
  {
    std::vector<std::vector<size_t>> clusters;
    if (static_cast<int>(points.size()) < min_points_) {
      return clusters;
    }

    std::unordered_map<VoxelKey, std::vector<size_t>, VoxelKeyHash> voxel_index;
    voxel_index.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      voxel_index[voxelKey(points[i])].push_back(i);
    }

    std::vector<bool> visited(points.size(), false);
    const double tolerance_sq = cluster_tolerance_ * cluster_tolerance_;
    for (size_t i = 0; i < points.size(); ++i) {
      if (visited[i]) {
        continue;
      }

      std::vector<size_t> cluster;
      std::queue<size_t> pending;
      visited[i] = true;
      pending.push(i);

      while (!pending.empty()) {
        const size_t point_index = pending.front();
        pending.pop();
        cluster.push_back(point_index);

        const auto center_key = voxelKey(points[point_index]);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              const VoxelKey neighbor_key{
                center_key.x + dx, center_key.y + dy, center_key.z + dz};
              const auto neighbor = voxel_index.find(neighbor_key);
              if (neighbor == voxel_index.end()) {
                continue;
              }

              for (const size_t candidate_index : neighbor->second) {
                if (visited[candidate_index]) {
                  continue;
                }
                if (squaredDistance(points[point_index], points[candidate_index]) > tolerance_sq) {
                  continue;
                }
                visited[candidate_index] = true;
                pending.push(candidate_index);
              }
            }
          }
        }
      }

      if (static_cast<int>(cluster.size()) >= min_points_) {
        clusters.push_back(std::move(cluster));
      }
    }

    std::sort(
      clusters.begin(), clusters.end(),
      [&points](const auto & lhs, const auto & rhs) {
        return averageDepth(points, lhs) < averageDepth(points, rhs);
      });
    if (static_cast<int>(clusters.size()) > max_objects_) {
      clusters.resize(static_cast<size_t>(max_objects_));
    }
    return clusters;
  }

  VoxelKey voxelKey(const Vec3 & point) const
  {
    return {
      static_cast<int>(std::floor(point.x / cluster_tolerance_)),
      static_cast<int>(std::floor(point.y / cluster_tolerance_)),
      static_cast<int>(std::floor(point.z / cluster_tolerance_))};
  }

  static double squaredDistance(const Vec3 & lhs, const Vec3 & rhs)
  {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
  }

  static double averageDepth(const std::vector<Vec3> & points, const std::vector<size_t> & cluster)
  {
    double z_sum = 0.0;
    for (const size_t index : cluster) {
      z_sum += points[index].z;
    }
    return z_sum / static_cast<double>(cluster.size());
  }

  mower_msgs::msg::ObsInfo makeObstacle(
    const std::vector<Vec3> & points,
    const std::vector<size_t> & cluster) const
  {
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    double z_sum = 0.0;

    for (const size_t index : cluster) {
      const auto & point = points[index];
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      z_sum += point.z;
    }

    const float z = static_cast<float>(z_sum / static_cast<double>(cluster.size()));

    mower_msgs::msg::ObsInfo obstacle;
    obstacle.sum = 4;
    obstacle.object_id = static_cast<uint8_t>(object_id_);
    obstacle.dynamic = static_cast<uint8_t>(dynamic_);
    obstacle.vertex_info = {
      makeVertex(min_x, min_y, z),
      makeVertex(max_x, min_y, z),
      makeVertex(max_x, max_y, z),
      makeVertex(min_x, max_y, z),
    };
    return obstacle;
  }

  std::string point_cloud_topic_;
  std::string visual_obs_topic_;
  int sensor_id_ = 0;
  int object_id_ = kObjectIdOther;
  int dynamic_ = 0;
  double min_depth_ = 0.05;
  double max_depth_ = 2.5;
  int min_points_ = 3;
  double cluster_tolerance_ = 0.12;
  int max_objects_ = 8;

  rclcpp::Publisher<mower_msgs::msg::VisualObs>::SharedPtr visual_obs_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualObstacleSimulator>());
  rclcpp::shutdown();
  return 0;
}
