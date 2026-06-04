#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mower_msgs/msg/obs_info.hpp"
#include "mower_msgs/msg/vertex_info.hpp"
#include "mower_msgs/msg/visual_obs.hpp"
#include "mower_wrapper_client/TopicName.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr uint8_t kObjectIdOther = 255;
constexpr uint8_t kObjectIdNonGrass = 183;

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

geometry_msgs::msg::Point toPoint(const Vec3 & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

bool isInsidePolygon(const std::vector<Vec3> & polygon, const Vec3 & point)
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const float yi = polygon[i].y;
    const float yj = polygon[j].y;
    const float xi = polygon[i].x;
    const float xj = polygon[j].x;

    if (((yi > point.y) != (yj > point.y)) &&
      (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }

  return inside;
}
}  // namespace

class VisualObstacleSimulator : public rclcpp::Node
{
public:
  VisualObstacleSimulator()
  : Node("mower_visual_obstacle_simulator")
  {
    loadParameters();
    loadBoundaryPolygon();

    visual_obs_publisher_ =
      create_publisher<mower_msgs::msg::VisualObs>(visual_obs_topic_, 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
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
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("camera_frame", std::string("camera_link"));
    declare_parameter("base_frame", std::string("base_link"));
    declare_parameter("boundary_points_file", std::string("/home/chensi/boundary_points.txt"));
    declare_parameter("polygon_vertices_file", std::string("/home/chensi/polygon_vertices.txt"));
    declare_parameter("boundary_half_size", 10.0);
    declare_parameter("enable_non_grass_detection", true);
    declare_parameter("non_grass_grid_resolution", 0.14);
    declare_parameter("non_grass_window_radius", 3.0);
    declare_parameter("horizontal_fov_deg", 90.0);
    declare_parameter("vertical_fov_deg", 70.0);
    declare_parameter("distance_to_camera_threshold", 2.0);

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
    get_parameter("map_frame", map_frame_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("base_frame", base_frame_);
    get_parameter("boundary_points_file", boundary_points_file_);
    get_parameter("polygon_vertices_file", polygon_vertices_file_);
    get_parameter("boundary_half_size", boundary_half_size_);
    get_parameter("enable_non_grass_detection", enable_non_grass_detection_);
    get_parameter("non_grass_grid_resolution", non_grass_grid_resolution_);
    get_parameter("non_grass_window_radius", non_grass_window_radius_);
    get_parameter("horizontal_fov_deg", horizontal_fov_deg_);
    get_parameter("vertical_fov_deg", vertical_fov_deg_);
    get_parameter("distance_to_camera_threshold", distance_to_camera_threshold_);

    sensor_id_ = std::clamp(sensor_id_, 0, 255);
    object_id_ = std::clamp(object_id_, 0, 255);
    dynamic_ = std::clamp(dynamic_, 0, 255);
    min_depth_ = std::max(0.0, min_depth_);
    max_depth_ = std::max(min_depth_ + 0.01, max_depth_);
    min_points_ = std::max(1, min_points_);
    cluster_tolerance_ = std::max(0.01, cluster_tolerance_);
    max_objects_ = std::clamp(max_objects_, 1, 255);
    boundary_half_size_ = std::max(0.1, boundary_half_size_);
    non_grass_grid_resolution_ = std::max(0.05, non_grass_grid_resolution_);
    non_grass_window_radius_ = std::max(non_grass_grid_resolution_, non_grass_window_radius_);
    distance_to_camera_threshold_ = std::max(0.1, distance_to_camera_threshold_);
  }

  void publishVisualObstacle(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    mower_msgs::msg::VisualObs visual_obs;
    visual_obs.timestamp = now().nanoseconds() / 1000000;
    visual_obs.sensor_id = static_cast<uint8_t>(sensor_id_);

    const auto obstacle_points = extractValidPoints(cloud);
    std::vector<Vec3> non_grass_points;
    appendNonGrassPoints(non_grass_points);

    const auto obstacle_clusters = clusterPoints(obstacle_points);
    const auto non_grass_clusters = clusterPoints(non_grass_points);
    RCLCPP_INFO(
      get_logger(),
      "Extracted %zu obstacle points in %zu clusters, and %zu non-grass points in %zu clusters",
      obstacle_points.size(), obstacle_clusters.size(),
      non_grass_points.size(), non_grass_clusters.size());
    if (!obstacle_points.empty() || !non_grass_points.empty()) {
      visual_obs.state = 1;
      appendObstacles(
        visual_obs, obstacle_points, {}, static_cast<uint8_t>(object_id_));
      appendObstacles(visual_obs, non_grass_points, {}, kObjectIdNonGrass);
      visual_obs.sum = static_cast<uint8_t>(visual_obs.obs.size());
    } else {
      visual_obs.state = 0;
      visual_obs.sum = 0;
    }

    visual_obs_publisher_->publish(visual_obs);
  }

  void appendObstacles(
    mower_msgs::msg::VisualObs & visual_obs,
    const std::vector<Vec3> & points,
    const std::vector<std::vector<size_t>> & clusters,
    uint8_t object_id) const
  {
    if (clusters.empty()) {
      visual_obs.obs.push_back(makeObstacle(points, {}, object_id));
      return;
    }
    for (const auto & cluster : clusters) {
      if (static_cast<int>(visual_obs.obs.size()) >= max_objects_) {
        return;
      }
      visual_obs.obs.push_back(makeObstacle(points, cluster, object_id));
    }
  }

  void loadBoundaryPolygon()
  {
    if (loadPointsFromFile(polygon_vertices_file_, boundary_polygon_) &&
      boundary_polygon_.size() >= 3)
    {
      RCLCPP_INFO(
        get_logger(), "Loaded %zu non-grass boundary polygon vertices from %s",
        boundary_polygon_.size(), polygon_vertices_file_.c_str());
      return;
    }

    if (loadPointsFromFile(boundary_points_file_, boundary_polygon_) &&
      boundary_polygon_.size() >= 3)
    {
      removeRepeatedClosingVertex(boundary_polygon_);
      RCLCPP_WARN(
        get_logger(),
        "Polygon vertices file missing; using %zu boundary points from %s as grass polygon",
        boundary_polygon_.size(), boundary_points_file_.c_str());
      return;
    }

    boundary_polygon_ = {
      {-static_cast<float>(boundary_half_size_), -static_cast<float>(boundary_half_size_), 0.0f},
      {static_cast<float>(boundary_half_size_), -static_cast<float>(boundary_half_size_), 0.0f},
      {static_cast<float>(boundary_half_size_), static_cast<float>(boundary_half_size_), 0.0f},
      {-static_cast<float>(boundary_half_size_), static_cast<float>(boundary_half_size_), 0.0f}
    };
    RCLCPP_WARN(
      get_logger(), "Failed to load boundary polygon; using fallback %.2fm square grass area",
      boundary_half_size_ * 2.0);
  }

  bool loadPointsFromFile(const std::string & file_path, std::vector<Vec3> & points) const
  {
    points.clear();
    std::ifstream file(file_path);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      Vec3 point;
      if (!(iss >> point.x >> point.y >> point.z)) {
        RCLCPP_WARN(get_logger(), "Invalid point line in %s: %s", file_path.c_str(), line.c_str());
        continue;
      }
      points.push_back(point);
    }

    return !points.empty();
  }

  void removeRepeatedClosingVertex(std::vector<Vec3> & polygon) const
  {
    if (polygon.size() < 2) {
      return;
    }

    const auto & first = polygon.front();
    const auto & last = polygon.back();
    const float dx = first.x - last.x;
    const float dy = first.y - last.y;
    const float dz = first.z - last.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.1f) {
      polygon.pop_back();
    }
  }

  void appendNonGrassPoints(std::vector<Vec3> & points)
  {
    if (!enable_non_grass_detection_ || boundary_polygon_.size() < 3) {
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_camera_transform;
    geometry_msgs::msg::TransformStamped map_to_base_transform;
    try {
      map_to_camera_transform = tf_buffer_->lookupTransform(
        camera_frame_, map_frame_, tf2::TimePointZero);
      map_to_base_transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot query TF needed by non-grass obstacle simulation: %s", ex.what());
      return;
    }

    const Vec3 mower_position_in_map{
      static_cast<float>(map_to_base_transform.transform.translation.x),
      static_cast<float>(map_to_base_transform.transform.translation.y),
      static_cast<float>(map_to_base_transform.transform.translation.z)};
    const float min_x = mower_position_in_map.x - static_cast<float>(non_grass_window_radius_);
    const float max_x = mower_position_in_map.x + static_cast<float>(non_grass_window_radius_);
    const float min_y = mower_position_in_map.y - static_cast<float>(non_grass_window_radius_);
    const float max_y = mower_position_in_map.y + static_cast<float>(non_grass_window_radius_);
    const float step = static_cast<float>(non_grass_grid_resolution_);

    for (float x = min_x; x <= max_x; x += step) {
      for (float y = min_y; y <= max_y; y += step) {
        const Vec3 map_point{x, y, 0.0f};
        if (isInsidePolygon(boundary_polygon_, map_point)) {
          continue;
        }

        const Vec3 camera_point = mapToCamera(map_point, map_to_camera_transform);
        if (!isInCameraFov(camera_point)) {
          continue;
        }
        points.push_back(camera_point);
      }
    }
  }

  Vec3 mapToCamera(
    const Vec3 & map_point,
    const geometry_msgs::msg::TransformStamped & map_to_camera_transform) const
  {
    geometry_msgs::msg::PointStamped map_point_msg;
    map_point_msg.header.frame_id = map_frame_;
    map_point_msg.header.stamp = map_to_camera_transform.header.stamp;
    map_point_msg.point = toPoint(map_point);

    geometry_msgs::msg::PointStamped camera_point_msg;
    tf2::doTransform(map_point_msg, camera_point_msg, map_to_camera_transform);

    return {
      static_cast<float>(camera_point_msg.point.x),
      static_cast<float>(camera_point_msg.point.y),
      static_cast<float>(camera_point_msg.point.z)
    };
  }

  bool isInCameraFov(const Vec3 & camera_point) const
  {
    if (camera_point.z <= 0.0f || camera_point.z > max_depth_) {
      return false;
    }

    const double distance_to_camera = std::sqrt(
      camera_point.x * camera_point.x + camera_point.y * camera_point.y +
      camera_point.z * camera_point.z);
    const double half_h = std::tan(horizontal_fov_deg_ * 0.5 / 180.0 * M_PI);
    const double half_v = std::tan(vertical_fov_deg_ * 0.5 / 180.0 * M_PI);
    return std::abs(camera_point.x / camera_point.z) <= half_h &&
           std::abs(camera_point.y / camera_point.z) <= half_v &&
           distance_to_camera <= distance_to_camera_threshold_;
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
    const std::vector<size_t> & cluster,
    uint8_t object_id) const
  {
    mower_msgs::msg::ObsInfo obstacle;
    obstacle.object_id = object_id;
    obstacle.dynamic = static_cast<uint8_t>(dynamic_);
    for (const size_t index : cluster) {
      const auto & point = points[index];
      obstacle.vertex_info.push_back(makeVertex(point.x, point.y, point.z));
    }
    if (cluster.empty()) {
      for (const auto & point : points) {
        obstacle.vertex_info.push_back(makeVertex(point.x, point.y, point.z));
      }
    }


    obstacle.sum = obstacle.vertex_info.size();

    return obstacle;
  }

  std::string point_cloud_topic_;
  std::string visual_obs_topic_;
  std::string map_frame_;
  std::string camera_frame_;
  std::string base_frame_;
  std::string boundary_points_file_;
  std::string polygon_vertices_file_;
  int sensor_id_ = 0;
  int object_id_ = kObjectIdOther;
  int dynamic_ = 0;
  double min_depth_ = 0.05;
  double max_depth_ = 2.5;
  int min_points_ = 3;
  double cluster_tolerance_ = 0.12;
  int max_objects_ = 8;
  double boundary_half_size_ = 10.0;
  bool enable_non_grass_detection_ = true;
  double non_grass_grid_resolution_ = 0.14;
  double non_grass_window_radius_ = 3.0;
  double horizontal_fov_deg_ = 90.0;
  double vertical_fov_deg_ = 70.0;
  double distance_to_camera_threshold_ = 2.0;

  std::vector<Vec3> boundary_polygon_;

  rclcpp::Publisher<mower_msgs::msg::VisualObs>::SharedPtr visual_obs_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualObstacleSimulator>());
  rclcpp::shutdown();
  return 0;
}
