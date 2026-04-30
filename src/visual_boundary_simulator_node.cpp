#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mower_msgs/msg/visual_boundary_points.hpp"
#include "mower_msgs/msg/vertex_info.hpp"
#include "mower_wrapper_client/TopicName.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace
{
struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

bool isInsidePolygon(const std::vector<Vec3> & polygon, const Vec3 & point)
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const double yi = polygon[i].y;
    const double yj = polygon[j].y;
    const double xi = polygon[i].x;
    const double xj = polygon[j].x;

    if (((yi > point.y) != (yj > point.y)) &&
      (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi))
    {
      inside = !inside;
    }
  }

  return inside;
}

Vec3 enuToMap(const Vec3 & enu)
{
  // In this simulation stack, map follows ENU: x=east, y=north, z=up.
  return enu;
}

geometry_msgs::msg::Point toPoint(const Vec3 & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

mower_msgs::msg::VertexInfo toVertex(const Vec3 & point)
{
  mower_msgs::msg::VertexInfo vertex;
  vertex.x = static_cast<float>(point.x);
  vertex.y = static_cast<float>(point.y);
  vertex.z = static_cast<float>(point.z);
  return vertex;
}
}  // namespace

class VisualBoundarySimulator : public rclcpp::Node
{
public:
  VisualBoundarySimulator()
  : Node("mower_visual_boundary_simulator")
  {
    loadParameters();
    loadOrBuildBoundarySamples();

    visual_boundary_points_publisher_ =
      create_publisher<mower_msgs::msg::VisualBoundaryPoints>(TOPIC_VISUAL_BOUNDARY_POINTS, 10);
    boundary_marker_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("boundary_in_fov", 10);
    grass_marker_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>("grass_in_fov", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      [this]() {
        publishFrame();
      });

    RCLCPP_INFO(
      get_logger(),
      "visual boundary simulator publishes %s from %s with %zu boundary points",
      TOPIC_VISUAL_BOUNDARY_POINTS, odom_topic_.c_str(), boundary_points_.size());
  }

private:
  void loadParameters()
  {
    declare_parameter("odom_topic", std::string("/odom"));
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("camera_frame", std::string("camera_link"));
    declare_parameter("publish_period_ms", 100);
    declare_parameter("boundary_half_size", 10.0);
    declare_parameter("boundary_sample_step", 0.08);
    declare_parameter("boundary_points_file", std::string("/home/chensi/boundary_points.txt"));
    declare_parameter(
      "boundary_tangential_file",
      std::string("/home/chensi/boundary_tangential_vectors.txt"));
    declare_parameter("polygon_vertices_file", std::string("/home/chensi/polygon_vertices.txt"));
    declare_parameter("grass_grid_resolution", 0.14);
    declare_parameter("grass_window_radius", 3.0);
    declare_parameter("max_depth", 2.0);
    declare_parameter("horizontal_fov_deg", 90.0);
    declare_parameter("vertical_fov_deg", 70.0);
    declare_parameter("noise_sigma", 0.025);
    declare_parameter("boundary_dropout_ratio", 0.5);
    declare_parameter("publish_debug_markers", true);
    declare_parameter("distance_to_camera_threshold", 1.4);

    get_parameter("odom_topic", odom_topic_);
    get_parameter("map_frame", map_frame_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("publish_period_ms", publish_period_ms_);
    get_parameter("boundary_half_size", boundary_half_size_);
    get_parameter("boundary_sample_step", boundary_sample_step_);
    get_parameter("boundary_points_file", boundary_points_file_);
    get_parameter("boundary_tangential_file", boundary_tangential_file_);
    get_parameter("polygon_vertices_file", polygon_vertices_file_);
    get_parameter("grass_grid_resolution", grass_grid_resolution_);
    get_parameter("grass_window_radius", grass_window_radius_);
    get_parameter("max_depth", max_depth_);
    get_parameter("horizontal_fov_deg", horizontal_fov_deg_);
    get_parameter("vertical_fov_deg", vertical_fov_deg_);
    get_parameter("noise_sigma", noise_sigma_);
    get_parameter("boundary_dropout_ratio", boundary_dropout_ratio_);
    get_parameter("publish_debug_markers", publish_debug_markers_);
    get_parameter("distance_to_camera_threshold", distance_to_camera_threshold_);

    noise_dist_ = std::normal_distribution<double>(0.0, noise_sigma_);
  }

  void loadOrBuildBoundarySamples()
  {
    std::vector<Vec3> loaded_boundary_points;
    std::vector<Vec3> loaded_tangential_vectors;
    std::vector<Vec3> loaded_polygon_vertices;

    const bool loaded_boundary = loadPointsFromFile(boundary_points_file_, loaded_boundary_points);
    const bool loaded_tangential =
      loadPointsFromFile(boundary_tangential_file_, loaded_tangential_vectors);
    const bool loaded_polygon = loadPointsFromFile(polygon_vertices_file_, loaded_polygon_vertices);

    if (loaded_boundary && loaded_boundary_points.size() >= 3) {
      boundary_points_ = loaded_boundary_points;
      RCLCPP_INFO(
        get_logger(), "Loaded %zu boundary points from %s",
        boundary_points_.size(), boundary_points_file_.c_str());

      if (loaded_tangential && loaded_tangential_vectors.size() == boundary_points_.size()) {
        boundary_tangential_vectors_ = loaded_tangential_vectors;
        RCLCPP_INFO(
          get_logger(), "Loaded %zu boundary tangential vectors from %s",
          boundary_tangential_vectors_.size(), boundary_tangential_file_.c_str());
      } else {
        boundary_tangential_vectors_ = computeTangentialVectors(boundary_points_);
        RCLCPP_WARN(
          get_logger(),
          "Boundary tangential file missing or size mismatch; computed %zu tangential vectors",
          boundary_tangential_vectors_.size());
      }

      if (loaded_polygon && loaded_polygon_vertices.size() >= 3) {
        boundary_polygon_ = loaded_polygon_vertices;
        RCLCPP_INFO(
          get_logger(), "Loaded %zu polygon vertices from %s",
          boundary_polygon_.size(), polygon_vertices_file_.c_str());
      } else {
        boundary_polygon_ = buildPolygonFromBoundary(boundary_points_);
        RCLCPP_WARN(
          get_logger(), "Polygon vertices file missing; using boundary points as grass polygon");
      }
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Failed to load boundary points from %s; using fallback square boundary",
      boundary_points_file_.c_str());
    buildFallbackBoundarySamples();
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
      Vec3 point_map = enuToMap(point);
      points.push_back(point_map);
    }

    return !points.empty();
  }

  std::vector<Vec3> computeTangentialVectors(const std::vector<Vec3> & points) const
  {
    std::vector<Vec3> tangential_vectors;
    tangential_vectors.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
      const auto & current = points[i];
      const auto & next = points[(i + 1) % points.size()];
      const double dx = next.x - current.x;
      const double dy = next.y - current.y;
      const double dz = next.z - current.z;
      const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (length > 1e-6) {
        tangential_vectors.push_back({dx / length, dy / length, dz / length});
      } else if (!tangential_vectors.empty()) {
        tangential_vectors.push_back(tangential_vectors.back());
      } else {
        tangential_vectors.push_back({1.0, 0.0, 0.0});
      }
    }

    return tangential_vectors;
  }

  std::vector<Vec3> buildPolygonFromBoundary(const std::vector<Vec3> & points) const
  {
    std::vector<Vec3> polygon = points;
    if (polygon.size() >= 2) {
      const auto & first = polygon.front();
      const auto & last = polygon.back();
      const double dx = first.x - last.x;
      const double dy = first.y - last.y;
      const double dz = first.z - last.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < 0.1) {
        polygon.pop_back();
      }
    }
    return polygon;
  }

  void buildFallbackBoundarySamples()
  {
    boundary_polygon_ = {
      {-boundary_half_size_, -boundary_half_size_, 0.0},
      {boundary_half_size_, -boundary_half_size_, 0.0},
      {boundary_half_size_, boundary_half_size_, 0.0},
      {-boundary_half_size_, boundary_half_size_, 0.0}
    };

    boundary_points_.clear();
    for (size_t i = 0; i < boundary_polygon_.size(); ++i) {
      const auto & start = boundary_polygon_[i];
      const auto & end = boundary_polygon_[(i + 1) % boundary_polygon_.size()];
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      const double length = std::hypot(dx, dy);
      const int steps = std::max(1, static_cast<int>(std::ceil(length / boundary_sample_step_)));
      for (int step = 0; step < steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        boundary_points_.push_back({start.x + dx * t, start.y + dy * t, 0.0});
      }
    }
    boundary_tangential_vectors_ = computeTangentialVectors(boundary_points_);
  }

  void publishFrame()
  {
    if (!has_odom_) {
      return;
    }

    geometry_msgs::msg::TransformStamped map_to_camera_transform;
    geometry_msgs::msg::TransformStamped map_to_base_transform;
    try {
      map_to_camera_transform = tf_buffer_->lookupTransform(
        camera_frame_, map_frame_, tf2::TimePointZero);
      map_to_base_transform = tf_buffer_->lookupTransform(
        map_frame_, "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot query TF needed by visual_boundary_simulator: %s", ex.what());
      return;
    }

    const Vec3 mower_position_in_map{
      map_to_base_transform.transform.translation.x,
      map_to_base_transform.transform.translation.y,
      map_to_base_transform.transform.translation.z};

    mower_msgs::msg::VisualBoundaryPoints visual_points;
    visual_points.timestamp = now().nanoseconds() / 1000000;

    auto boundary_marker = makeMarker("boundary_in_fov", 0, 1.0, 0.0, 0.0);
    auto grass_marker = makeMarker("grass_in_fov", 0, 0.0, 1.0, 0.0);

    appendBoundaryPoints(visual_points, boundary_marker, map_to_camera_transform);
    appendGrassPoints(
      visual_points, grass_marker, map_to_camera_transform, mower_position_in_map);

    visual_points.boundary_num = static_cast<uint16_t>(
      std::min<size_t>(visual_points.points.size(), std::numeric_limits<uint16_t>::max()));
    visual_points.grass_num = static_cast<uint16_t>(
      std::min<size_t>(visual_points.grass_points.size(), std::numeric_limits<uint16_t>::max()));

    visual_boundary_points_publisher_->publish(visual_points);

    if (publish_debug_markers_) {
      boundary_marker_publisher_->publish(boundary_marker);
      grass_marker_publisher_->publish(grass_marker);
    }
  }

  visualization_msgs::msg::Marker makeMarker(
    const std::string & name, int id, float r, float g, float b) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = camera_frame_;
    marker.header.stamp = now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
  }

  void appendBoundaryPoints(
    mower_msgs::msg::VisualBoundaryPoints & visual_points,
    visualization_msgs::msg::Marker & marker,
    const geometry_msgs::msg::TransformStamped & map_to_camera_transform)
  {
    for (size_t i = 0; i < boundary_points_.size(); ++i) {
      const auto & map_point = boundary_points_[i];
      auto camera_point = mapToCamera(map_point, map_to_camera_transform);
      if (!isInCameraFov(camera_point)) {
        continue;
      }

      if (dropout_dist_(rand_engine_) < boundary_dropout_ratio_) {
        continue;
      }

      addBoundaryNoise(camera_point, i, map_to_camera_transform);
      visual_points.points.push_back(toVertex(camera_point));
      marker.points.push_back(toPoint(camera_point));
    }
  }

  void appendGrassPoints(
    mower_msgs::msg::VisualBoundaryPoints & visual_points,
    visualization_msgs::msg::Marker & marker,
    const geometry_msgs::msg::TransformStamped & map_to_camera_transform,
    const Vec3 & mower_position_in_map)
  {
    const double min_x = mower_position_in_map.x - grass_window_radius_;
    const double max_x = mower_position_in_map.x + grass_window_radius_;
    const double min_y = mower_position_in_map.y - grass_window_radius_;
    const double max_y = mower_position_in_map.y + grass_window_radius_;

    for (double x = min_x; x <= max_x; x += grass_grid_resolution_) {
      for (double y = min_y; y <= max_y; y += grass_grid_resolution_) {
        const Vec3 map_point{x, y, 0.0};
        if (!isInsidePolygon(boundary_polygon_, map_point)) {
          continue;
        }

        auto camera_point = mapToCamera(map_point, map_to_camera_transform);
        if (!isInCameraFov(camera_point)) {
          continue;
        }

        addNoise(camera_point);
        visual_points.grass_points.push_back(toVertex(camera_point));
        marker.points.push_back(toPoint(camera_point));
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
      camera_point_msg.point.x,
      camera_point_msg.point.y,
      camera_point_msg.point.z
    };
  }

  bool isInCameraFov(const Vec3 & camera_point) const
  {
    if (camera_point.z <= 0.0 || camera_point.z > max_depth_) {
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

  void addNoise(Vec3 & point)
  {
    point.x += noise_dist_(rand_engine_);
    point.y += noise_dist_(rand_engine_);
    point.z += noise_dist_(rand_engine_);
  }

  void addBoundaryNoise(
    Vec3 & point,
    size_t index,
    const geometry_msgs::msg::TransformStamped & map_to_camera_transform)
  {
    if (index >= boundary_tangential_vectors_.size()) {
      addNoise(point);
      return;
    }

    auto tangent = mapVectorToCamera(boundary_tangential_vectors_[index], map_to_camera_transform);
    const double tangent_norm = std::sqrt(
      tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
    if (tangent_norm < 1e-6) {
      addNoise(point);
      return;
    }

    tangent.x /= tangent_norm;
    tangent.y /= tangent_norm;
    tangent.z /= tangent_norm;

    Vec3 normal{-tangent.y, tangent.x, 0.0};
    const double normal_norm = std::sqrt(normal.x * normal.x + normal.y * normal.y);
    if (normal_norm > 1e-6) {
      normal.x /= normal_norm;
      normal.y /= normal_norm;
    }

    const double normal_noise = noise_dist_(rand_engine_);
    const double tangent_noise = noise_dist_(rand_engine_);
    point.x += normal.x * normal_noise + tangent.x * tangent_noise;
    point.y += normal.y * normal_noise + tangent.y * tangent_noise;
    point.z += normal.z * normal_noise + tangent.z * tangent_noise;
  }

  Vec3 mapVectorToCamera(
    const Vec3 & map_vector,
    const geometry_msgs::msg::TransformStamped & map_to_camera_transform) const
  {
    geometry_msgs::msg::Vector3Stamped map_vector_msg;
    map_vector_msg.header.frame_id = map_frame_;
    map_vector_msg.header.stamp = map_to_camera_transform.header.stamp;
    map_vector_msg.vector.x = map_vector.x;
    map_vector_msg.vector.y = map_vector.y;
    map_vector_msg.vector.z = map_vector.z;

    geometry_msgs::msg::Vector3Stamped camera_vector_msg;
    tf2::doTransform(map_vector_msg, camera_vector_msg, map_to_camera_transform);

    return {
      camera_vector_msg.vector.x,
      camera_vector_msg.vector.y,
      camera_vector_msg.vector.z
    };
  }

  std::string odom_topic_;
  std::string map_frame_;
  std::string camera_frame_;
  std::string boundary_points_file_;
  std::string boundary_tangential_file_;
  std::string polygon_vertices_file_;

  int publish_period_ms_ = 100;
  double boundary_half_size_ = 10.0;
  double boundary_sample_step_ = 0.08;
  double grass_grid_resolution_ = 0.14;
  double grass_window_radius_ = 3.0;
  double max_depth_ = 2.0;
  double horizontal_fov_deg_ = 90.0;
  double vertical_fov_deg_ = 70.0;
  double noise_sigma_ = 0.025;
  double boundary_dropout_ratio_ = 0.5;
  bool publish_debug_markers_ = true;
  double distance_to_camera_threshold_ = 1.4;

  std::vector<Vec3> boundary_polygon_;
  std::vector<Vec3> boundary_points_;
  std::vector<Vec3> boundary_tangential_vectors_;

  bool has_odom_ = false;
  nav_msgs::msg::Odometry latest_odom_;

  std::default_random_engine rand_engine_{std::random_device{}()};
  std::normal_distribution<double> noise_dist_{0.0, 0.025};
  std::uniform_real_distribution<double> dropout_dist_{0.0, 1.0};

  rclcpp::Publisher<mower_msgs::msg::VisualBoundaryPoints>::SharedPtr
    visual_boundary_points_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grass_marker_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualBoundarySimulator>());
  rclcpp::shutdown();
  return 0;
}
