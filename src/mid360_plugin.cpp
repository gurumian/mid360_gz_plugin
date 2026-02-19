#include "mid360_gz_plugin/mid360_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/common/Console.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <string>

namespace mid360_gz_plugin
{

Mid360Plugin::Mid360Plugin() = default;

Mid360Plugin::~Mid360Plugin() = default;

std::string Mid360Plugin::resolve_csv_path(const std::string & path) const
{
  const std::string prefix("package://");
  if (path.size() < prefix.size() || path.compare(0, prefix.size(), prefix) != 0) {
    return path;
  }
  std::string rest = path.substr(prefix.size());
  size_t slash = rest.find('/');
  if (slash == std::string::npos) {
    return path;
  }
  std::string pkg = rest.substr(0, slash);
  std::string subpath = rest.substr(slash + 1);
  try {
    std::string share = ament_index_cpp::get_package_share_directory(pkg);
    return share + "/" + subpath;
  } catch (const std::exception & e) {
    gzerr << "[Mid360Plugin] resolve_csv_path failed for " << path << ": " << e.what() << std::endl;
    return path;
  }
}

void Mid360Plugin::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*event_mgr*/)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  ros_node_ = std::make_shared<rclcpp::Node>("mid360_gz_plugin_node");

  if (sdf->HasElement("frame_id")) {
    frame_id_ = sdf->Get<std::string>("frame_id");
  }
  if (sdf->HasElement("ros_topic")) {
    ros_topic_ = sdf->Get<std::string>("ros_topic");
  }
  if (sdf->HasElement("csv_file")) {
    csv_path_ = sdf->Get<std::string>("csv_file");
  }
  if (sdf->HasElement("min_range")) {
    min_range_ = sdf->Get<double>("min_range");
  }
  if (sdf->HasElement("max_range")) {
    max_range_ = sdf->Get<double>("max_range");
  }
  if (sdf->HasElement("samples")) {
    samples_ = sdf->Get<int>("samples");
  }
  if (sdf->HasElement("downsample")) {
    downsample_ = sdf->Get<int>("downsample");
    if (downsample_ < 1) {
      downsample_ = 1;
    }
  }

  if (csv_path_.empty()) {
    gzerr << "[Mid360Plugin] Missing <csv_file>. Cannot load scan pattern." << std::endl;
    return;
  }

  std::string resolved = resolve_csv_path(csv_path_);
  if (!read_scan_csv(resolved, scan_directions_)) {
    gzerr << "[Mid360Plugin] Failed to load CSV: " << resolved << std::endl;
    return;
  }
  gzmsg << "[Mid360Plugin] Loaded " << scan_directions_.size() << " scan directions from " << resolved
        << std::endl;

  cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_, 10);

  sensor_entity_ = entity;
  if (ecm.HasComponent<gz::sim::components::ParentEntity>(entity)) {
    parent_link_entity_ = ecm.Component<gz::sim::components::ParentEntity>(entity)->Data();
  }

  num_rays_ = static_cast<size_t>(samples_ / downsample_);
  if (num_rays_ == 0) {
    num_rays_ = 1;
  }

  gz::sim::components::RaycastDataInfo raycast_info;
  raycast_info.rays.resize(num_rays_);
  raycast_info.results.resize(num_rays_);
  for (size_t i = 0; i < num_rays_; ++i) {
    raycast_info.rays[i].start = gz::math::Vector3d::Zero;
    raycast_info.rays[i].end = gz::math::Vector3d::Zero;
    raycast_info.results[i].point = gz::math::Vector3d::Zero;
    raycast_info.results[i].fraction = 1.0;
    raycast_info.results[i].normal = gz::math::Vector3d::Zero;
  }
  ecm.CreateComponent(sensor_entity_, gz::sim::components::RaycastData(raycast_info));

  RCLCPP_INFO(ros_node_->get_logger(), "Mid360Plugin configured. Publishing to %s", ros_topic_.c_str());
}

void Mid360Plugin::PreUpdate(
  const gz::sim::UpdateInfo & /*info*/,
  gz::sim::EntityComponentManager & ecm)
{
  if (scan_directions_.empty() || num_rays_ == 0) {
    return;
  }

  auto * comp = ecm.Component<gz::sim::components::RaycastData>(sensor_entity_);
  if (!comp) {
    return;
  }

  gz::sim::components::RaycastDataInfo & data = comp->Data();
  if (data.rays.size() != num_rays_ || data.results.size() != num_rays_) {
    return;
  }

  const size_t num_dirs = scan_directions_.size();
  for (size_t i = 0; i < num_rays_; ++i) {
    size_t idx = (scan_index_ + i * downsample_) % num_dirs;
    const auto & dir = scan_directions_[idx];
    double az = dir.first;
    double ze = dir.second;
    gz::math::Quaterniond ray_rot;
    ray_rot.Euler(0.0, ze, az);
    gz::math::Vector3d axis = ray_rot * gz::math::Vector3d(1.0, 0.0, 0.0);

    data.rays[i].start = axis * min_range_;
    data.rays[i].end = axis * max_range_;
  }
}

void Mid360Plugin::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  if (scan_directions_.empty() || !cloud_pub_ || num_rays_ == 0) {
    return;
  }
  if (info.paused) {
    return;
  }

  const auto * comp = ecm.Component<gz::sim::components::RaycastData>(sensor_entity_);
  if (!comp) {
    return;
  }

  const gz::sim::components::RaycastDataInfo & data = comp->Data();
  if (data.rays.size() != num_rays_ || data.results.size() != num_rays_) {
    return;
  }

  const size_t num_dirs = scan_directions_.size();

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = static_cast<int32_t>(info.simTime.count() / 1'000'000'000);
  msg.header.stamp.nanosec = static_cast<uint32_t>(info.simTime.count() % 1'000'000'000);
  msg.header.frame_id = frame_id_;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(num_rays_);
  msg.is_dense = true;
  msg.point_step = 16u;
  msg.row_step = msg.point_step * msg.width;

  msg.fields.resize(4);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;

  msg.data.resize(msg.row_step * msg.height);

  float * ptr = reinterpret_cast<float *>(msg.data.data());

  for (size_t i = 0; i < num_rays_; ++i) {
    const auto & result = data.results[i];
    double fraction = result.fraction;
    gz::math::Vector3d point_link;
    float intensity = 0.0f;

    if (fraction > 0.0 && fraction < 1.0) {
      point_link = result.point;
      intensity = static_cast<float>(1.0 - fraction);
    } else {
      size_t idx = (scan_index_ + i * downsample_) % num_dirs;
      const auto & dir = scan_directions_[idx];
      double az = dir.first;
      double ze = dir.second;
      gz::math::Quaterniond ray_rot;
      ray_rot.Euler(0.0, ze, az);
      gz::math::Vector3d axis = ray_rot * gz::math::Vector3d(1.0, 0.0, 0.0);
      point_link = axis * max_range_;
    }

    *ptr++ = static_cast<float>(point_link.X());
    *ptr++ = static_cast<float>(point_link.Y());
    *ptr++ = static_cast<float>(point_link.Z());
    *ptr++ = intensity;
  }

  scan_index_ = (scan_index_ + samples_) % num_dirs;
  cloud_pub_->publish(msg);
}

}  // namespace mid360_gz_plugin

GZ_ADD_PLUGIN(
  mid360_gz_plugin::Mid360Plugin,
  gz::sim::System,
  mid360_gz_plugin::Mid360Plugin::ISystemConfigure,
  mid360_gz_plugin::Mid360Plugin::ISystemPreUpdate,
  mid360_gz_plugin::Mid360Plugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(mid360_gz_plugin::Mid360Plugin, "mid360_gz_plugin")
