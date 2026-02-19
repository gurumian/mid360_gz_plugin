#ifndef MID360_GZ_PLUGIN__MID360_PLUGIN_HPP_
#define MID360_GZ_PLUGIN__MID360_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

#include "mid360_gz_plugin/csv_reader.hpp"

namespace mid360_gz_plugin
{

class Mid360Plugin : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate,
                     public gz::sim::ISystemPostUpdate
{
public:
  Mid360Plugin();
  ~Mid360Plugin() override;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & event_mgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override;

private:
  /// Resolve csv path (e.g. package://mid360_gz_plugin/config/mid360.csv) to absolute path.
  std::string resolve_csv_path(const std::string & path) const;

private:
  gz::transport::Node gz_node_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::string frame_id_{"livox_mid360"};
  std::string ros_topic_{"points"};
  std::string csv_path_;
  double min_range_{0.1};
  double max_range_{200.0};
  int samples_{24000};
  int downsample_{1};

  std::vector<RayDirection> scan_directions_;
  size_t scan_index_{0};

  /// Number of rays per update (samples_ / downsample_).
  size_t num_rays_{0};

  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity parent_link_entity_{gz::sim::kNullEntity};
};

}  // namespace mid360_gz_plugin

#endif  // MID360_GZ_PLUGIN__MID360_PLUGIN_HPP_
