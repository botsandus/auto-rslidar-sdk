#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "node_manager.hpp"

namespace robosense {
namespace lidar {

class RslidarComposableNode : public rclcpp::Node {
public:
  explicit RslidarComposableNode(const rclcpp::NodeOptions& options)
  : Node("rslidar_sdk_node", options) {
    declare_parameter<std::string>("config_path", "");
    std::string config_path = get_parameter("config_path").as_string();
    if (config_path.empty()) {
      RCLCPP_ERROR(get_logger(), "config_path parameter is empty");
      return;
    }
    YAML::Node config = YAML::LoadFile(config_path);
    // Use non-owning shared_ptr since the container owns this node's lifetime
    auto non_owning = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
    node_manager_ = std::make_shared<NodeManager>();
    node_manager_->init(config, non_owning);
    node_manager_->start();
  }

  ~RslidarComposableNode() {
    if (node_manager_) {
      node_manager_->stop();
    }
  }

private:
  std::shared_ptr<NodeManager> node_manager_;
};

}  // namespace lidar
}  // namespace robosense

RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RslidarComposableNode)
