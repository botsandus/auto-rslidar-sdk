#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

using namespace robosense::lidar;

namespace rslidar_driver {
class RslidarSdkComposableNode : public rclcpp::Node {
public:
  RslidarSdkComposableNode(rclcpp::NodeOptions const & options)
  : Node("rslidar_sdk_composable_node", options)
  {
    std::string config_path;
    config_path = (std::string)PROJECT_PATH;
    config_path += "/config/config.yaml";

    this->declare_parameter("config_path", config_path);
    this->get_parameter("config_path", config_path);

    RS_MSG << "config_path " << config_path << RS_REND;
    YAML::Node config;
    try
    {
      config = YAML::LoadFile(config_path);
    }
    catch (...)
    {
      RS_ERROR << "The format of config file " << config_path
        << " is wrong. Please check (e.g. indentation)." << RS_REND;
      throw rclcpp::exceptions::InvalidParameterTypeException("config_path", config_path);
    }

    demo_ptr = std::make_shared<NodeManager>();
    demo_ptr->init(config);
    demo_ptr->start();

    RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;
  }

  std::shared_ptr<NodeManager> demo_ptr;
};
}

static void signalHandler(int)
{
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rslidar_driver::RslidarSdkComposableNode>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
