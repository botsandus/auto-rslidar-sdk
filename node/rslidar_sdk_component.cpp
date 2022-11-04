#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

using namespace robosense::lidar;

namespace rslidar_driver {
class RslidarSdkComponent : public rclcpp::Node {
public:
  RslidarSdkComponent(rclcpp::NodeOptions const & options)
  : Node("rslidar_sdk_component_node", options)
  {
    signal(SIGINT, signalHandler);

    std::string config_path;
    config_path = (std::string)PROJECT_PATH;
    config_path += "/config/config.yaml";

    declare_parameter("config_path", config_path);
    get_parameter("config_path", config_path);

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

  static void signalHandler(int)
  {
    rclcpp::shutdown();
    exit(0);
  }
};
}



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rslidar_driver::RslidarSdkComponent)
