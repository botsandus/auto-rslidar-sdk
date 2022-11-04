/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>

using namespace robosense::lidar;

namespace rslidar_driver {
class RslidarSdkComposableNode : public rclcpp::Node {
public:
  RslidarSdkComposableNode()
  : Node("rslidar_sdk_composable_node")
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
  auto node = std::make_shared<rslidar_driver::RslidarSdkComposableNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
