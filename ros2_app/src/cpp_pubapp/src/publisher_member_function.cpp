// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <csignal>
#include <cstdio>
#include <cstdlib>

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>

DEFINE_string(urdf, "robot.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link_7", "Tip link name");

bool isRuning = true;

#pragma region //*测试9  完整上电保护程序

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

namespace rocos
{

  void Robot::test()
  {
    //**变量初始化 **//
    std::string str{""};
    std::ifstream csv_null_motion;

    char tem[2048];
    std::vector<std::string> tokens;
    std::vector<KDL::JntArray> servo_data;
    KDL::JntArray joints(_joint_num);
    KDL::JntArray last_joints(_joint_num);

    int row_index = 1;

    auto t_start = std::chrono::high_resolution_clock::now(); // 记录程序启动时间

    //**-------------------------------**//
  }
} // namespace rocos

#pragma endregion

/// \brief 处理终端的Ctrl-C信号
/// \param signo
void signalHandler(int signo)
{
  if (signo == SIGINT)
  {
    std::cout << "\033[1;31m"
              << "[!!SIGNAL!!]"
              << "INTERRUPT by CTRL-C"
              << "\033[0m" << std::endl;

    isRuning = false;
    exit(0);
  }
}

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

int main(int argc, char *argv[])
{
  if (signal(SIGINT, signalHandler) == SIG_ERR)
  {
    std::cout << "\033[1;31m"
              << "Can not catch SIGINT"
              << "\033[0m" << std::endl;
  }

  using namespace rocos;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  //**-------------------------------**//

  boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
  //  boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

  Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

  auto robotService = RobotServiceImpl::getInstance(&robot);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();



  //------------------------wait----------------------------------
  robotService->runServer();

  return 0;
}
