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

DEFINE_string(urdf, "gjb_urdf2.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link7", "Tip link name");

bool isRuning = true;

#pragma region //*测试9  完整上电保护程序

// class JointStatePublisher : public rclcpp::Node
// {
// public:
//   JointStatePublisher() : Node("joint_state_publisher")
//   {
//     // 初始化发布者
//     publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

//     // 创建JointState消息

//     joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
//     joint_state_.position = {0.0, 0.0, 0.0};
//     //std::cout<<"publish joint"<<dof_7->getJointPosition(0)<<std::endl;

//     //joint_state_.position = {dof_7->getJointPosition(0), dof_7->getJointPosition(1), dof_7->getJointPosition(2), dof_7->getJointPosition(3), dof_7->getJointPosition(4), dof_7->getJointPosition(5), dof_7->getJointPosition(6)};
//     std::cout<<"publish joint"<<std::endl;
//     // 创建定时器，定时发布JointState消息
//     timer_ = create_wall_timer(std::chrono::milliseconds(100),
//                                std::bind(&JointStatePublisher::publishJointState, this));
//   }
//   rocos::Robot *dof_7;

// private:
//   void publishJointState()
//   {
//     // 更新JointState消息的时间戳
//     joint_state_.header.stamp = now();

//     // 发布JointState消息
//     publisher_->publish(joint_state_);
//   }

//   sensor_msgs::msg::JointState joint_state_;

//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

namespace rocos
{


  void Robot::test()
  {
    //**变量初始化 **//
    auto node = rclcpp::Node::make_shared("joint_state_publisher");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    rclcpp::WallRate loop_rate(500); // 1 Hz publishing rate

    while (true)
    {
      auto joint_state_ = sensor_msgs::msg::JointState();

     
      // Set joint names
      joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    

      // Set joint positions
      //joint_state_.position = {0.0, 0.0, 0.0,0,0,0,0};
      joint_state_.position = {getJointPosition(0), getJointPosition(1), getJointPosition(2), getJointPosition(3), getJointPosition(4), getJointPosition(5), getJointPosition(6)};
      std::cout << "publish joint" << getJointPosition(0) << std::endl;
      // Set header timestamp
      joint_state_.header.stamp = node->get_clock()->now();
      

      publisher->publish(joint_state_);
      
      // rclcpp::spin_some(node);
      loop_rate.sleep();
    }

    // std::cout << "publish joint" << getJointPosition(0) << std::endl;
    // auto node = std::make_shared<JointStatePublisher>();
    // rclcpp::spin(node);
    rclcpp::shutdown();

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

  std::thread thread_demo(&rocos::Robot::test, &robot);

  // auto node = std::make_shared<JointStatePublisher>();












  
  robotService->runServer();
  thread_demo.join();

  //------------------------wait----------------------------------

  return 0;
}
