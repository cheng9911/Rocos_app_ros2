

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
using namespace rocos;
DEFINE_string(urdf, "gjb_urdf2.urdf", "Urdf file path");
DEFINE_string(base, "base_link", "Base link name");
DEFINE_string(tip, "link7", "Tip link name");
// int space_joints_num=7;
bool isRuning = true;
boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(7); // 仿真
//  boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

rocos::Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

#pragma region //*测试9  完整上电保护程序

namespace rocos
{
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::cout << "---------------------------------" << std::endl;
    // 处理接收到的Joint State数据
    KDL::JntArray joints( 7 );
    for (size_t i = 0; i < msg->name.size(); ++i)
    {

      const std::string &joint_name = msg->name[i];
      double joint_position = msg->position[i];
      joints(i) = joint_position;
      RCLCPP_INFO(rclcpp::get_logger("joint_state_receiver"),
                  "Joint Name: %s, Position: %f",
                  joint_name.c_str(), joint_position);
    }
    std::cout << "---------------------------------" << std::endl;
    robot.MoveJ( joints, 0.3, 1, 0, 0, false );
    std::cout << "movej is ending" << std::endl;
  }

  void sub()
  {
    auto node1 = std::make_shared<rclcpp::Node>("joint_state_receiver");
    std::cout << "等待接收/joint_state_command话题数据" << std::endl;
    auto subscription = node1->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states_command", 20, jointStateCallback);
    rclcpp::spin(node1);
  }

  void Robot::test()
  {
    //**变量初始化 **//
    auto node = rclcpp::Node::make_shared("joint_state_publisher");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    rclcpp::WallRate loop_rate(100); // 1 Hz publishing rate

    while (true)
    {
      auto joint_state_ = sensor_msgs::msg::JointState();

      // Set joint names
      joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

      // Set joint positions
      // joint_state_.position = {0.0, 0.0, 0.0,0,0,0,0};
      // for(int i=0;i<space_joints_num;i++)
      // {
      //   joint_state_.position.push_back(getJointPosition(i));
      // }
      joint_state_.position = {getJointPosition(0), getJointPosition(1), getJointPosition(2), getJointPosition(3), getJointPosition(4), getJointPosition(5), getJointPosition(6)};
      //std::cout << "publish joint" << getJointPosition(0) << std::endl;
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

  

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  //**-------------------------------**//

  // boost::shared_ptr<HardwareInterface> hw = boost::make_shared<HardwareSim>(_joint_num); // 仿真
  // //  boost::shared_ptr< HardwareInterface > hw = boost::make_shared< Hardware >( );  //真实机械臂

  // Robot robot(hw, FLAGS_urdf, FLAGS_base, FLAGS_tip);

  auto robotService = RobotServiceImpl::getInstance(&robot);

  rclcpp::init(argc, argv);

  std::thread thread_demo(&rocos::Robot::test, &robot);
  std::thread thread_sub(&rocos::sub);
  // auto node = std::make_shared<JointStatePublisher>();

  robotService->runServer();
  thread_demo.join();
  thread_sub.join();
  //------------------------wait----------------------------------

  return 0;
}
