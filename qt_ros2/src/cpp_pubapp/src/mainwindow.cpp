#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/**/


/**/














using namespace std::chrono_literals;





MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    double joint_0 =ui->joint0->text().toDouble();
    double joint_1 =ui->joint1->text().toDouble();
    double joint_2 =ui->joint2->text().toDouble();
    double joint_3 =ui->joint3->text().toDouble();
    double joint_4 =ui->joint4->text().toDouble();
    double joint_5 =ui->joint5->text().toDouble();
    double joint_6 =ui->joint6->text().toDouble();
    std::cout<<joint_0<<","<<joint_1<<std::endl;
    
    auto node = rclcpp::Node::make_shared("joint_state_publisher");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

    rclcpp::WallRate loop_rate(2); // 1 Hz publishing rate

    for(int i=0 ; i<1;i++)
    {
      auto joint_state_ = sensor_msgs::msg::JointState();

     
      // Set joint names
      joint_state_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    

      // Set joint positions
      joint_state_.position = {joint_0,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6};
      //joint_state_.position = {getJointPosition(0), getJointPosition(1), getJointPosition(2), getJointPosition(3), getJointPosition(4), getJointPosition(5), getJointPosition(6)};
      
      // Set header timestamp
      joint_state_.header.stamp = node->get_clock()->now();
      

      publisher->publish(joint_state_);
      
      // rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    
}

