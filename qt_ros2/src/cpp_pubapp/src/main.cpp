#include "mainwindow.h"

#include <QApplication>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    rclcpp::init(argc, argv);
    std::cout<<"hello,world"<<std::endl;
    w.show();
    
    
    

    return a.exec();
}
