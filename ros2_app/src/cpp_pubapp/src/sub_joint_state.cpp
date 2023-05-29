#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::cout<<"---------------------------------"<<std::endl;
    // 处理接收到的Joint State数据
    for (size_t i = 0; i < msg->name.size(); ++i) {
        
        const std::string& joint_name = msg->name[i];
        double joint_position = msg->position[i];
        // std::cout<<"joint_position: "<<i<<" "<<joint_position<<std::endl;
        // double joint_velocity = msg->velocity[i];
        // double joint_effort = msg->effort[i];
        RCLCPP_INFO(rclcpp::get_logger("joint_state_receiver"), 
            "Joint Name: %s, Position: %f",
            joint_name.c_str(), joint_position );
    }
    std::cout<<"---------------------------------"<<std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joint_state_receiver");
    std::cout<<"等待接收/joint_state话题数据"<<std::endl;
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, jointStateCallback);
    rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
