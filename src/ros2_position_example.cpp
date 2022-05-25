#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <cmath>

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    auto node = rclcpp::Node::make_shared("node_ros2_postition_example");
    rclcpp::WallRate loop_rate(500);

    long motiontime = 0;

    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;

    bool initiated_flag = false; // initiate need time
    int count = 0;

    auto pub = node->create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1);

    low_cmd_ros.head[0] = 0xFE;
    low_cmd_ros.head[1] = 0xEF;
    low_cmd_ros.level_flag = LOWLEVEL;

    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motor_cmd[i].q = PosStopF; // 禁止位置环
        low_cmd_ros.motor_cmd[i].kp = 0;
        low_cmd_ros.motor_cmd[i].dq = VelStopF; // 禁止速度环
        low_cmd_ros.motor_cmd[i].kd = 0;
        low_cmd_ros.motor_cmd[i].tau = 0;
    }

    while (rclcpp::ok())
    {

        if (initiated_flag == true)
        {
            motiontime += 2;

            low_cmd_ros.motor_cmd[FR_0].tau = -0.65f;
            low_cmd_ros.motor_cmd[FL_0].tau = +0.65f;
            low_cmd_ros.motor_cmd[RR_0].tau = -0.65f;
            low_cmd_ros.motor_cmd[RL_0].tau = +0.65f;

            low_cmd_ros.motor_cmd[FR_2].q = -M_PI / 2 + 0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
            low_cmd_ros.motor_cmd[FR_2].dq = 0.0;
            low_cmd_ros.motor_cmd[FR_2].kp = 5.0;
            low_cmd_ros.motor_cmd[FR_2].kd = 1.0;

            low_cmd_ros.motor_cmd[FR_0].q = 0.0;
            low_cmd_ros.motor_cmd[FR_0].dq = 0.0;
            low_cmd_ros.motor_cmd[FR_0].kp = 5.0;
            low_cmd_ros.motor_cmd[FR_0].kd = 1.0;

            low_cmd_ros.motor_cmd[FR_1].q = 0.0;
            low_cmd_ros.motor_cmd[FR_1].dq = 0.0;
            low_cmd_ros.motor_cmd[FR_1].kp = 5.0;
            low_cmd_ros.motor_cmd[FR_1].kd = 1.0;
        }

        count++;
        if (count > 10)
        {
            count = 10;
            initiated_flag = true;
        }

        pub->publish(low_cmd_ros);

        rclcpp::spin_some(node);

        loop_rate.sleep();
    }

    return 0;
}