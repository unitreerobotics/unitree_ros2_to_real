#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }
};

Custom custom;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low;

long high_count = 0;
long low_count = 0;

void highCmdCallback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    custom.high_udp.SetSend(custom.high_cmd);
    custom.high_udp.Send();

    ros2_unitree_legged_msgs::msg::HighState high_state_ros;

    custom.high_udp.Recv();
    custom.high_udp.GetRecv(custom.high_state);

    high_state_ros = state2rosMsg(custom.high_state);

    pub_high->publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.low_cmd = rosMsg2Cmd(msg);

    custom.low_udp.SetSend(custom.low_cmd);
    custom.low_udp.Send();

    ros2_unitree_legged_msgs::msg::LowState low_state_ros;

    custom.low_udp.Recv();
    custom.low_udp.GetRecv(custom.low_state);

    low_state_ros = state2rosMsg(custom.low_state);

    pub_low->publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_ros2_udp");

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        printf("low level runing!\n");

        pub_low = node->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 1);
        sub_low = node->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 1, lowCmdCallback);

        rclcpp::spin(node);
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        printf("high level runing!\n");

        pub_high = node->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1);
        sub_high = node->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);

        rclcpp::spin(node);
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    rclcpp::shutdown();

    return 0;
}