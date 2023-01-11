#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>

using Twist = geometry_msgs::msg::Twist;

int getch()
{
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

int main(int argc, char **argv)
{
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("keyboard_input_node");
        rclcpp::Publisher<Twist>::SharedPtr pub;
        rclcpp::Rate loop_rate(500);

        pub = node->create_publisher<Twist>("cmd_vel", 1);

	Twist twist;

	long count = 0;

	while (rclcpp::ok())
	{
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		int ch = 0;

		ch = getch();

		printf("%ld\n", count++);
		printf("ch = %d\n\n", ch);

		switch (ch)
		{
		case 'q':
			printf("already quit!\n");
			return 0;

		case 'w':
			twist.linear.x = 0.5;
			printf("move forward!\n");
			break;

		case 's':
			twist.linear.x = -0.5;
			printf("move backward!\n");
			break;

		case 'a':
			twist.linear.y = 0.5;
			printf("move left!\n");
			break;

		case 'd':
			twist.linear.y = -0.5;
			printf("move right!\n");
			break;

		case 'j':
			twist.angular.z = 1.0;
			printf("turn left!\n");
			break;

		case 'l':
			twist.angular.z = -1.0;
			printf("turn right!\n");
			break;

		default:
			printf("Stop!\n");
			break;
		}

		pub->publish(twist);

		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	return 0;
}
