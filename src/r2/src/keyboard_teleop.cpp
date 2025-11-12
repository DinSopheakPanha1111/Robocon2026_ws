#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>

class KeyboardSpeedController : public rclcpp::Node
{
public:
    KeyboardSpeedController()
    : Node("keyboard_speed_controller"), lin_x_(0.0), lin_y_(0.0), ang_z_(0.0)
    {
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mecanum_drive_controller/reference", 10);

        step_linear_ = 0.05;
        step_angular_ = 0.1;
        max_linear_ = 0.6;
        max_angular_ = 1.0;

        RCLCPP_INFO(this->get_logger(),
            "\nControl mecanum robot with keyboard:\n"
            "   W = forward (X+)\n"
            "   S = backward (X-)\n"
            "   Q = strafe left (Y-)\n"
            "   E = strafe right (Y+)\n"
            "   A = rotate left (Z+)\n"
            "   D = rotate right (Z-)\n"
            "   SPACE = stop\n"
            "   X = quit\n");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyboardSpeedController::publish_cmd, this));
    }

    void run()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok())
        {
            char c = 0;
            if (kbhit())
            {
                read(STDIN_FILENO, &c, 1);
                c = std::tolower(c);

                if (c == 'x')
                {
                    RCLCPP_INFO(this->get_logger(), "Exit pressed, stopping robot.");
                    break;
                }
                else if (c == 'w')
                {
                    lin_x_ = std::min(lin_x_ + step_linear_, max_linear_);
                    RCLCPP_INFO(this->get_logger(), "Forward: X=%.2f", lin_x_);
                }
                else if (c == 's')
                {
                    lin_x_ = std::max(lin_x_ - step_linear_, -max_linear_);
                    RCLCPP_INFO(this->get_logger(), "Backward: X=%.2f", lin_x_);
                }
                else if (c == 'e')
                {
                    lin_y_ = std::min(lin_y_ + step_linear_, max_linear_);
                    RCLCPP_INFO(this->get_logger(), "Right strafe: Y=%.2f", lin_y_);
                }
                else if (c == 'q')
                {
                    lin_y_ = std::max(lin_y_ - step_linear_, -max_linear_);
                    RCLCPP_INFO(this->get_logger(), "Left strafe: Y=%.2f", lin_y_);
                }
                else if (c == 'a')
                {
                    ang_z_ = std::min(ang_z_ + step_angular_, max_angular_);
                    RCLCPP_INFO(this->get_logger(), "Rotate left: Z=%.2f", ang_z_);
                }
                else if (c == 'd')
                {
                    ang_z_ = std::max(ang_z_ - step_angular_, -max_angular_);
                    RCLCPP_INFO(this->get_logger(), "Rotate right: Z=%.2f", ang_z_);
                }
                else if (c == ' ')
                {
                    lin_x_ = 0.0;
                    lin_y_ = 0.0;
                    ang_z_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Stop");
                }
            }

            rclcpp::spin_some(shared_from_this());
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        // Stop robot before exiting
        geometry_msgs::msg::TwistStamped stop;
        stop.header.stamp = now();
        pub_->publish(stop);
    }

private:
    bool kbhit()
    {
        struct timeval tv{0L, 0L};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    }

    void publish_cmd()
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = now();
        msg.twist.linear.x = lin_x_;
        msg.twist.linear.y = lin_y_;
        msg.twist.angular.z = ang_z_;
        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double lin_x_, lin_y_, ang_z_;
    double step_linear_, step_angular_;
    double max_linear_, max_angular_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardSpeedController>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
