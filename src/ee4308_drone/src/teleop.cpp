#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ee4308_lib/core.hpp"


namespace ee4308::drone
{
    struct TeleopParameters
    {
        struct Topics
        {
            std::string cmd_vel = "cmd_vel";
            std::string takeoff = "takeoff";
            std::string land = "land";
        } topics;

        double linear_step = 0.1;
        double angular_step = 0.1;
        double linear_step_size = 0.01;
        double angular_step_size = 0.01;
    };

    class ROSNodeTeleop : public rclcpp::Node
    {
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_; 
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;      
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;         
        rclcpp::TimerBase::SharedPtr looper_;                                 
        geometry_msgs::msg::Twist msg_cmd_vel_;
        TeleopParameters params_;
        double linear_max = 1.0;  // from sjtu drone teleop
        double angular_max = 1.0; // from sjtu drone teleop

    public:
        explicit ROSNodeTeleop(
            const std::string &name = "teleop_drone")
            : Node(name)
        {
            initParams();
            initTopics();

            std::cout << "=================== Controls ====================" << std::endl;
            std::cout << "1: Request takeoff" << std::endl; 
            std::cout << "2: Request land" << std::endl; 
            std::cout << "s: Stop / hover" << std::endl; 
            std::cout << "w:  Move forward: Increase x-axis linear velocity" << std::endl; 
            std::cout << "x: Move backward: Decrease x-axis linear velocity" << std::endl; 
            std::cout << "a:   Strafe left: Increase y-axis linear velocity" << std::endl; 
            std::cout << "d:  Strafe right: Decrease y-axis linear velocity" << std::endl; 
            std::cout << "q:      Yaw left: Increase z-axis angular velocity" << std::endl; 
            std::cout << "e:     Yaw right: Decrease z-axis angular velocity" << std::endl; 
            std::cout << "r:          Rise: Increase z-axis linear velocity" << std::endl; 
            std::cout << "f:          Fall: Decrease z-axis linear velocity" << std::endl; 
            std::cout << "W: Increase linear velocity increment" << std::endl; 
            std::cout << "X: Decrease linear velocity increment" << std::endl; 
            std::cout << "Q: Increase angular velocity increment" << std::endl; 
            std::cout << "E: Decrease angular velocity increment" << std::endl; 
            std::cout << "==================================================" << std::endl;

            looper_ = this->create_wall_timer(
                50ms, 
                std::bind(&ROSNodeTeleop::loop, this));
        }

    private:
        void initTopics()
        {
            pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(params_.topics.cmd_vel, 1);
            pub_takeoff_ = this->create_publisher<std_msgs::msg::Empty>(params_.topics.takeoff, 1);
            pub_land_ = this->create_publisher<std_msgs::msg::Empty>(params_.topics.land, 1);
        }

        void initParams()
        {
            initParam(this, "topics.cmd_vel", params_.topics.cmd_vel);

            initParam(this, "topics.takeoff", params_.topics.takeoff);
            
            initParam(this, "topics.land", params_.topics.land);

            initParam(this, "linear_step", params_.linear_step);

            initParam(this, "angular_step", params_.angular_step);

            initParam(this, "linear_step_size", params_.linear_step_size);

            initParam(this, "angular_step_size", params_.angular_step_size);
        }

        void loop()
        {
            auto constrain = [](double &value, const double increment, const double limit) -> void { // limit is positive.
                value += increment;
                if (increment < 0 && value < -limit)
                    value = -limit;
                else if (increment > 0 && value > limit)
                    value = limit;
            };

            char c;

            if (getch(c) == true || rclcpp::ok() == false)
            {
                std::cout << std::endl;
                // std::cout << "Terminated / Exception" << std::endl;
                rclcpp::shutdown();
                return;
            }

            switch (c)
            {
            case 'w': // forward
                constrain(msg_cmd_vel_.linear.x, params_.linear_step, linear_max);
                break;

            case 's': // stop
                msg_cmd_vel_ = geometry_msgs::msg::Twist();
                break;

            case 'a': // strafe left
                constrain(msg_cmd_vel_.linear.y, params_.linear_step, linear_max);
                break;

            case 'd': // strafe right
                constrain(msg_cmd_vel_.linear.y, -params_.linear_step, linear_max);
                break;

            case 'x': // reverse
                constrain(msg_cmd_vel_.linear.x, -params_.linear_step, linear_max);
                break;

            case 'q': // yaw left
                constrain(msg_cmd_vel_.angular.z, params_.angular_step, angular_max);
                break;

            case 'e': // yaw right
                constrain(msg_cmd_vel_.angular.z, -params_.angular_step, angular_max);
                break;

            case 'r': // rise
                constrain(msg_cmd_vel_.linear.z, params_.linear_step, linear_max);
                break;

            case 'f': // fall
                constrain(msg_cmd_vel_.linear.z, -params_.linear_step, linear_max);
                break;

            case '1': // takeoff
                std::cout << std::endl
                          << "Taking Off" << std::endl;
                pub_takeoff_->publish(std_msgs::msg::Empty());
                break;

            case '2': // land
                std::cout << std::endl
                          << "Landing" << std::endl;
                pub_land_->publish(std_msgs::msg::Empty());
                break;

            case 'W': // increase horizontal step
                constrain(params_.linear_step, params_.linear_step_size, linear_max);
                break;

            case 'X': // decrease horizontal step
                constrain(params_.linear_step, -params_.linear_step_size, params_.linear_step_size);
                break;

            case 'Q': // increase angular step
                constrain(params_.angular_step, params_.angular_step_size, angular_max);
                break;

            case 'E': // decrease angular step
                constrain(params_.angular_step, -params_.angular_step_size, params_.angular_step_size);
                break;

            default: // don't do anything.
                break;
            }
            std::cout << "\r[" << c << "]";
            std::cout << std::fixed;
            std::cout << " LinVel("
                      << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.linear.x << ", "
                      << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.linear.y << ", "
                      << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.linear.z << ")";
            std::cout << " YawVel("
                      << std::setw(5) << std::setprecision(2) << msg_cmd_vel_.angular.z << ")";
            std::cout << " Steps(Lin:"
                      << std::setw(5) << std::setprecision(2) << params_.linear_step << ", Ang"
                      << std::setw(5) << std::setprecision(2) << params_.angular_step << ")";

            std::cout << "    "; // pad some spaces just in case
            std::cout.flush();
            std::cout << "\b\b\b\b"; // remove extra nonsense characters.
            std::cout.flush();

            pub_cmd_vel_->publish(msg_cmd_vel_);
        }

        bool getch(char &c)
        {
            c = 0;
            termios old;
            bool error = false;
            if (tcgetattr(0, &old) < 0)
                return true; // perror("tcsetattr()");
            old.c_lflag &= ~ICANON;
            old.c_lflag &= ~ECHO;
            old.c_cc[VMIN] = 1;
            old.c_cc[VTIME] = 0;
            if (tcsetattr(0, TCSANOW, &old) < 0)
                error = true; // perror("tcsetattr ICANON");
            if (read(0, &c, 1) < 0)
                error = true;
            old.c_lflag |= ICANON;
            old.c_lflag |= ECHO;
            if (tcsetattr(0, TCSADRAIN, &old) < 0)
                error = true; // perror("tcsetattr ~ICANON");
            return error;
        }
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor exe;

    // mapper node
    auto node = std::make_shared<ee4308::drone::ROSNodeTeleop>(
        "teleop" // node name
    );

    exe.add_node(node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
