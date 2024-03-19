#include <iostream>
#include <chrono>

#include <geometry_msgs/msg/pose.hpp>

#include "ee4308_lib/common.hpp"
#include "ee4308_lib/vec2.hpp"

#pragma once
namespace ee4308
{
    using namespace std::chrono_literals;

    using V2 = Vec2<int>;
    using V2d = Vec2<double>;
    
    /**
     * Returns the discrete heading 0 to 7, with 0 being in the (>0, 0) direction, 1 being in (>0, >0), etc. in the anti-clockwise direction.
    */
    int directionToHeading(const V2 &direction)
    {   // treating (1, 0) as north,
        if (direction.x > 0)
        {
            if (direction.y > 0) 
                return 1; // northwest
            else if (direction.y < 0)
                return 7; // northeast
            else
                return 0; // north
        }
        else if (direction.x < 0)
        {
            if (direction.y > 0) 
                return 3; // southwest
            else if (direction.y < 0)
                return 5; // southeast
            else
                return 4; // south
        }
        else
        {
            if (direction.y > 0) 
                return 2; // west
            else if (direction.y < 0)
                return 6; // east
            else
                return -1; // invalid.
        }
    }

    /**
     * Returns the sign direction vector corresponding to the heading. 0 returns (1, 0), 1 returns (1, 1), etc.
    */
    V2 headingToDirection(const int &heading)
    {   // treating (1, 0) as north,
        switch (heading)
        {
            case 0:
                return {1, 0};
            case 1:
                return {1, 1};
            case 2:
                return {0, 1};
            case 3:
                return {-1, 1};
            case 4:
                return {-1, 0};
            case 5:
                return {-1, -1};
            case 6:
                return {0, -1};
            case 7:
                return {1, -1};
            default:
                return {0, 0};
        }
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return atan2(siny_cosp, cosy_cosp);
    }

    geometry_msgs::msg::Quaternion yawToQuaternion(const double &yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.w = cos(yaw / 2);
        q.x = 0;
        q.y = 0;
        q.z = sin(yaw / 2);
        return q;
    }

    struct DOF3
    {
        V2d position;
        double orientation;
    };

    DOF3 convertPose(const geometry_msgs::msg::Pose &pose)
    {
        DOF3 dof3;
        dof3.position = {pose.position.x, pose.position.y};
        dof3.orientation = quaternionToYaw(pose.orientation);
        
        return dof3;
    }

    /**
     * Declares a parameter and assigns any set value (from YAML) to "to". 
     * @param param_name the name of the parameter
     * @param to must be initialized with a default value.
     * @param cout_width the width of the printed string describing the parameter. Set to zero to disable the print.
    */
    template <typename T>
    void initParam(rclcpp::Node *const &node, const std::string &param_name, T &to, const size_t &cout_width=25)
    {   
        if (node->has_parameter(param_name) == false)
            node->declare_parameter<T>(param_name, to);
        node->get_parameter_or<T>(param_name, to, to);
        
        if (cout_width > 0)
            std::cout << std::setw(cout_width) << param_name << ": " << to << std::endl;
    }

}