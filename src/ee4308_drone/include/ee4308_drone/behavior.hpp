#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ee4308_lib/core.hpp"
#include "ee4308_interfaces/srv/get_waypoint.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "ee4308_drone/behavior_state.hpp"

#pragma once
namespace ee4308::drone
{
    struct BehaviorParameters
    { // contains defaults that can be overwritten
        struct Services
        {
            std::string get_turtle_waypoint = "/turtle/get_waypoint";
        } services;
        struct Topics
        {
            std::string odom_drone = "odom";
            std::string odom_turtle = "/turtle/odom";
            std::string waypoint = "waypoint";
            std::string takeoff = "takeoff";
            std::string land = "land";
        } topics;
        std::string frame_id = "/world";
        double nearby = 0.1; // the threshold in meters to determine if a waypoint is close enough.
        double cruise_height = 1;
        double frequency = 5;
    };

    class ROSNodeBehavior : public rclcpp::Node
    {
    private:
        BehaviorParameters params_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_drone_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_turtle_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_waypoint_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_takeoff_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_land_;
        rclcpp::Client<ee4308_interfaces::srv::GetWaypoint>::SharedPtr client_get_turtle_waypoint_;
        rclcpp::TimerBase::SharedPtr looper_;
        rclcpp::CallbackGroup::SharedPtr cbg_reentrant_;

        nav_msgs::msg::Odometry odom_drone_;
        nav_msgs::msg::Odometry odom_turtle_;
        geometry_msgs::msg::PointStamped waypoint_;

        std::mutex mutex_odom_drone_;
        std::mutex mutex_odom_turtle_;

        double initial_x_, initial_y_, initial_z_;
        BehaviorState state_;

    public:
        ROSNodeBehavior(
            const double &initial_x, const double &initial_y, const double &initial_z,
            const std::string &name = "behavior")
            : Node(name), initial_x_(initial_x), initial_y_(initial_y), initial_z_(initial_z)
        {
            cbg_reentrant_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            initParams();
            initTopics();
            initServices();
            initLoop();

            RCLCPP_INFO_STREAM(this->get_logger(), "Behavior node initialized!");
        }

    private:
        // ================================ INITIALIZERS ========================================
        void initParams()
        {
            initParam(this, "services.get_turtle_waypoint", params_.services.get_turtle_waypoint);
            initParam(this, "topics.odom_drone", params_.topics.odom_drone);
            initParam(this, "topics.odom_turtle", params_.topics.odom_turtle);
            initParam(this, "topics.waypoint", params_.topics.waypoint);
            initParam(this, "topics.takeoff", params_.topics.takeoff);
            initParam(this, "topics.land", params_.topics.land);
            initParam(this, "frame_id", params_.frame_id);
            initParam(this, "nearby", params_.nearby);
            initParam(this, "cruise_height", params_.cruise_height);
            initParam(this, "frequency", params_.frequency);
        }

        void initTopics()
        {
            // Initialize publishers
            pub_waypoint_ = create_publisher<geometry_msgs::msg::PointStamped>(params_.topics.waypoint, 1);
            pub_takeoff_ = create_publisher<std_msgs::msg::Empty>(params_.topics.takeoff, 1);
            pub_land_ = create_publisher<std_msgs::msg::Empty>(params_.topics.land, 1);

            // Initialize published messages
            waypoint_.point.x = initial_x_;
            waypoint_.point.y = initial_y_;
            waypoint_.point.z = initial_z_;

            // Initialize messages with values that will never be written by their publishers.
            odom_drone_.pose.pose.position.x = NAN;
            odom_turtle_.pose.pose.position.x = NAN;

            // Initialize subscribers
            sub_odom_drone_ = create_subscription<nav_msgs::msg::Odometry>(
                params_.topics.odom_drone, 1,
                std::bind(&ROSNodeBehavior::subCbOdomDrone, this, std::placeholders::_1));
            sub_odom_turtle_ = create_subscription<nav_msgs::msg::Odometry>(
                params_.topics.odom_turtle, 1,
                std::bind(&ROSNodeBehavior::subCbOdomTurtle, this, std::placeholders::_1));

            // Wait for messages to arrive.
            while (rclcpp::ok())
            {
                rclcpp::sleep_for(200ms);
                rclcpp::spin_some(this->get_node_base_interface());
                {
                    const std::lock_guard<std::mutex> lock(mutex_odom_drone_);
                    if (std::isnan(odom_drone_.pose.pose.position.x) == true)
                    continue;
                    }
                {
                    const std::lock_guard<std::mutex> lock(mutex_odom_turtle_);
                    if (std::isnan(odom_turtle_.pose.pose.position.x) == true)
                    continue;}
                break;
            }
        }

        void initServices()
        {
            client_get_turtle_waypoint_ = create_client<ee4308_interfaces::srv::GetWaypoint>(
                params_.services.get_turtle_waypoint,
                rmw_qos_profile_services_default,
                cbg_reentrant_);

            // wait for service to respond
            while (rclcpp::ok() == true &&
                   client_get_turtle_waypoint_->wait_for_service(200ms) == false) // wait for 200ms
            {
            }
        }

        void initLoop()
        {
            state_ = BehaviorState::Takeoff;
            transition();

            looper_ = this->create_wall_timer(
                std::chrono::duration<double, std::ratio<1>>(1 / params_.frequency),
                std::bind(&ROSNodeBehavior::loop, this),
                cbg_reentrant_);
        }

        // ================================ SUBSCRIBER CALLBACKS ========================================
        void subCbOdomDrone(const nav_msgs::msg::Odometry &msg)
        {
            const std::lock_guard<std::mutex> lock(mutex_odom_drone_);
            odom_drone_ = msg;
        }

        void subCbOdomTurtle(const nav_msgs::msg::Odometry &msg)
        {
            const std::lock_guard<std::mutex> lock(mutex_odom_turtle_);
            odom_turtle_ = msg;
        }

        nav_msgs::msg::Odometry getOdomDrone()
        {
            const std::lock_guard<std::mutex> lock(mutex_odom_drone_);
            return odom_drone_;
        }

        nav_msgs::msg::Odometry getOdomTurtle()
        {
            const std::lock_guard<std::mutex> lock(mutex_odom_turtle_);
            return odom_turtle_;
        }

        // ================================  PUBLISHING ========================================
        void publishWaypoint()
        {
            pub_waypoint_->publish(waypoint_);
        }

        void publishWaypoint(const double &x, const double &y, const double &z)
        {
            waypoint_.header.stamp = this->now();
            waypoint_.header.frame_id = params_.frame_id;
            waypoint_.point.x = x;
            waypoint_.point.y = y;
            waypoint_.point.z = z;
            publishWaypoint();
        }

        // ================================  SERVICES ========================================
        /** Synchronous (blocking) request for turtle waypoint. Returns false if timed out (no response for 500ms after request).*/
        bool requestTurtleWaypoint()
        {

            auto request = std::make_shared<ee4308_interfaces::srv::GetWaypoint::Request>();
            auto result = client_get_turtle_waypoint_->async_send_request(request);

            // block until there is a response
            auto status = result.future.wait_for(2000ms);
            if (status == std::future_status::ready)
            {
                waypoint_.point = result.get()->waypoint.point; // copy
                waypoint_.point.z = params_.cruise_height;
                return true;
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "Turtle Waypoint request timed-out!"); // try to land instead
            return false;
        }

        // ================================  MAIN LOOP and FUNCTIONS ========================================
        bool isNearby()
        {
            nav_msgs::msg::Odometry odom_drone = getOdomDrone();
            double dx = waypoint_.point.x - odom_drone.pose.pose.position.x;
            double dy = waypoint_.point.y - odom_drone.pose.pose.position.y;
            double dz = waypoint_.point.z - odom_drone.pose.pose.position.z;
            double distance = sqrt(dx * dx + dy * dy + dz * dz);
            return distance < params_.nearby;
        }

        void transition()
        {
            if (state_ == BehaviorState::Takeoff)
            {
                std_msgs::msg::Empty msg;
                pub_takeoff_->publish(msg);
                publishWaypoint(initial_x_, initial_y_, params_.cruise_height);
                state_ = BehaviorState::Start;
            }
            else if (state_ == BehaviorState::Start)
            {
                if (requestTurtleWaypoint() == true)
                { // can obtain waypoint. turtle/behavior node is still alive.
                    // waypoint is determined and published in the main loop for BehaviorState::Turtle
                    state_ = BehaviorState::Turtle;
                }
                else
                { // request timed out. turtle/behavior node has probably shutdown.
                    publishWaypoint(initial_x_, initial_y_, initial_z_);
                    state_ = BehaviorState::Land;
                }
            }
            else if (state_ == BehaviorState::Turtle)
            {
                // if turtle/behavior node is alive, waypoint is written and published.
                // if turtle/behavior node has shutdown, no change to waypoint as turtle's can be assumed to be at its final waypoint.
                if (requestTurtleWaypoint() == true)
                    publishWaypoint();

                state_ = BehaviorState::Waypoint;
            }
            else if (state_ == BehaviorState::Waypoint)
            {
                publishWaypoint(initial_x_, initial_y_, params_.cruise_height);
                state_ = BehaviorState::Start;
            }
            else if (state_ == BehaviorState::Land)
            {
                //  no change to waypoint.
                pub_land_->publish(std_msgs::msg::Empty());
                state_ = BehaviorState::Shutdown;
            }
            else
            {
                // else case not managed
                state_ = BehaviorState::Shutdown;
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Transitioning to " << state_);
        }

        void loop()
        {
            if (isNearby() == true)
                transition();

            if (state_ == BehaviorState::Turtle)
            {
                nav_msgs::msg::Odometry odom_turtle = getOdomTurtle();
                publishWaypoint(
                    odom_turtle.pose.pose.position.x,
                    odom_turtle.pose.pose.position.y,
                    params_.cruise_height);
            }
            else if (state_ == BehaviorState::Shutdown)
            {
                rclcpp::shutdown();
            }
        }
    };
}