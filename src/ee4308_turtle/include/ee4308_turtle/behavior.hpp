#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ee4308_lib/core.hpp"
#include "ee4308_interfaces/srv/waypoint.hpp"
#include "ee4308_interfaces/srv/get_waypoint.hpp"

#pragma once
namespace ee4308::turtle
{
    struct BehaviorParameters
    { // contains defaults that can be overwritten
        struct Services
        {
            std::string goto_waypoint = "goto_waypoint";
            std::string get_waypoint = "get_waypoint";
        } services;
        std::vector<V2d> waypoints;
    };

    /**
     * The Behavior ROS Node that maintains subscribers and publishers for the Behavior class.
     */
    class ROSNodeBehavior : public rclcpp::Node
    {
    private:
        BehaviorParameters params_;
        rclcpp::Client<ee4308_interfaces::srv::Waypoint>::SharedPtr client_goto_waypoint_;     // client
        rclcpp::Service<ee4308_interfaces::srv::GetWaypoint>::SharedPtr service_get_waypoint_; // service
        rclcpp::CallbackGroup::SharedPtr cb_group_;                                            // to allow all callbacks to simultaneously occur in the executor.
        rclcpp::TimerBase::SharedPtr looper_;                                 // main looper
        std::mutex mutex_waypoint_;
        V2d waypoint_;
        

    public:
        /**
         * Constructor for the Behavior ROS Node.
         * @param name name of node.
         */
        ROSNodeBehavior(
            const std::string &name = "behavior")
            : Node(name)
        {
            cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            // Get all parameters from the param server.
            initParams();

            // Initialize services
            initServices();

            // Initialize main loop
            looper_ = this->create_wall_timer(std::chrono::duration<int64_t, std::milli>(500),
                                              std::bind(&ROSNodeBehavior::loop, this));

            // RCLCPP_INFO_STREAM(this->get_logger(), "Behavior node initialized!");
        }

        /**
         * main run loop
         */
        void loop()
        {
            for (const V2d &waypoint : params_.waypoints)
            {
                { // lock should occur almost instantly.
                    const std::lock_guard<std::mutex> lock(mutex_waypoint_);
                    waypoint_ = waypoint; // write to private property
                }

                requestGotoWaypoint(waypoint);

                if (rclcpp::ok() == false)
                    break;
            }

            rclcpp::shutdown();
        }

    private:
        /**
         * Initialize parameters from the parameter server
         */
        void initParams()
        {
            declare_parameter<std::string>("services.goto_waypoint", params_.services.goto_waypoint);
            get_parameter<std::string>("services.goto_waypoint", params_.services.goto_waypoint);
            std::cout << "services.goto_waypoint: " << params_.services.goto_waypoint << std::endl;

            // waypoints
            std::vector<double> flat;
            declare_parameter<std::vector<double>>("waypoints", flat);
            flat = get_parameter("waypoints").as_double_array();
            for (auto & w : flat)
                std::cout << w << "; ";
            std::cout << std::endl;
            params_.waypoints.clear();
            for (size_t i = 1; i < flat.size(); i += 2) // ignore the last value if flat's size is odd numbered.
                params_.waypoints.emplace_back(flat[i - 1], flat[i]);
            std::cout << "waypoints: { ";
            for (const V2d &waypoint : params_.waypoints)
                std::cout << waypoint << "; ";
            std::cout << "}" << std::endl;
        }

        void initServices()
        {
            // Initialize services
            client_goto_waypoint_ = create_client<ee4308_interfaces::srv::Waypoint>(
                params_.services.goto_waypoint, 
                rmw_qos_profile_services_default,
                cb_group_);

            service_get_waypoint_ = create_service<ee4308_interfaces::srv::GetWaypoint>(
                params_.services.get_waypoint,
                std::bind(&ROSNodeBehavior::serviceGetWaypoint, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default, cb_group_);

            // Wait for the service to respond.
            while (rclcpp::ok() == true &&
                   client_goto_waypoint_->wait_for_service(std::chrono::duration<int, std::milli>(200)) == false) // wait for 200ms
            {
                // RCLCPP_INFO_STREAM(get_logger(), "Waiting for service servers...");
            }
        }

        /**
         * Requests a map from mapper. returns true if the map is received and the planner's internal cost map is updated.
         */
        bool requestGotoWaypoint(const V2d &waypoint)
        {
            // std::cout << "Requesting waypoint ( " << waypoint << " )" << std::endl;

            // Send the request.
            auto request = std::make_shared<ee4308_interfaces::srv::Waypoint::Request>();
            request->waypoint.position.x = waypoint.x;
            request->waypoint.position.y = waypoint.y;
            auto result = client_goto_waypoint_->async_send_request(request);

            while (rclcpp::ok() == true)
            {   // poll every 200ms to see if service is complete.
                auto status = result.future.wait_for(std::chrono::duration<int, std::milli>(200));
                if (status == std::future_status::ready)
                    break;
            }

            // return false if shutdown
            if (rclcpp::ok() == false)
                return false; // Ctrl+C

            // std::cout << "Waypoint ( " << waypoint << " ) reached." << std::endl;
            return true;
        }

        /**
         * Services a request to obtain the current waypoint.
        */
        void serviceGetWaypoint(const std::shared_ptr<ee4308_interfaces::srv::GetWaypoint::Request> request,
                                std::shared_ptr<ee4308_interfaces::srv::GetWaypoint::Response> response)
        {
            (void) request; // not used.

            const std::lock_guard<std::mutex> lock(mutex_waypoint_);
            response->waypoint.point.x = waypoint_.x;
            response->waypoint.point.y = waypoint_.y;
        }
    };
}