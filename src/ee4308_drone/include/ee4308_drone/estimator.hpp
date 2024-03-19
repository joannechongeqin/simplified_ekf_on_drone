#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"             // odom_drone
#include "geometry_msgs/msg/twist.hpp"           // gt_vel, cmd_vel
#include "geometry_msgs/msg/pose.hpp"            // gt_pose
#include "geometry_msgs/msg/point_stamped.hpp"   // altitude
#include "geometry_msgs/msg/vector3_stamped.hpp" // magnetic
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp" // gps
#include "sensor_msgs/msg/range.hpp"

#include "Eigen/Dense"

#include "ee4308_lib/core.hpp"

#pragma once
namespace ee4308::drone
{
    struct EstimatorParameters
    { // contains defaults that can be overwritten
        struct Topics
        {
            std::string odom_drone = "odom";
            std::string gps = "gps";
            std::string sonar = "sonar";
            std::string baro = "altitude";
            std::string magnetic = "magnetic";
            std::string imu = "imu";
            std::string gt_pose = "gt_pose";
            std::string gt_vel = "gt_vel";
        } topics;
        double frequency = 20;
        double G = 9.8;
        double var_imu_x = 0.2;
        double var_imu_y = 0.2;
        double var_imu_z = 0.2;
        double var_imu_a = 0.2;
        double var_gps_x = 0.1;
        double var_gps_y = 0.1;
        double var_gps_z = 0.1;
        double var_baro = 0.1;
        double var_sonar = 0.1;
        double var_magnet = 0.1;
        double rad_polar = 6356752.3;
        double rad_equator = 6378137;
        double keep_old_sonar = 0.5;
        bool verbose = true;
        bool use_gt = false;
    };

    /**
     * The Estimator ROS Node that maintains subscribers and publishers for the Estimator class.
     */
    class ROSNodeEstimator : public rclcpp::Node
    {
    private:
        EstimatorParameters params_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_drone_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sonar_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_baro_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_magnetic_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_gt_pose_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_gt_vel_;
        rclcpp::TimerBase::SharedPtr looper_;

        Eigen::Vector2d Xx_ = {0, 0}, Xy_ = {0, 0}, Xa_ = {0, 0}, Xz_ = {0, 0};
        Eigen::Matrix2d Px_ = Eigen::Matrix2d::Constant(1e3),
                        Py_ = Eigen::Matrix2d::Constant(1e3),
                        Pa_ = Eigen::Matrix2d::Constant(1e3),
                        Pz_ = Eigen::Matrix2d::Constant(1e3);

        // Eigen::Vector2d Xx_ = {0, 0}, Xy_ = {0, 0}, Xa_ = {0, 0};
        // Eigen::Vector3d Xz_ = {0, 0, 0};
        // Eigen::Matrix2d Px_ = Eigen::Matrix2d::Constant(1e3),
        //                 Py_ = Eigen::Matrix2d::Constant(1e3),
        //                 Pa_ = Eigen::Matrix2d::Constant(1e3);
        // Eigen::Matrix3d Pz_ = Eigen::Matrix3d::Constant(1e3);

        Eigen::Vector3d initial_ECEF_ = {NAN, NAN, NAN};
        Eigen::Vector3d initial_;

        Eigen::Vector3d Ygps_ = {NAN, NAN, NAN};
        double Ymagnet_ = NAN, Ybaro_ = NAN, Ysonar_ = 0;

        double last_time_ = 0;
        bool initialized_ecef_ = false;
        bool initialized_magnetic_ = false;

    public:
        /**
         * Constructor for the Estimator ROS Node.
         * @param name name of node.
         */
        ROSNodeEstimator(
            const double &initial_x, const double &initial_y, const double &initial_z,
            const std::string &name = "estimator")
            : Node(name)
        {
            Xx_[0] = initial_x;
            Xy_[0] = initial_y;
            Xz_[0] = initial_z;
            initial_ << initial_x, initial_y, initial_z;

            initParams();
            initTopics();
            initLoop();

            RCLCPP_INFO_STREAM(this->get_logger(), "Estimator node initialized!");
        }

    private:
        // ================================ INITIALIZERS ========================================
        void initParams()
        {
            initParam(this, "topics.odom_drone", params_.topics.odom_drone);
            initParam(this, "topics.gps", params_.topics.gps);
            initParam(this, "topics.sonar", params_.topics.sonar);
            initParam(this, "topics.magnetic", params_.topics.magnetic);
            initParam(this, "topics.baro", params_.topics.baro);
            initParam(this, "topics.imu", params_.topics.imu);
            initParam(this, "topics.gt_pose", params_.topics.gt_pose);
            initParam(this, "topics.gt_vel", params_.topics.gt_vel);
            initParam(this, "frequency", params_.frequency);
            initParam(this, "G", params_.G);
            initParam(this, "var_imu_x", params_.var_imu_x);
            initParam(this, "var_imu_y", params_.var_imu_y);
            initParam(this, "var_imu_z", params_.var_imu_z);
            initParam(this, "var_imu_a", params_.var_imu_a);
            initParam(this, "var_gps_x", params_.var_gps_x);
            initParam(this, "var_gps_y", params_.var_gps_y);
            initParam(this, "var_gps_z", params_.var_gps_z);
            initParam(this, "rad_polar", params_.rad_polar);
            initParam(this, "rad_equator", params_.rad_equator);
            initParam(this, "keep_old_sonar", params_.keep_old_sonar);
            initParam(this, "verbose", params_.verbose);
            initParam(this, "use_gt", params_.use_gt);
        }

        void initTopics()
        {
            // Initialize publishers
            pub_odom_drone_ = create_publisher<nav_msgs::msg::Odometry>(params_.topics.odom_drone, 1);

            // Initialize subscribers
            if (params_.use_gt == false)
            {
                sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
                    params_.topics.gps, 1,
                    std::bind(&ROSNodeEstimator::correctFromGps, this, std::placeholders::_1));
                sub_sonar_ = create_subscription<sensor_msgs::msg::Range>(
                    params_.topics.sonar, 1,
                    std::bind(&ROSNodeEstimator::correctFromSonar, this, std::placeholders::_1));
                sub_magnetic_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
                    params_.topics.magnetic, 1,
                    std::bind(&ROSNodeEstimator::correctFromMagnetic, this, std::placeholders::_1));
                sub_baro_ = create_subscription<geometry_msgs::msg::PointStamped>(
                    params_.topics.baro, 1,
                    std::bind(&ROSNodeEstimator::correctFromBaro, this, std::placeholders::_1));
                sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
                    params_.topics.imu, 1,
                    std::bind(&ROSNodeEstimator::predict, this, std::placeholders::_1));
            }
            else
            { // if using ground truth.
                sub_gt_pose_ = create_subscription<geometry_msgs::msg::Pose>(
                    params_.topics.gt_pose, 10,
                    std::bind(&ROSNodeEstimator::subCbGtPose, this, std::placeholders::_1));
                sub_gt_vel_ = create_subscription<geometry_msgs::msg::Twist>(
                    params_.topics.gt_vel, 10,
                    std::bind(&ROSNodeEstimator::subCbGtVel, this, std::placeholders::_1));
            }

            // Waiting not necessary becos calculations are triggered in subscriber callbacks
        }

        void initLoop()
        {
            auto period = std::chrono::duration<double, std::ratio<1>>(1 / params_.frequency);
            looper_ = this->create_wall_timer(
                period,
                std::bind(&ROSNodeEstimator::publishOdom, this));
        }

        // ================================ SUBSCRIBER CALLBACKS ========================================
        void subCbGtPose(const geometry_msgs::msg::Pose &msg)
        {
            Xx_[0] = msg.position.x;
            Xy_[0] = msg.position.y;
            Xz_[0] = msg.position.z;
            Xa_[0] = quaternionToYaw(msg.orientation);
        }

        void subCbGtVel(const geometry_msgs::msg::Twist &msg)
        {
            Xx_[1] = msg.linear.x;
            Xy_[1] = msg.linear.y;
            Xz_[1] = msg.linear.z;
            Xa_[1] = msg.angular.z;
        }

        // ================================  Main Loop / Odom Publisher ========================================
        void verbose()
        {
            if (params_.verbose == false)
                return;

            RCLCPP_INFO_STREAM(this->get_logger(), "===");

            std::cout << std::fixed;
            std::cout << " Pose("
                      << std::setw(7) << std::setprecision(3) << Xx_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(0) << ","
                      << std::setw(7) << std::setprecision(3) << limit_angle(Xa_(0)) << ")"
                      << std::endl;
            std::cout << "Twist("
                      << std::setw(7) << std::setprecision(3) << Xx_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xy_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xz_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Xa_(1) << ")"
                      << std::endl;
            std::cout << "  GPS("
                      << std::setw(7) << std::setprecision(3) << Ygps_(0) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(1) << ","
                      << std::setw(7) << std::setprecision(3) << Ygps_(2) << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << " Baro("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ybaro_ << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            // std::cout << "BBias("
            //           << std::setw(8) << "---  ,"
            //           << std::setw(8) << "---  ,"
            //           << std::setw(7) << std::setprecision(3) << Xz_(2) << ","
            //           << std::setw(8) << "---  )"
            //           << std::endl;
            std::cout << "Sonar("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ysonar_ << ","
                      << std::setw(8) << "---  )"
                      << std::endl;
            std::cout << "Magnt("
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(8) << "---  ,"
                      << std::setw(7) << std::setprecision(3) << Ymagnet_ << ")"
                      << std::endl;
        }

        void publishOdom()
        {
            // you can extend this to include velocities if you want, but the topic name may have to change from pose to something else.
            // odom is already taken.
            nav_msgs::msg::Odometry msg;

            msg.header.stamp = this->now();
            msg.child_frame_id = std::string(this->get_namespace()) + "/base_footprint";
            msg.header.frame_id = std::string(this->get_namespace()) + "/odom";

            msg.pose.pose.position.x = Xx_[0];
            msg.pose.pose.position.y = Xy_[0];
            msg.pose.pose.position.z = Xz_[0];
            msg.pose.pose.orientation = yawToQuaternion(Xa_[0]);
            msg.pose.covariance[0] = Px_(0, 0);
            msg.pose.covariance[7] = Py_(0, 0);
            msg.pose.covariance[14] = Pz_(0, 0);
            msg.pose.covariance[35] = Pa_(0, 0);

            msg.twist.twist.linear.x = Xx_[1];
            msg.twist.twist.linear.y = Xy_[1];
            msg.twist.twist.linear.z = Xz_[1];
            msg.twist.twist.angular.z = Xa_[1];
            msg.twist.covariance[0] = Px_(1, 1);
            msg.twist.covariance[7] = Py_(1, 1);
            msg.twist.covariance[14] = Pz_(1, 1);
            msg.twist.covariance[35] = Pa_(1, 1);

            pub_odom_drone_->publish(msg);

            verbose();
        }

        // ================================ GPS sub callback / EKF Correction ========================================
        Eigen::Vector3d getECEF(
            const double &sin_lat, const double &cos_lat,
            const double &sin_lon, const double &cos_lon,
            const double &alt)
        {
            Eigen::Vector3d ECEF;
            // --- FIXME ---
            // params_.rad_polar, params_.rad_equator
            // --- EOFIXME ---
            return ECEF;
        }

        void correctFromGps(const sensor_msgs::msg::NavSatFix msg)
        { // avoiding const & due to possibly long calcs.
            const double DEG2RAD = M_PI / 180;
            double lat = -msg.latitude * DEG2RAD;  // !!! Gazebo spherical coordinates have a bug. Need to negate.
            double lon = -msg.longitude * DEG2RAD; // !!! Gazebo spherical coordinates have a bug. Need to negate.
            double alt = msg.altitude;

            double sin_lat = sin(lat);
            double cos_lat = cos(lat);
            double sin_lon = sin(lon);
            double cos_lon = cos(lon);

            if (initialized_ecef_ == false)
            {
                initial_ECEF_ = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);
                initialized_ecef_ = true;
                return;
            }

            Eigen::Vector3d ECEF = getECEF(sin_lat, cos_lat, sin_lon, cos_lon, alt);

            // !!! After obtaining NED, and *rotating* to Gazebo's world frame,
            //      Store the measured x,y,z, in Ygps_.
            //      Required for terminal printing during demonstration.

            // --- FIXME ---
            // get NED
            // get world coords (Ygps_ = ...)
            // Correct x y z
            // params_.var_gps_x, ...y, ...z
            // --- EOFIXME ---
        }

        // ================================ Sonar sub callback / EKF Correction ========================================
        void correctFromSonar(const sensor_msgs::msg::Range msg)
        {
            // !!! Store the measured sonar range in Ysonar_.
            //      Required for terminal printing during demonstration.

            double new_Ysonar = msg.range;
            if (new_Ysonar > msg.max_range)
            { // skip erroneous measurements
                return;
            }

            // --- FIXME ---
            // Ysonar_ = ...
            // Correct z
            // params_.var_sonar
            // --- EOFIXME ---
        }

        // ================================ Magnetic sub callback / EKF Correction ========================================
        void correctFromMagnetic(const geometry_msgs::msg::Vector3Stamped msg)
        {
            // Along the horizontal plane, the magnetic north in Gazebo points towards +x, when it should point to +y. It is a bug.
            // As the drone always starts pointing towards +x, there is no need to offset the calculation with an initial heading.

            // !!! Use limit_angle() to constrain angles between -pi and pi, especially if an angular difference needs to be calculated.

            // !!! Store the measured angle (world frame) in Ymagnet_.
            //      Required for terminal printing during demonstration.

            // --- FIXME ---
            // Ymagnet_ = ...
            // Correct yaw
            // params_.var_magnet
            // --- EOFIXME ---
        }

        // ================================ Baro sub callback / EKF Correction ========================================
        void correctFromBaro(const geometry_msgs::msg::PointStamped msg)
        {
            // !!! Store the measured barometer altitude in Ybaro_.
            //      Required for terminal printing during demonstration.

            (void) msg;

            // --- FIXME ---
            // Ybaro_ = ...
            // Correct z
            // params_.var_baro
            // --- EOFIXME ---
        }

        // ================================ IMU sub callback / EKF Prediction ========================================
        void predict(const sensor_msgs::msg::Imu msg)
        {
            rclcpp::Time tnow = msg.header.stamp;
            double dt = tnow.seconds() - last_time_;
            last_time_ = tnow.seconds();

            if (dt < 1e-3)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "elapsed too small(" << dt << "). Skipping Prediction");
                return;
            }

            // !!! NOT ALLOWED TO USE ORIENTATION FROM IMU as ORIENTATION IS DERIVED FROM ANGULAR VELOCTIY !!!

            // !!! Store the states in Xx_, Xy_, Xz_, and Xa_.
            //      Store the covariances in Px_, Py_, Pz_, and Pa_.
            //      Required for terminal printing during demonstration.


            // --- FIXME ---
            // params_.G
            // params._var_imu_x, ...y, ...z, ...a
            // Xx_ = ..., Xy_, Xz_, Xa_
            // Px_, Py_, Pz_, Pa_
            // --- EOFIXME ---
        }
    };
}
