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
        double keep_old_sonar = 0.5; // this value will need to be updated according to the aplha value in a low pass filter
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

        // Eigen::Vector2d Xx_ = {0, 0}, Xy_ = {0, 0}, Xa_ = {0, 0}, Xz_ = {0, 0};
        // Eigen::Vector2d Xx_ = {0, 0}, Xy_ = {0, 0}, Xa_ = {0, 0}, Xz_ = {0, 0};
        // Eigen::Matrix2d Px_ = Eigen::Matrix2d::Constant(1e3),
        //                 Py_ = Eigen::Matrix2d::Constant(1e3),
        //                 Pa_ = Eigen::Matrix2d::Constant(1e3),
        //                 Pz_ = Eigen::Matrix2d::Constant(1e3);
        //                 Pa_ = Eigen::Matrix2d::Constant(1e3),
        //                 Pz_ = Eigen::Matrix2d::Constant(1e3);

        Eigen::Vector2d Xx_ = {0, 0}, Xy_ = {0, 0}, Xa_ = {0, 0};
        Eigen::Vector3d Xz_ = {0, 0, 0};
        Eigen::Matrix2d Px_ = Eigen::Matrix2d::Constant(1e3),
                        Py_ = Eigen::Matrix2d::Constant(1e3),
                        Pa_ = Eigen::Matrix2d::Constant(1e3);
        Eigen::Matrix3d Pz_ = Eigen::Matrix3d::Constant(1e3);

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
            std::cout << "BBias("
                       << std::setw(8) << "---  ,"
                       << std::setw(8) << "---  ,"
                       << std::setw(7) << std::setprecision(3) << Xz_(2) << ","
                       << std::setw(8) << "---  )"
                       << std::endl;
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

        // ================================ Variance Calculation ========================================
        double diffSquared(double val, double mean){
            double res = pow((val-mean), 2);
            return res;
        }

        double variance_calc(const std::vector<double>& y_list){
            double var_sum = 0;
            double sum = std::accumulate(y_list.begin(), y_list.end(), 0.0);
            double mean = sum/(y_list.size());
            
            for(const double& val : y_list){
                var_sum += pow((val-mean), 2);
            }

            return var_sum/(y_list.size() - 1);
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

            double a = params_.rad_equator, b = params_.rad_polar;
            double a2 = a * a, b2 = b * b;
            double e2 = 1.0 - b2 / a2; // square of the first numerical eccentricity
            double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
            ECEF[0] = (N + alt) * cos_lat * cos_lon; // x_e
            ECEF[1] = (N + alt) * cos_lat * sin_lon; // y_e
            ECEF[2] = (b2 / a2 * N + alt) * sin_lat; // z_e

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
            Eigen::Vector3d NED;
            Eigen::Matrix3d R_NED2ECEF; // rotation matrix from NED frame to ECEF frame
            R_NED2ECEF << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,
                          -sin_lat * sin_lon,  cos_lon, -cos_lat * sin_lon,
                                cos_lat,          0,         -sin_lat;
            NED = R_NED2ECEF.transpose() * (ECEF - initial_ECEF_);

            // get world coords (Ygps_ = ...)
            Eigen::Matrix3d R_NED2WORLD; // rotation matrix from NED frame to world frame
            R_NED2WORLD << 0, 1, 0,
                           1, 0, 0,
                           0, 0, -1;
            Ygps_ = R_NED2WORLD * NED + initial_; // measurement

            // Correct x y z
            // params_.var_gps_x, ...y, ...z        
            // EKF correction for Xx_
            Eigen::VectorXd Ygps_x(1), hgps_x(1), Vgps_x(1), Rgps_x(1);
            Eigen::RowVector2d Hgps_x;
            Ygps_x << Ygps_[0];
            hgps_x << Xx_[0];
            Hgps_x << 1, 0;
            Vgps_x << 1;
            Rgps_x << params_.var_gps_x;
            auto Kx = Px_ * Hgps_x.transpose() * (Hgps_x * Px_ * Hgps_x.transpose() 
                                                    + Vgps_x * Rgps_x * Vgps_x).inverse();
            Xx_ = Xx_ + Kx * (Ygps_x - hgps_x);
            Px_ = Px_ - Kx * Hgps_x * Px_;

            // EKF correction for Xy_
            Eigen::VectorXd Ygps_y(1), hgps_y(1), Vgps_y(1), Rgps_y(1);
            Eigen::RowVector2d Hgps_y;
            Ygps_y << Ygps_[1];
            hgps_y << Xy_[0];
            Hgps_y << 1, 0;
            Vgps_y << 1;
            Rgps_y << params_.var_gps_y;
            auto Ky = Py_ * Hgps_y.transpose() * (Hgps_y * Py_ * Hgps_y.transpose() 
                                                    + Vgps_y * Rgps_y * Vgps_y).inverse();
            Xy_ = Xy_ + Ky * (Ygps_y - hgps_y);
            Py_ = Py_ - Ky * Hgps_y * Py_;

            // EKF correction for Xz_
            Eigen::VectorXd Ygps_z(1), hgps_z(1), Vgps_z(1), Rgps_z(1);
            Eigen::RowVector3d Hgps_z;
            Ygps_z << Ygps_[2];
            hgps_z << Xz_[0];
            Hgps_z << 1, 0, 0;
            Vgps_z << 1;
            Rgps_z << params_.var_gps_z;
            auto Kz = Pz_ * Hgps_z.transpose() * (Hgps_z * Pz_ * Hgps_z.transpose() 
                                                    + Vgps_z * Rgps_z * Vgps_z).inverse();
            Xz_ = Xz_ + Kz * (Ygps_z - hgps_z);
            Pz_ = Pz_ - Kz * Hgps_z * Pz_;

            // --- EOFIXME ---
        }

        // ================================ Sonar sub callback / EKF Correction ========================================

        Eigen::VectorXd h_sonar(Eigen::VectorXd Xz_) {
            Eigen::VectorXd h(1);
            h << Xz_(0); // Assuming the first element of Xz_ is the vertical position
            return h;
        }

        // Sonar Measurement Jacobian
        Eigen::MatrixXd H_sonar() {
            Eigen::MatrixXd H(1, Xz_.size());
            H.setZero();
            H(0, 0) = 1; // Derivative of the vertical position with respect to itself is 1
            return H;
        }

        void correctFromSonar(const sensor_msgs::msg::Range msg)
        {
            // !!! Store the measured sonar range in Ysonar_.
            //      Required for terminal printing during demonstration.

            double new_Ysonar = msg.range;
            if (new_Ysonar > msg.max_range)
            { // skip erroneous measurements
                return;
            }

        // Low-pass filter parameters
        const double alpha = params_.keep_old_sonar; // Exponential forgetting factor. Close to 1: more weight to older measurements.

        // Apply low-pass filter to smooth the sonar measurements
        Ysonar_ = alpha * Ysonar_ + (1 - alpha) * new_Ysonar; // Smoothed measurement

        // Sonar measurement noise variance
        double var_sonar = params_.var_sonar;

        // Measurement vector with the smoothed value
        Eigen::VectorXd Y_sonar(1);
        Y_sonar << Ysonar_; // Use the smoothed sonar measurement

        // Calculate the Kalman Gain
        Eigen::MatrixXd H = H_sonar();
        Eigen::MatrixXd S = H * Pz_ * H.transpose() + var_sonar * Eigen::MatrixXd::Identity(1, 1);
        Eigen::MatrixXd K = Pz_ * H.transpose() * S.inverse();

        // Update state estimate and covariance matrix using the smoothed measurement
        Eigen::VectorXd z_pred = h_sonar(Xz_);
        Xz_ = Xz_ + K * (Y_sonar - z_pred);
        Pz_ = Pz_ - K * H * Pz_;
        }

        // ================================ Magnetic sub callback / EKF Correction ========================================

        std::vector<double> mag_y_list;
        double mag_init_count = 0;
        double mag_y_var = 0;
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

            Eigen::VectorXd V_mgn_a(1), R_mgn_a(1);
            Eigen::RowVector2d H_mgn_a;
            Ymagnet_ = limit_angle(atan2(-msg.vector.y, msg.vector.x)); //atan2(y, x)

            // Code to check for the variance value
            // if (mag_init_count >= 100) {
            //     mag_y_var = variance_calc(mag_y_list);
            //     std::cout << "Variance of magnetometer is: " << mag_y_var << std::endl;
            //     mag_y_list.clear();
            //     mag_init_count = 0;
            // }
            // else{
            //     mag_y_list.push_back(Y_mgn_a[0]);
            //     mag_init_count++;
            // }

            double h_mgn_a = limit_angle(Xa_[0]);
            H_mgn_a << 1, 0;
            V_mgn_a << 1;
            R_mgn_a << params_.var_magnet;

            // EKF Correction
            auto K_mgn = Pa_ * H_mgn_a.transpose() * (H_mgn_a * Pa_ * H_mgn_a.transpose() 
                                                        + V_mgn_a * R_mgn_a * V_mgn_a).inverse();
            double angle_diff = limit_angle(Ymagnet_ - h_mgn_a);
            Xa_ = Xa_ + K_mgn * angle_diff;
            Xa_[0] = limit_angle(Xa_[0]);
            Pa_ = Pa_ - K_mgn * H_mgn_a * Pa_;

            // --- EOFIXME ---
        }

        std::vector<double> baro_list;
        double baro_init_count = 0;
        double baro_var = params_.var_baro;
        // ================================ Baro sub callback / EKF Correction ========================================
        void correctFromBaro(const geometry_msgs::msg::PointStamped msg)
        {
            // !!! Store the measured barometer altitude in Ybaro_.
            //      Required for terminal printing during demonstration.

            (void) msg;

            Ybaro_ = msg.point.z;

            // Variance calculation using Ybaro_ samples, update baro_var every 100 samples
            if (baro_init_count >= 100) {
                baro_var = variance_calc(baro_list);
                baro_list.clear();
                baro_init_count = 0;
            }
            else {
                baro_list.push_back(Ybaro_);
                baro_init_count++;
            }

            std::cout << "Variance of barometer is: " << baro_var << std::endl;

            double hbar_z = Xz_[0];
            Eigen::VectorXd Vbar_z(1), Rbar_z(1);
            Vbar_z << 1;
            Rbar_z << baro_var;
            Eigen::RowVector3d Hbar_z = {1, 0, 1}; // include bias 

            // EKF Correction
            auto Kbar_z = Pz_ * Hbar_z.transpose() * (Hbar_z * Pz_ * Hbar_z.transpose() + Vbar_z * Rbar_z * Vbar_z).inverse();
            Xz_ = Xz_ + Kbar_z * (Ybaro_ - hbar_z - Xz_[2]);
            Pz_ = Pz_ - Kbar_z * Hbar_z * Pz_;
            
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

            double u_x = msg.linear_acceleration.x; // u_x,k = Measured IMU x acceleration value in robot frame
            double u_y = msg.linear_acceleration.y; // u_y,k = Measured IMU y acceleration value in robot frame
            double u_z = msg.linear_acceleration.z; // u_z,k = Measured IMU z acceleration value in robot frame
            double u_a = msg.angular_velocity.z; // u_ψ,k = Measured IMU ψ velocity value in robot frame
            double G = params_.G; // acceleration due to gravity
            // double prev_x = Xx_[0], prev_xx = Xx_[1]; // xx = x_dot
            // double prev_y = Xy_[0], prev_yy = Xy_[1]; // yy = y_dot
            // double prev_z = Xz_[0], prev_zz = Xz_[1]; // zz = z_dot
            double yaw = Xa_[0];

            // --- Simplified Motion Model -> to derive EKF prediction formulas ---
            // Xx_[0] = prev_x + prev_xx * dt + 0.5 * dt * dt * (u_x * cos(yaw) - u_y * sin(yaw)); // x_k|k-1
            // Xx_[1] = prev_xx + dt * (u_x * cos(yaw) - u_y * sin(yaw)); // x_dot_k|k-1
            // Xy_[0] = prev_y + prev_yy * dt - 0.5 * dt * dt * (u_x * sin(yaw) + u_y * cos(yaw)); // y_k|k-1
            // Xy_[1] = prev_yy - dt * (u_x * sin(yaw) + u_y * cos(yaw)); // y_dot_k|k-1
            // Xz_[0] = prev_z + prev_zz * dt + 0.5 * dt * dt * (u_z - G); // z_k|k-1
            // Xz_[1] = prev_zz + dt * (u_z - G); // z_dot_k|k-1
            // Xa_[0] = yaw + dt * u_a; // ψ_k|k-1
            // Xa_[1] = u_a; // ψ_dot_k|k-1

            // EKF prediction for Xx_
            Eigen::Vector2d U_x = {u_x, u_y};
            Eigen::Matrix2d F_x, W_x, Q_x;
            F_x << 1, dt,
                   0, 1;
            W_x << 0.5 * dt * dt * cos(yaw), -0.5 * dt * dt * sin(yaw),
                        dt * cos(yaw),              -dt * sin(yaw);
            Q_x << params_.var_imu_x, 0,
                   0, params_.var_imu_y;
            Xx_ = F_x * Xx_ + W_x * U_x;
            Px_ = F_x * Px_ * F_x.transpose() + W_x * Q_x * W_x.transpose();

            // EKF prediction for Xy_
            Eigen::Vector2d U_y = {u_x, u_y};
            Eigen::Matrix2d F_y, W_y, Q_y;
            F_y << 1, dt,
                   0, 1;
            W_y << -0.5 * dt * dt * sin(yaw), -0.5 * dt * dt * cos(yaw),
                        -dt * sin(yaw),              -dt * cos(yaw);
            Q_y << params_.var_imu_x, 0,
                   0, params_.var_imu_y;
            Xy_ = F_y * Xy_ + W_y * U_y;
            Py_ = F_y * Py_ * F_y.transpose() + W_y * Q_y * W_y.transpose();

            // EKF prediction for Xz_ (already updated Xz Pz for baro correction)
            Eigen::Matrix3d F_z;
            Eigen::Vector3d W_z;
            F_z << 1, dt, 0,
                   0, 1, 0,
                   0, 0, 1;
            W_z << 0.5 * dt * dt,
                   dt,
                   0;
            Xz_ = F_z * Xz_ + W_z * (u_z - G);
            Pz_ = F_z * Pz_ * F_z.transpose() + W_z * params_.var_imu_z * W_z.transpose();

            // EKF prediction for Xa_
            Eigen::Matrix2d F_a;
            Eigen::Vector2d W_a;
            F_a << 1, 0,
                   0, 0;
            W_a << dt,
                   1;
            Xa_ = F_a * Xa_ + W_a * u_a;
            Pa_ = F_a * Pa_ * F_a.transpose() + W_a * params_.var_imu_a * W_a.transpose();

            // --- EOFIXME ---
        }
    };
}
