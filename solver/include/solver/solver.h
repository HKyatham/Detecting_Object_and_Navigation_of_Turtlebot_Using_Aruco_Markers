#pragma once

#include <cmath>
#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct poseVector {
    float x,y,z; //x,y,z components of the vector.

    /**
     * @brief Construct a new pose Vector object.
     * 
     * @param pose A pose out of which the position is extracted.
     */
    poseVector(const geometry_msgs::msg::Pose& pose) : x(pose.position.x), y(pose.position.y), z(pose.position.z) {};

    /**
     * @brief calculates the magnitude of the pose vector.
     * 
     * @return * float Returns the magnitude of the pose vector.
     */
    float magnitude() const;
};


class Solver : public rclcpp::Node {
    public:

        /**
         * @brief Construct a new Solver object.
         * 
         * @param tf_buffer a reference to the tf2_ros::Buffer.
         */
        Solver(tf2_ros::Buffer& tf_buffer);

        /**
         * @brief Callback function for aruco markers.
         * 
         * @param msg Shared pointer for the Aruco markers msg.
         */
        void arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Callback function for the odometry.
         * 
         * @param msg Shared pointer for the odom msg.
         */
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        /**
         * @brief Callback function for the battery.
         * 
         * @param msg Shared pointer for the battery msg.
         */
        void batteryCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    private:

        /**
         * @brief Calculates the Euclidian distance between 2 vectors.
         * 
         * @param vect1 vector 1 w.r.t which the distance is calculated.
         * @param vect2 vector 2 w.r.t which the distance is calculated.
         * @return double Euclidian distance between the 2 vectors.
         */
        double euclidDistance(const std::vector<double>& vect1, const std::vector<double>& vect2);

        /**
         * @brief function to move forward.
         * 
         */
        void forward();

        /**
         * @brief function to turn the robot.
         * 
         */
        void turn();

        /**
         * @brief new transforms are updated based on the data from the sensors.
         * 
         */
        void newTransform();

        /**
         * @brief function to solve the maze.
         * 
         */
        void solve();

        /**
         * @brief Get the Parameters object and store it in our own variables.
         * 
         */
        void getParameters();

        //publisher and tf related variables.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        tf2_ros::Buffer& tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr update_timer_for_transform_;
        std::shared_ptr<geometry_msgs::msg::Twist> twist_msg_;
        
        //const variables used later in code.
        const std::string frame1 = "camera_rgb_optical_frame";
        const std::string frame2 = "logical_camera_link";
        const std::string required_frame = "base_footprint";
        const std::string odom_frame = "odom";

        //paramers variable to store the parameters.
        std::map<std::string, std::string> parameters_;

        //Transforms variable.
        geometry_msgs::msg::TransformStamped camera_to_base_;
        geometry_msgs::msg::TransformStamped transformStamped1;
        geometry_msgs::msg::TransformStamped transformStamped2;

        //vectors to store the pose values for odom, aruco, battery and arucoodom.
        std::vector<double> odomPose_{};
        std::vector<double> arucoPose_{};
        std::vector<double> aruco_odom_{};
        std::map<int, geometry_msgs::msg::Pose> battery_odom_;

        //Aruco marker Id.
        int markerID;
        
        //a variable to store the state of the robot, 0 - move forward, 1 - rotate right, -1 - rorate left, 2 - stop.
        int m_;
        
        //time variables.
        rclcpp::Time turn_time_;
        rclcpp::Clock my_clock_;
}; // end of Solver