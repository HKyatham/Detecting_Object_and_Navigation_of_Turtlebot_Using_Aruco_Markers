#include "solver.h"

float poseVector::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}//end of magnitude method.

Solver::Solver(tf2_ros::Buffer& tf_buffer) 
        : Node("solver_node", rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)), 
        tf_buffer_(tf_buffer) {
            // getting params from yaml file.
            getParameters();

            //Initialize the publisher.
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);

            //Initialize tf.
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

            // Initialize other variables.
            twist_msg_ = std::make_shared<geometry_msgs::msg::Twist>();
            m_ = 0;

            // Update dynamic transformations
            update_timer_for_transform_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust time interval as needed
            std::bind(&Solver::newTransform, this));
        }//end of constructor.

void Solver::getParameters() {
    rclcpp::Parameter ar0, ar1, ar2;
    ar0 = this->get_parameter("aruco_marker_0");
    ar1 = this->get_parameter("aruco_marker_1");
    ar2 = this->get_parameter("aruco_marker_2");

    parameters_.insert(std::pair<std::string, std::string>("aruco_marker_0", ar0.value_to_string().c_str()));
    parameters_.insert(std::pair<std::string, std::string>("aruco_marker_1", ar1.value_to_string().c_str()));
    parameters_.insert(std::pair<std::string, std::string>("aruco_marker_2", ar2.value_to_string().c_str()));
}//end of get parameters.

void Solver::arucoCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
    if(!msg->marker_ids.empty()) {
        poseVector pos = msg->poses[0];
        int index = 0;

        for (int i = 0; i<int(msg->poses.size()); i++) {
            const poseVector& newPos = msg->poses[i];
            if(newPos.magnitude() < pos.magnitude()) {
                pos = msg->poses[i];
                index = i;
            }
        }

        markerID = msg->marker_ids[index];

        geometry_msgs::msg::PoseStamped base_pose;
        geometry_msgs::msg::PoseStamped odom_pose;
        geometry_msgs::msg::PoseStamped aruco_pose;
        aruco_pose.header.frame_id = frame1;
        aruco_pose.pose = msg->poses[index];

        tf2::doTransform(aruco_pose, base_pose, camera_to_base_);
        tf2::doTransform(aruco_pose, odom_pose, transformStamped1);

        auto base = base_pose.pose;
        auto odom = odom_pose.pose;

        arucoPose_ = {base.position.x, base.position.y, base.position.z};
        aruco_odom_ = {odom.position.x, odom.position.y, odom.position.z};
    }//end of if case.
    solve();
}//end of arucocallback

void Solver::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odomPose_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    solve();
}// end of odomcallback

void Solver::batteryCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (!msg->part_poses.empty()) {
        auto color = msg->part_poses[0].part.color;

        geometry_msgs::msg::PoseStamped battery_pose;
        geometry_msgs::msg::PoseStamped battery_odom_pose;

        geometry_msgs::msg::Pose pose;

        battery_pose.header.frame_id = frame1;
        battery_pose.pose = msg->part_poses[0].pose;

        tf2::doTransform(battery_pose, battery_odom_pose, transformStamped2);

        battery_odom_[color] = battery_odom_pose.pose;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("solver_node"), "battery_odom_ size " << battery_odom_.size() << "\n");
    }// end of if case.
}// end of battery callback

void Solver::forward() {
    twist_msg_->linear.x = 0.3;
    twist_msg_->angular.z = arucoPose_[1] * 0.3; 
    publisher_->publish(*twist_msg_);
}// end of forward.

void Solver::turn() {
    twist_msg_->linear.x = 0.0;
    twist_msg_->angular.z = (-3.14159 / 20) * m_;
    publisher_->publish(*twist_msg_);
}//end of turn

void Solver::newTransform() {
    transformStamped1 = tf_buffer_.lookupTransform(odom_frame, frame1, tf2::TimePointZero, tf2::durationFromSec(1.0));
    transformStamped2 = tf_buffer_.lookupTransform(odom_frame, frame2, tf2::TimePointZero, tf2::durationFromSec(1.0));
    camera_to_base_ = tf_buffer_.lookupTransform(required_frame, frame1, tf2::TimePointZero, tf2::durationFromSec(4.0));
}//end of new transforms.

void Solver::solve() {
    // Solve code

    if (odomPose_.empty() || arucoPose_.empty() || m_ == 2) return;

    auto dist = euclidDistance(aruco_odom_, odomPose_);
    

    if (dist > 0.8 && m_ == 0) {
        // Keep driving
        forward();
    } //end of if case.
    
    else {
        if (m_ == 0) {
            turn_time_ = my_clock_.now();
            std::string direction = parameters_["aruco_marker_" + std::to_string(markerID)];

            if (direction == "right_90") {
                m_ = 1;
            } // end of if case
            if (direction == "left_90") {
                m_ = -1;
            } // end of if case
            if (direction == "end") {
                m_ = 2;
                twist_msg_->linear.x = 0;
                twist_msg_->angular.z = 0;
                publisher_->publish(*twist_msg_);

                for (const auto& battPose : battery_odom_) {
                    int key = battPose.first;
                    const auto& pose = battPose.second;
                    std::string color;

                    tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                    if (key == 1) {
                        color = "Green";
                    } else if (key == 2) {
                        color = "Blue";
                    } else if (key == 3) {
                        color = "Orange";
                    }

                    RCLCPP_INFO_STREAM(rclcpp::get_logger("solver_node"), color << " battery detected at xyz=[" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
                    << "] rpy=[" << roll << ", " << pitch << ", " << yaw << "]");

                } // end if for loop.
                return;
            }//end of if case.
        }//end of if case.

        turn();

        if ((m_ == 1 || m_ == -1) && my_clock_.now() - turn_time_ >= rclcpp::Duration::from_seconds(10)) {
            m_ = 0;
            twist_msg_->angular.z = 0.0;
            publisher_->publish(*twist_msg_);
        }//end of if case
    } // end of else case
}//end of solve method.

//Implementation for euclidian distance
double Solver::euclidDistance(const std::vector<double>& list1, const std::vector<double>& list2) {
    auto dist = sqrt(pow(list1[0] - list2[0], 2) + pow(list1[1] - list2[1], 2));
    return dist;
} // end of euclid distance

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); 
    tf2_ros::Buffer tf_buffer(std::make_shared<rclcpp::Clock>());
    auto node = std::make_shared<Solver>(tf_buffer);

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Initialize subscriptions
    auto subscription_aruco_ = node->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "/aruco_markers", qos, [node](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
            node->arucoCallback(msg);
        });

    auto subscription_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [node](const nav_msgs::msg::Odometry::SharedPtr msg) {
            node->odomCallback(msg);
        });

    auto subscription_battery_ = node->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/advanced_logical_camera/image", qos, [node](const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
            node->batteryCallback(msg);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} // end of main