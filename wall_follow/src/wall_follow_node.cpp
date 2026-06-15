#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10,
            std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Wall Follow Node Started");
    }

private:

    sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;

    // PID CONTROL PARAMS
    double kp = 1.2;
    double kd = 1.0;
    double ki = 0.0;
    
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    
    double desired_dist = 1.0;   // desired distance to wall (meters)
    double lookahead_dist = 1.0; // lookahead L
    double theta_deg = 30.0;     // angle between beams


    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        if (!latest_scan_) return 0.0;

        int index = (angle - latest_scan_->angle_min) /
                    latest_scan_->angle_increment;

        if (index < 0 || index >= (int)latest_scan_->ranges.size())
            return 0.0;

        double range = latest_scan_->ranges[index];

        if (std::isnan(range) || std::isinf(range))
            return 0.0;

        return range;
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        if (!latest_scan_) return 0.0;

        double theta = theta_deg * M_PI / 180.0;

        double b_angle = M_PI / 2.0;
        double a_angle = M_PI / 2.0 - theta;

        double a = get_range(nullptr, a_angle);
        double b = get_range(nullptr, b_angle);

        if (a == 0.0 || b == 0.0)
            return 0.0;

        double alpha = atan2(
            (a * cos(theta) - b),
            (a * sin(theta))
        );

        double Dt = b * cos(alpha);
        double Dt1 = Dt + lookahead_dist * sin(alpha);

        return Dt1 - desired_dist;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

        integral += error;
        double derivative = error - prev_error;

        double angle = kp * error + ki * integral + kd * derivative;

        prev_error = error;

        angle += servo_offset;
        angle = std::clamp(angle, -0.4189, 0.4189);

        // Step speed control
        double speed;
        double angle_deg = std::abs(angle) * 180.0 / M_PI;

        if (angle_deg <= 10.0)
            speed = 1.5;
        else if (angle_deg <= 20.0)
            speed = 1.0;
        else
            speed = 0.5;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = speed;

        drive_pub_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        // Store scan so helper functions can access it
        latest_scan_ = scan_msg;

        double error = get_error(nullptr, desired_dist);

        pid_control(error, 0.0);
    }
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}