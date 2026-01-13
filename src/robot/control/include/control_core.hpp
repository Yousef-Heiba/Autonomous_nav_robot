#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void setPath(const nav_msgs::msg::Path& path);
    void setOdometry(const nav_msgs::msg::Odometry& odom);
    void setLookaheadDistance(double distance);
    void setGoalTolerance(double tolerance);
    void setLinearSpeed(double speed);
    void setMaxAngularSpeed(double speed);

    bool computeCommand(geometry_msgs::msg::Twist& cmd);
  
  private:
    double distance2D(const geometry_msgs::msg::Point& a,
                      const geometry_msgs::msg::Point& b) const;
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const;
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
        const geometry_msgs::msg::Point& robot_position) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_;
    bool has_path_ = false;
    bool has_odom_ = false;
    double lookahead_distance_ = 1.0;
    double goal_tolerance_ = 0.2;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.0;
};

} 

#endif 