#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void ControlCore::setPath(const nav_msgs::msg::Path& path) {
  path_ = path;
  has_path_ = true;
}

void ControlCore::setOdometry(const nav_msgs::msg::Odometry& odom) {
  odom_ = odom;
  has_odom_ = true;
}

void ControlCore::setLookaheadDistance(double distance) {
  if (distance > 0.0) {
    lookahead_distance_ = distance;
  }
}

void ControlCore::setGoalTolerance(double tolerance) {
  if (tolerance > 0.0) {
    goal_tolerance_ = tolerance;
  }
}

void ControlCore::setLinearSpeed(double speed) {
  if (speed >= 0.0) {
    linear_speed_ = speed;
  }
}

void ControlCore::setMaxAngularSpeed(double speed) {
  if (speed > 0.0) {
    max_angular_speed_ = speed;
  }
}

bool ControlCore::computeCommand(geometry_msgs::msg::Twist& cmd) {
  cmd = geometry_msgs::msg::Twist();

  if (!has_odom_) {
    return false;
  }

  if (!has_path_ || path_.poses.empty()) {
    return true;
  }

  const auto& robot_pose = odom_.pose.pose;
  const auto& robot_position = robot_pose.position;
  const auto& goal_position = path_.poses.back().pose.position;

  const double goal_dist = distance2D(robot_position, goal_position);
  if (goal_dist <= goal_tolerance_) {
    return true;
  }

  const auto lookahead_point = findLookaheadPoint(robot_position);
  if (!lookahead_point) {
    return true;
  }

  const double yaw = yawFromQuaternion(robot_pose.orientation);
  const double dx = lookahead_point->x - robot_position.x;
  const double dy = lookahead_point->y - robot_position.y;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double x_r = cos_yaw * dx + sin_yaw * dy;
  const double y_r = -sin_yaw * dx + cos_yaw * dy;

  if (x_r < 0.0) {
      // Logic: Stop moving forward, rotate towards the point
      cmd.linear.x = 0.0; 
      
      // If y_r is positive, point is to the Left -> Turn Left (+)
      // If y_r is negative, point is to the Right -> Turn Right (-)
      cmd.angular.z = (y_r > 0 ? 1.0 : -1.0) * max_angular_speed_;
      
      return true;
  }

  const double distance = std::hypot(x_r, y_r);

  if (distance <= 1e-6) {
    return false;
  }

  const double curvature = 2.0 * y_r / (distance * distance);
  cmd.linear.x = linear_speed_;
  cmd.angular.z = std::clamp(curvature * linear_speed_,
                             -max_angular_speed_, max_angular_speed_);

  return true;
}

double ControlCore::distance2D(const geometry_msgs::msg::Point& a,
                               const geometry_msgs::msg::Point& b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) const {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
    const geometry_msgs::msg::Point& robot_position) const {
  if (path_.poses.empty()) {
    return std::nullopt;
  }

  std::size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < path_.poses.size(); ++i) {
    const auto& pose = path_.poses[i].pose.position;
    const double dist = distance2D(robot_position, pose);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  for (std::size_t i = closest_idx; i < path_.poses.size(); ++i) {
    const auto& pose = path_.poses[i].pose.position;
    if (distance2D(robot_position, pose) >= lookahead_distance_) {
      return pose;
    }
  }

  return path_.poses.back().pose.position;
}

}  