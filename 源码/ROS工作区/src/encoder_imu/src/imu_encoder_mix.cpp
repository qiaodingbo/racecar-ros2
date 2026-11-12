#include <iostream>

// Include geometry messages
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Include navigation messages
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Include standard messages
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>

// Include sensor messages
#include <sensor_msgs/msg/imu.hpp>

// Include tf2 messages
#include <tf2_msgs/msg/tf_message.hpp>

// Include tf2 libraries
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Include message filters
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

// Include ROS2 client library

#define PI 3.14159
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;
using namespace nav_msgs;

rclcpp::Time current_time, last_time;
double x = 0.0;
double y = 0.0;
double s = 0.0;
double th = 0.0;
double vth = 0.0;
double th_init = 0.0;
static double theta_first = 0.0;
static bool flag = true;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoder_imu_pub;

// 从车身姿态获取航向角
double getYawFromPose(const sensor_msgs::msg::Imu::ConstSharedPtr &carPose) {
  sensor_msgs::msg::Imu imu = (*carPose);
  float x = imu.orientation.x;
  float y = imu.orientation.y;
  float z = imu.orientation.z;
  float w = imu.orientation.w;

  double tmp, yaw;
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}

double getRollFromPose(const sensor_msgs::msg::Imu::ConstSharedPtr &carPose) {
  sensor_msgs::msg::Imu imu = (*carPose);
  float x = imu.orientation.x;
  float y = imu.orientation.y;
  float z = imu.orientation.z;
  float w = imu.orientation.w;

  double roll, pitch, yaw;
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 quaternion(q);
  quaternion.getRPY(roll, pitch, yaw);

  return roll;
}

double getPitchFromPose(const sensor_msgs::msg::Imu::ConstSharedPtr &carPose) {
  sensor_msgs::msg::Imu imu = (*carPose);
  float x = imu.orientation.x;
  float y = imu.orientation.y;
  float z = imu.orientation.z;
  float w = imu.orientation.w;

  double roll, pitch, yaw;
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 quaternion(q);
  quaternion.getRPY(roll, pitch, yaw);

  return pitch;
}

void callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_data, const nav_msgs::msg::Odometry::ConstSharedPtr &speed) {
  if (flag == true) {
    theta_first = getYawFromPose(imu_data);
    flag = false;
  }

  if (flag == false) {
    double v = (*speed).twist.twist.linear.x;
    double theta = getYawFromPose(imu_data);
    double roll = getRollFromPose(imu_data);
    double pitch = getPitchFromPose(imu_data);
    theta = theta - theta_first;

    th_init = theta_first * 180 / PI;
    th = theta * 180 / PI; // 转换成角度

    double vx = v * cos(theta);
    double vy = v * sin(theta);
    current_time = rclcpp::Clock().now();
    double dt = (current_time - last_time).seconds();
    double delta_x = vx * dt;
    double delta_y = vy * dt;

    x += delta_x;
    y += delta_y;
    s = sqrt(x * x + y * y);

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "v    :%.4f", v);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "theta:%.4f", th);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "length:%.1f", s * 100);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, theta);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = 0;

    encoder_imu_pub->publish(odom);

    last_time = current_time;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("encoder_imu_mix");

  auto imu_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(node.get(), "/IMU_data", rmw_qos_profile_default);
  auto encoder_sub = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(node.get(), "/encoder", rmw_qos_profile_default);
  //RCLCPP_INFO(node->get_logger(), "-------");

  encoder_imu_pub = node->create_publisher<nav_msgs::msg::Odometry>("/encoder_imu_odom", 10);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, nav_msgs::msg::Odometry> MySyncPolicy;
  auto sync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *imu_sub, *encoder_sub);
  sync->registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}