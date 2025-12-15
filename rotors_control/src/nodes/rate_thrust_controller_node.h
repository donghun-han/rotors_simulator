#ifndef ROTORS_CONTROL_RATE_THRUST_CONTROLLER_NODE_H
#define ROTORS_CONTROL_RATE_THRUST_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/rate_thrust_controller.h"

namespace rotors_control {

class RateThrustControllerNode {
 public:
  RateThrustControllerNode();
  ~RateThrustControllerNode();

  void InitializeParams();
  void Publish();

 private:

  RateThrustController rate_thrust_controller_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_rate_thrust_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher motor_velocity_reference_pub_;

  void RateThrustCallback(
      const mav_msgs::RateThrustConstPtr& rate_thrust_reference_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
}

#endif // ROTORS_CONTROL_RATE_THRUST_CONTROLLER_NODE_H
