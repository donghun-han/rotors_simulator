#ifndef ROTORS_CONTROL_RATE_THRUST_CONTROLLER_NODE_H
#define ROTORS_CONTROL_RATE_THRUST_CONTROLLER_NODE_H

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/Actuators.h>

#include "rotors_control/common.h"
#include "rotors_control/rate_thrust_controller.h"

namespace rotors_control {

class RateThrustControllerNode {
public:
    RateThrustControllerNode();
    ~RateThrustControllerNode();

    void InitializeParams();

private:
    void RateThrustCallback(const mav_msgs::RateThrust::ConstPtr& rate_thrust_reference_msg);
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // subscribers
    ros::Subscriber cmd_rate_thrust_sub_;
    ros::Subscriber odometry_sub_;

    ros::Publisher motor_velocity_reference_pub_;

    RateThrustController rate_thrust_controller_;
};

} // namespace rotors_control

#endif // ROTORS_CONTROL_RATE_THRUST_CONTROLLER_NODE_H
