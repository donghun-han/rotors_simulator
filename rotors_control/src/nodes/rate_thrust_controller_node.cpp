#include "rate_thrust_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

RateThrustControllerNode::RateThrustControllerNode() {
    InitializeParams();

    ros::NodeHandle nh;

    cmd_rate_thrust_sub_ = nh.subscribe(kDefaultCommandRateThrustTopic, 1, &RateThrustControllerNode::RateThrustCallback, this);
    odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 1, &RateThrustControllerNode::OdometryCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(kDefaultCommandMotorSpeedTopic, 1);
}

RateThrustControllerNode::~RateThrustControllerNode() { }

void RateThrustControllerNode::InitializeParams() {
    ros::NodeHandle pnh("~");

    // Read parameters from rosparam.
    GetRosParameter(pnh, "angular_rate_gain/x",
                    rate_thrust_controller_.controller_parameters_.angular_rate_gain_.x(),
                    &rate_thrust_controller_.controller_parameters_.angular_rate_gain_.x());
    GetRosParameter(pnh, "angular_rate_gain/y",
                    rate_thrust_controller_.controller_parameters_.angular_rate_gain_.y(),
                    &rate_thrust_controller_.controller_parameters_.angular_rate_gain_.y());
    GetRosParameter(pnh, "angular_rate_gain/z",
                    rate_thrust_controller_.controller_parameters_.angular_rate_gain_.z(),
                    &rate_thrust_controller_.controller_parameters_.angular_rate_gain_.z());
    GetVehicleParameters(pnh, &rate_thrust_controller_.vehicle_parameters_);
    rate_thrust_controller_.InitializeParameters();
}

void RateThrustControllerNode::RateThrustCallback(const mav_msgs::RateThrustConstPtr& rate_thrust_reference_msg) {
    mav_msgs::EigenRateThrust rate_thrust;
    mav_msgs::eigenRateThrustFromMsg(*rate_thrust_reference_msg, &rate_thrust);
    rate_thrust_controller_.SetRateThrust(rate_thrust);
}


void RateThrustControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("RateThrustController got first odometry message.");

    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    rate_thrust_controller_.SetOdometry(odometry);

    Eigen::VectorXd ref_rotor_velocities;
    rate_thrust_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    // Todo(ffurrer): Do this in the conversions header.
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    actuator_msg->header.stamp = odometry_msg->header.stamp;

    motor_velocity_reference_pub_.publish(actuator_msg);
}

} // namespace rotors_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "rate_thrust_controller_node");
    rotors_control::RateThrustControllerNode rate_thrust_controller_node;
    ros::spin();
    return 0;
}
