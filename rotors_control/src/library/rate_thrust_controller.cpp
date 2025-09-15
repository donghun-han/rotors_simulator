#include "rotors_control/rate_thrust_controller.h"

namespace rotors_control {

RateThrustController::RateThrustController()
    : initialized_params_(false)
    , controller_active_(false) {
    InitializeParameters();
}

RateThrustController::~RateThrustController() {}

void RateThrustController::InitializeParameters() {
    calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  
    // To make the tuning independent of the inertia matrix we divide here.
    normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
        * vehicle_parameters_.inertia_.inverse();

    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
    I(3, 3) = 1;
    angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
    // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
    // A^{ \dagger} = A^T*(A*A^T)^{-1}
    angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
        * (controller_parameters_.allocation_matrix_ * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  
    initialized_params_ = true;
}

void RateThrustController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
    assert(rotor_velocities);
    assert(initialized_params_);

    rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
    // Return 0 velocities on all rotors, until the first command is received.
    if (!controller_active_) {
        *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
        return;
    }

    Eigen::Vector3d angular_acceleration;
    Eigen::Vector3d angular_rate_err = rate_thrust_.angular_rates - odometry_.angular_velocity;

    angular_acceleration = normalized_angular_rate_gain_.cwiseProduct(angular_rate_err);
    // ComputeDesiredAngularAcc(&angular_acceleration);

    Eigen::Vector4d angular_acceleration_thrust;
    angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
    angular_acceleration_thrust(3) = rate_thrust_.thrust.z();

    *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
    *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void RateThrustController::SetOdometry(const EigenOdometry& odometry) {
    odometry_ = odometry;
}

void RateThrustController::SetRateThrust(
    const mav_msgs::EigenRateThrust& rate_thrust) {
    rate_thrust_ = rate_thrust;
    controller_active_ = true;
}

// // Implementation from the T. Lee et al. paper
// // Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
// void RateThrustController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {
//   assert(angular_acceleration);

//   Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
//   double yaw = atan2(R(1, 0), R(0, 0));

//   // Get the desired rotation matrix.
//   Eigen::Matrix3d R_des;
//   R_des = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())  // yaw
//         * Eigen::AngleAxisd(roll_pitch_yawrate_thrust_.roll, Eigen::Vector3d::UnitX())  // roll
//         * Eigen::AngleAxisd(roll_pitch_yawrate_thrust_.pitch, Eigen::Vector3d::UnitY());  // pitch

//   // Angle error according to lee et al.
//   Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
//   Eigen::Vector3d angle_error;
//   vectorFromSkewMatrix(angle_error_matrix, &angle_error);

//   // TODO(burrimi) include angular rate references at some point.
//   Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
//   angular_rate_des[2] = roll_pitch_yawrate_thrust_.yaw_rate;

//   Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

//   *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
//                            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
//                            + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
// }
}
