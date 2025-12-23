#include "rotors_control/rate_thrust_controller.h"

namespace rotors_control {

RateThrustController::RateThrustController()
    : initialized_params_(false), controller_active_(false) {
    InitializeParameters();
}

RateThrustController::~RateThrustController() {}

void RateThrustController::InitializeParameters() {
    calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  
    // To make the tuning independent of the inertia matrix we divide here.
    normalized_angular_rate_gain_ = 
        controller_parameters_.angular_rate_gain_.transpose() * vehicle_parameters_.inertia_.inverse();

    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
    I(3, 3) = 1;
    angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  
    // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
    // A^{ \dagger} = A^T*(A*A^T)^{-1}
    angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
        * (controller_parameters_.allocation_matrix_
        * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  
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
    ComputeDesiredAngularAcc(&angular_acceleration);

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

void RateThrustController::SetRateThrust(const mav_msgs::EigenRateThrust& rate_thrust) {
    rate_thrust_ = rate_thrust;
    controller_active_ = true;
}

void RateThrustController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {
    assert(angular_acceleration);

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - rate_thrust_.angular_rates;
  *angular_acceleration = - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_);
}

} // namespace rotors_control
