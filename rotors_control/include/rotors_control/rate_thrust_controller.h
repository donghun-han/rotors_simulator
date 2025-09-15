#ifndef ROTORS_CONTROL_RATE_THRUST_CONTROLLER_H
#define ROTORS_CONTROL_RATE_THRUST_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the roll pitch yawrate thrust controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class RateThrustControllerParameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RateThrustControllerParameters() : angular_rate_gain_(kDefaultAngularRateGain) {
        calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
    }

    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::Vector3d angular_rate_gain_;
    RotorConfiguration rotor_configuration_;
};

class RateThrustController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RateThrustController();
    ~RateThrustController();
    
    void InitializeParameters();
    void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

    void SetOdometry(const EigenOdometry& odometry);
    void SetRateThrust(const mav_msgs::EigenRateThrust& rate_thrust);

    RateThrustControllerParameters controller_parameters_;
    VehicleParameters vehicle_parameters_;

private:
    bool initialized_params_;
    bool controller_active_;

    Eigen::Vector3d normalized_angular_rate_gain_;
    Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

    mav_msgs::EigenRateThrust rate_thrust_;
    EigenOdometry odometry_;

    void ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const;
};

} // namespace rotors_control

#endif // ROTORS_CONTROL_RATE_THRUST_CONTROLLER_H
