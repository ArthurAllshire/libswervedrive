#include "libswervedrive/kinematic_model.h"

namespace swervedrive {

/**
 * @brief Construct a new Kinematic Model:: Kinematic Model object
 * 
 * @param chassis Chassis object containing the parameters for the KinematicModel to operate on.
 * @param k_beta The gain for wheel reconfiguration.
 */
KinematicModel::KinematicModel(const Chassis& chassis, double k_beta=1) : k_beta(k_beta), chassis_(chassis) {}

/**
 * @brief Compute the path to the desired chassis state and implement the control laws required to produce the motion.
 * Note that currently this method does not currently support beta_bounds in the chassis (TODO!).
 * 
 * @param lambda_desired 
 * @param mu_desired 
 * @param lamda_estimated 
 * @param mu_estimated 
 * @param k_backtrack 
 * @param k_lambda 
 * @param k_mu 
 * @return std::pair<Nu, Lambda> 
 */
std::pair<Nu, Lambda> KinematicModel::compute_chassis_motion(Lambda lambda_desired, double mu_desired,
    Lambda lamda_estimated, double mu_estimated,
    double k_backtrack, double k_lambda, double k_mu)
{
    // placeholder
    std::pair<Nu, Lambda> ret;
    return ret;
}

/**
 * @brief 
 * 
 * @param phi_dot 
 * @return double 
 */
double KinematicModel::compute_mu(Lambda, double phi_dot)
{
    // placeholder
    return 0.0;
}

/**
 * @brief 
 * 
 * @param lambda 
 * @param lambda_dot 
 * @param lambda_2dot 
 * @param mu 
 * @param mu_dot 
 * @param betas 
 * @return Motion 
 */
Motion KinematicModel::compute_actuator_motion(Lambda lambda, Lambda lambda_dot, Lambda lambda_2dot,
    double mu, double mu_dot, Eigen::VectorXd betas)
{
    // placeholder
    Motion motion;
    return motion;
}

/**
 * @brief 
 * 
 * @param betas_desired 
 * @param betas_estimated 
 * @return Motion 
 */
Motion KinematicModel::reconfigure_wheels(Eigen::VectorXd betas_desired, Eigen::VectorXd betas_estimated)
{
    // placeholder
    Motion motion;
    return motion;
}

/**
 * @brief 
 * 
 * @param mu 
 * @param dt 
 * @return Xi 
 */
Xi KinematicModel::compute_odometry(Lambda, double mu, double dt)
{
    // placeholder
    return Eigen::Vector3d::Zero(3);
}

/**
 * @brief 
 * 
 * @param phi_dot 
 * @return double 
 */
double KinematicModel::estimate_mu(Lambda, Eigen::VectorXd phi_dot)
{
    // placeholder
    return 0.0;

}

}
