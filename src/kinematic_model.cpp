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
Xi KinematicModel::compute_odometry(Lambda lambda, double mu, double dt)
{
    // placeholder
    return Eigen::Vector3d::Zero(3);
}

/**
 * @brief Find the rotational position of the robot about the ICR.
 * 
 * @param lambda the current estimated ICR of the robot.
 * @param phi_dot angular velocities of the wheels.
 * @return double the estimate of mu.
 */
double KinematicModel::estimate_mu(Lambda lambda, Eigen::VectorXd phi_dot)
{
    using namespace Eigen;
    using namespace swervedrive;
    // b divided by r of each module
    RowVectorXd b_on_r = chassis_.b_vector_.cwiseQuotient(chassis_.r_);
    MatrixXd b_on_r_block(3, chassis_.n_);
    b_on_r_block << b_on_r, b_on_r, b_on_r;
    b_on_r_block.transposeInPlace();

    std::pair<MatrixXd, MatrixXd> s_perp = chassis_.s_perp(lambda);
    RowVectorXd s2d = s_perp.second.transpose() * lambda;
    MatrixXd s2d_block(3, chassis_.n_);
    s2d_block << s2d, s2d, s2d;
    MatrixXd C = s_perp.first.cwiseQuotient(s2d_block);
    C.transposeInPlace();

    VectorXd D = (s_perp.second - chassis_.b_).transpose() * lambda;

    // build the matrix (from equation 19 in the control paper)
    MatrixXd k_lambda(chassis_.n_ + 1, 4);
    k_lambda.block(0, 0, 1, 4) << lambda.transpose(), 0.;
    k_lambda.block(1, 0, chassis_.n_, 3) << b_on_r_block.cwiseProduct(C);
    k_lambda.block(0, 3, chassis_.n_, 1) << D;
    
    VectorXd phi_dot_augmented(chassis_.n_ + 1);
    phi_dot_augmented << 0, phi_dot;

    // Solve the least squares system using the normal equations
    // (as solving A^TAx = A^Tb is equivalent to solving Ax=b)
    Vector4d solution = (k_lambda.transpose() * k_lambda).ldlt().solve(k_lambda.transpose()*phi_dot_augmented);

    return solution(3);

/*
        assert len(phi_dot.shape) == 2 and phi_dot.shape[1] == 1, phi_dot
        assert lmda_e.shape == (3,1), lmda_e

        # this requires solving equation (22) from the control paper, i think
        # we may need to look into whether this is valid for a system with no
        # wheel coupling
        s1_lmda, s2_lmda = self.s_perp(lmda_e)
        C = np.multiply(1.0 / s2_lmda.T.dot(lmda_e), s1_lmda.T)
        D = (s2_lmda - self.b_vector).T.dot(lmda_e) / self.r
        # Build the matrix
        K_lmda = np.block([[lmda_e.T, 0.0], [self.b / self.r * C, D]])
        phi_dot_augmented = np.block([[0], [phi_dot]])
        state = np.linalg.lstsq(K_lmda, phi_dot_augmented, rcond=None)[0]
        mu = state[-1, 0]
        return mu
        */
}

}
