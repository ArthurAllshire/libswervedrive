#include "libswervedrive/kinematic_model.h"

using std::make_pair;
using std::pair;

namespace swervedrive
{
/**
 * @brief Construct a new Kinematic Model:: Kinematic Model object
 *
 * @param chassis Chassis object containing the parameters for the KinematicModel to operate on.
 * @param k_beta The gain for wheel reconfiguration.
 */
KinematicModel::KinematicModel(Chassis& chassis, double k_beta = 1) : k_beta(k_beta), chassis_(chassis)
{
}

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
 * @return std::pair<Nu, Lambda> Nu_dot, lambda_2dot
 */
pair<Nu, Lambda> KinematicModel::computeChassisMotion(const Lambda& lambda_desired, const double& mu_desired,
                                                      const Lambda& lamda_estimated, const double& mu_estimated,
                                                      const double& k_backtrack, const double& k_lambda,
                                                      const double& k_mu)
{
  // placeholder
  Nu nu_dot{ 0, 0, 0, 0 };
  Lambda lambda_2dot{ 0, 0, 0 };

  return make_pair(nu_dot, lambda_2dot);
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
Motion KinematicModel::computeActuatorMotion(const Lambda& lambda, const Lambda& lambda_dot, const Lambda& lambda_2dot,
                                             const double& mu, const double& mu_dot)
{
  using namespace Eigen;
  using namespace swervedrive;

  if (chassis_.state_ == Chassis::STOPPING && abs(mu) < 1e-3)
  {
    // stopped, so we can reconfigure
    chassis_.state_ = Chassis::RECONFIGURING;
  }

  pair<MatrixXd, MatrixXd> s_perp = chassis_.sPerp(lambda);

  VectorXd denom = s_perp.second.transpose() * lambda;
  // don't divide by 0!
  denom = denom.unaryExpr([](double x) {
    if (abs(x) < 1e-20)
    {
      // preserve the sign of the entry
      return x >= 0 ? 1e-20 : -1e-20;
    }
    else
    {
      return x;
    }
  });

  // equation 15a
  VectorXd beta_dot = -(s_perp.first.transpose() * lambda_dot).cwiseQuotient(denom);

  // equation 15b
  VectorXd beta_2dot =
      -(2 * beta_dot.cwiseProduct(s_perp.second.transpose() * lambda_dot) + s_perp.first.transpose() * lambda_2dot)
           .cwiseQuotient(denom);

  // equation 15c
  VectorXd phi_dot =
      (mu * (s_perp.second - chassis_.b_).transpose() * lambda - chassis_.b_vector_.cwiseProduct(beta_dot))
          .cwiseQuotient(chassis_.r_);

  // equation 15d
  VectorXd phi_2dot = (((s_perp.second - chassis_.b_).transpose() * (mu * lambda_dot + mu_dot * lambda)) -
                       chassis_.b_vector_.cwiseProduct(beta_2dot))
                          .cwiseQuotient(chassis_.r_);

  Motion m;
  for (unsigned int i = 0; i < chassis_.n_; ++i)
  {
    ModuleMotion mm;
    mm.beta_dot = beta_dot[i];
    mm.beta_2dot = beta_2dot[i];
    mm.phi_dot = phi_dot[i];
    mm.phi_2dot = phi_2dot[i];
    m.push_back(mm);
  }
  return m;
}

/**
 * @brief
 *
 * @param betas_desired
 * @param betas_estimated
 * @return Motion
 */
Motion KinematicModel::reconfigureWheels(const Eigen::VectorXd& betas_desired, const Eigen::VectorXd& betas_estimated)
{
  using namespace Eigen;
  using namespace swervedrive;
  VectorXd displacement = chassis_.displacement(betas_estimated, betas_desired);
  VectorXd beta_dot = k_beta * displacement;
  if (displacement.norm() < M_PI / 180 * chassis_.n_)
  {
    chassis_.state_ = Chassis::RUNNING;
  }
  Motion m;
  // TODO: investigate some sort of motion profiling / smoothing here?
  for (unsigned int i = 0; i < chassis_.n_; ++i)
  {
    ModuleMotion mm;
    mm.beta_dot = beta_dot(i);
    mm.beta_2dot = 0;
    mm.phi_dot = 0;
    mm.phi_2dot = 0;
    m.push_back(mm);
  }
  return m;
}

/**
 * @brief Update an estimate of xi (twist position) based on the ICR/mu estimates.
 *
 * @param xi twist position to update
 * @param mu robot's position about the icr, as described in the relevant papers.
 * @param dt time since estimate was last updated
 * @return Xi the updated twist position
 */
Xi KinematicModel::computeOdometry(const Lambda& lambda, const double& mu, const double& dt)
{
  using namespace Eigen;
  using namespace swervedrive;
  VectorXd xi_dot(3);
  // equation 2
  xi_dot << lambda(1), -lambda(0), lambda(2);
  xi_dot *= mu;
  double theta = chassis_.xi_(2);

  MatrixXd m3(3, 3);
  m3 << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
  Xi xi_prime = chassis_.xi_ + m3 * xi_dot * dt;  // equation 24
  return xi_prime;
}

/**
 * @brief Find the rotational position of the robot about the ICR.
 *
 * @param lambda the current estimated ICR of the robot.
 * @param phi_dot angular velocities of the wheels.
 * @return double the estimate of mu.
 */
double KinematicModel::estimateMu(const Lambda& lambda, const Eigen::VectorXd& phi_dot)
{
  using namespace Eigen;
  using namespace swervedrive;
  // b divided by r of each module
  VectorXd b_on_r = chassis_.b_vector_.cwiseQuotient(chassis_.r_);
  MatrixXd b_on_r_block(chassis_.n_, 3);
  b_on_r_block << b_on_r, b_on_r, b_on_r;

  pair<MatrixXd, MatrixXd> s_perp = chassis_.sPerp(lambda);
  VectorXd s2d = s_perp.second.transpose() * lambda;
  MatrixXd s2d_block(chassis_.n_, 3);
  s2d_block << s2d, s2d, s2d;
  MatrixXd C = s_perp.first.transpose().cwiseQuotient(s2d_block);
  VectorXd D = ((s_perp.second - chassis_.b_).transpose() * lambda).cwiseQuotient(chassis_.r_);

  // build the matrix (from equation 19 in the control paper)
  MatrixXd k_lambda(chassis_.n_ + 1, 4);
  k_lambda.block(0, 0, 1, 4) << lambda.transpose(), 0.;
  k_lambda.block(1, 0, chassis_.n_, 3) << b_on_r_block.cwiseProduct(C);
  k_lambda.block(1, 3, chassis_.n_, 1) << D;

  VectorXd phi_dot_augmented(chassis_.n_ + 1);
  phi_dot_augmented << 0, phi_dot;

  // Solve the least squares system using the normal equations
  // (as solving A^TAx = A^Tb is equivalent to solving Ax=b)
  Vector4d solution = (k_lambda.transpose() * k_lambda).ldlt().solve(k_lambda.transpose() * phi_dot_augmented);
  return solution(3);

  /*
          assert len(phi_dot.shape) == 2 and phi_dot.shape[1] == 1, phi_dot
          assert lmda_e.shape == (3,1), lmda_e

          # this requires solving equation (22) from the control paper, i think
          # we may need to look into whether this is valid for a system with no
          # wheel coupling
          s1_lmda, s2_lmda = self.sPerp(lmda_e)
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

}  // namespace swervedrive
