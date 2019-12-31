#include "libswervedrive/kinematic_model.h"

using std::make_pair;
using std::pair;

namespace swervedrive
{
/**
 * @brief Construct a new Kinematic Model:: Kinematic Model object
 *
 * @param chassis Chassis object containing the parameters for the KinematicModel to operate on.
 */
KinematicModel::KinematicModel(Chassis& chassis) : chassis_(chassis)
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
pair<Nu, Lambda> KinematicModel::computeChassisMotion(Lambda lambda_desired, double mu_desired,
                                                      const Lambda& lambda_estimated, const double& mu_estimated,
                                                      const double& k_backtrack, const double& k_lambda,
                                                      const double& k_mu, const Eigen::VectorXd& beta)
{
  using namespace std;
  using namespace Eigen;

  // bound mu based on the ph_dot constraits
  auto mu_limits = muLimits(lambda_estimated, beta);
  mu_desired = max(min(mu_desired, mu_limits.second), mu_limits.first);

  // Check for lambda on singularity
  for (auto s : chassis_.singularities_)
  {
    if (lambda_desired.transpose().dot(s) > 0.99)
    {
      lambda_desired = lambda_estimated;
    }
  }

  auto dlambda =
      k_backtrack * k_lambda * (lambda_desired - (lambda_estimated.transpose().dot(lambda_desired) * lambda_estimated));

  auto d2lambda = pow(k_backtrack, 2) * pow(k_lambda, 2) *
                  ((lambda_estimated.transpose().dot(lambda_desired)) * lambda_desired - lambda_estimated);

  auto dmu = k_backtrack * k_mu * (mu_desired - mu_estimated);

  Nu nu_dot{ dlambda(0), dlambda(1), dlambda(2), dmu };

  return make_pair(nu_dot, d2lambda);
}

pair<double, double> KinematicModel::muLimits(const Lambda& lambda, const Eigen::VectorXd& beta)
{
  using namespace Eigen;
  using namespace std;

  VectorXd phi_dot_min(chassis_.n_), phi_dot_max(chassis_.n_);
  for (unsigned int i = 0; i < chassis_.n_; i++)
  {
    phi_dot_min(i) = chassis_.phi_dot_bounds_[i](0);
    phi_dot_max(i) = chassis_.phi_dot_bounds_[i](1);
  }
  // Use eq(25) to find mu limits
  auto s_perp = chassis_.sPerp(lambda, beta);
  auto f_lambda = chassis_.r_.cwiseQuotient((s_perp.second - chassis_.b_).transpose() * lambda);

  // Iterate through each wheel, because sometimes max phi_dot gives negative mu
  auto mu_min_phi = f_lambda.cwiseProduct(phi_dot_min);
  auto mu_max_phi = f_lambda.cwiseProduct(phi_dot_max);
  double mu_min = -1e10, mu_max = 1e10;
  for (unsigned int i = 0; i < chassis_.n_; i++)
  {
    mu_min = max(mu_min, min(mu_min_phi(i), mu_max_phi(i)));
    mu_max = min(mu_max, max(mu_min_phi(i), mu_max_phi(i)));
  }
  return make_pair(mu_min, mu_max);
}

/**
 * @brief
 *
 * @param lambda
 * @param lambda_dot
 * @param lambda_2dot
 * @param mu
 * @param mu_dot
 * @param beta
 * @return Motion
 */
Motion KinematicModel::computeActuatorMotion(const Lambda& lambda, const Lambda& lambda_dot, const Lambda& lambda_2dot,
                                             const double& mu, const double& mu_dot, const Eigen::VectorXd& beta)
{
  using namespace Eigen;
  using namespace swervedrive;

  if (chassis_.state_ == Chassis::STOPPING && abs(mu) < 1e-3)
  {
    // stopped, so we can reconfigure
    chassis_.state_ = Chassis::RECONFIGURING;
  }

  auto [s_perp1, s_perp2] = chassis_.sPerp(lambda, beta);

  VectorXd denom = s_perp2.transpose() * lambda;
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
  VectorXd beta_dot = -(s_perp1.transpose() * lambda_dot).cwiseQuotient(denom);

  // equation 15b
  VectorXd beta_2dot =
      -(2 * beta_dot.cwiseProduct(s_perp2.transpose() * lambda_dot) + s_perp1.transpose() * lambda_2dot)
           .cwiseQuotient(denom);

  // equation 15c
  VectorXd phi_dot = (mu * (s_perp2 - chassis_.b_).transpose() * lambda - chassis_.b_vector_.cwiseProduct(beta_dot))
                         .cwiseQuotient(chassis_.r_);

  // equation 15d
  VectorXd phi_2dot = (((s_perp2 - chassis_.b_).transpose() * (mu * lambda_dot + mu_dot * lambda)) -
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
 * @param k_beta The gain for wheel reconfiguration.
 * @return Motion
 */
Motion KinematicModel::reconfigureWheels(const Eigen::VectorXd& betas_desired, const Eigen::VectorXd& betas_estimated,
                                         const double& k_beta)
{
  using namespace Eigen;
  using namespace swervedrive;
  VectorXd displacement = chassis_.displacement(betas_estimated, betas_desired);

  VectorXd beta_dot = k_beta * displacement;
  if (displacement.norm() < 1.0 * M_PI / 180 * chassis_.n_)
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
double KinematicModel::estimateMu(const Lambda& lambda, const Eigen::VectorXd& phi_dot, const Eigen::VectorXd& beta)
{
  using namespace Eigen;
  using namespace swervedrive;
  // b divided by r of each module
  VectorXd b_on_r = chassis_.b_vector_.cwiseQuotient(chassis_.r_);
  MatrixXd b_on_r_block(chassis_.n_, 3);
  b_on_r_block << b_on_r, b_on_r, b_on_r;

  auto [s_perp1, s_perp2] = chassis_.sPerp(lambda, beta);
  VectorXd s2d = s_perp2.transpose() * lambda;
  MatrixXd s2d_block(chassis_.n_, 3);
  s2d_block << s2d, s2d, s2d;
  MatrixXd C = s_perp1.transpose().cwiseQuotient(s2d_block);
  VectorXd D = ((s_perp2 - chassis_.b_).transpose() * lambda).cwiseQuotient(chassis_.r_);

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
