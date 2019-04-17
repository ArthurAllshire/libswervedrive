#ifndef chassis_h
#define chassis_h

#include <Eigen/Dense>

#include <vector>

namespace libswervedrive {

// velocity bounds
using Bounds = std::vector<double>;
// position of the robot in terms of [x, y, theta]
using Epsilon = Eigen::Vector3d;
// icr of the robot in terms of the spherical parametrisation (u, v, w)
using Lambda = Eigen::Vector3d;

struct Chassis {
    Chassis(Eigen::VectorXd alpha,
        Eigen::VectorXd l,
        Eigen::VectorXd b,
        Eigen::VectorXd r,
        Bounds beta_bounds,
        Bounds beta_dot_bounds,
        Bounds beta_2dot_bounds,
        Bounds phi_dot_bounds,
        Bounds phi_2dot_bounds
        ):
        beta_dot_bounds(beta_dot_bounds), beta_2dot_bounds(beta_2dot_bounds),
        phi_dot_bounds(phi_dot_bounds), phi_2dot_bounds(phi_2dot_bounds) {
            // TODO: initialisation of matrices
        };
    
    // number of wheels
    int n;
    // array containing the angle to each of the modules
    // measured counter clockwise from the x-axis in radians
    Eigen::MatrixXd alpha;
    // horizontal distance from the origin to the axis of rotation of each wheel
    Eigen::MatrixXd l;
    //  horizontal distance from the axis of rotation of each module to its contact with the gound
    Eigen::MatrixXd b;
    // radii of the wheels (m)
    Eigen::MatrixXd r;
    // Min/max allowable value for steering angle, in rad.
    Bounds beta_bounds;
    // Min/max allowable value for rotation rate of modules, in rad/s
    Bounds beta_dot_bounds;
    // Min/max allowable value for the angular acceleration of the modules, in rad/s^2.
    Bounds beta_2dot_bounds; 
    // Min/max allowable value for rotation rate of module wheels, in rad/s
    Bounds phi_dot_bounds;
    // Min/max allowable value for the angular acceleration of the module wheels, in rad/s^2.
    Bounds phi_2dot_bounds;
};

}

#endif