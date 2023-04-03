//==============================================================================
// Autonomous Vehicle Library
//
// Description: Gyrocompassing algorithm using the UKF-M.
//==============================================================================

#include <avl_navigation/algorithm/gyrocompass_ukf2.h>

// MUKF class
#include <avl_navigation/filter/ukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/matrix.h>

// Alias for placeholders namespace
namespace ph = std::placeholders;

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        f_func
// Description: Nonlinear state propagation function for 15 state inertial
//              navigation algorithm.
// Arguments:   - state: Current state.
//              - omega: Input.
//              - w: Input noise vector.
// Returns:     New state.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf2::f_func(VectorXd x, VectorXd w)
{

    // Propagate
    Vector3d a_b = x.segment(3,3);
    double dt = 0.01;
    double T_c = 2.0;
    a_b = exp(-dt/T_c) * a_b;

    x(0) = avl::wrap_to_pi(x(0));
    x(1) = avl::wrap_to_pi(x(1));
    x(2) = avl::wrap_to_2pi(x(2));
    x.segment(3,3) = a_b;
    return x + w;

}

//------------------------------------------------------------------------------
// Name:        h_func
// Description: Nonlinear measurement function.
// Arguments:   - x: Current state.
// Returns:     Masurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf2::h_func(VectorXd x, VectorXd w,
    MatrixXd C_b_i, MatrixXd C_n_n0, Vector3d g_b_n)
{

    // State components
    Vector3d theta_n0_i = x.segment(0,3);
    Vector3d a_b = x.segment(3,3);
    Matrix3d C_n0_i = avl::euler_to_matrix(theta_n0_i);
    Vector3d eta_a = w;

    // Calculate n to b rotation matrix
    Matrix3d C_n_b = C_b_i.transpose() * C_n0_i * C_n_n0;

    return C_n_b*-g_b_n + a_b + eta_a;

}

//------------------------------------------------------------------------------
// Name:        GyrocompassUkf2 constructor
//------------------------------------------------------------------------------
GyrocompassUkf2::GyrocompassUkf2()
{

}

//------------------------------------------------------------------------------
// Name:        GyrocompassUkf2 destructor
//------------------------------------------------------------------------------
GyrocompassUkf2::~GyrocompassUkf2()
{

}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the filter with an initial state, covariance,
//              and process noise covariance.
// Arguments:   - x0: Initial state.
//              - P0: Initial state covariance matrix. (6x6)
//              - Q: Process noise covariance matrix. (3x3)
//              - R: Measurement noise covariance matrix. (3x3)
//              - p_b0: Initial curvilinear position. (3x1)[rad,rad,m]
//------------------------------------------------------------------------------
void GyrocompassUkf2::init(VectorXd x0, MatrixXd P0, MatrixXd Q, MatrixXd R,
    Vector3d p_b0)
{

    // Save initial position
    this->p_b0 = p_b0;

    // Save measurement covariance matrix
    this->R = R;
    this->Q = Q;

    // Reset time
    t = 0;

    // Reset rotation matrices
    C_b_i = Matrix3d::Identity();
    C_n_b = Matrix3d::Identity();

    // Reset acceleration due to gravity
    g_b_n = gravitational_acceleration(p_b0);

    // Initialize the MUKF
    std::vector<StateType> state_types = {
        STATE_ANGLE_RAD, STATE_ANGLE_RAD, STATE_ANGLE_RAD,
        STATE_STANDARD, STATE_STANDARD, STATE_STANDARD };
    ukf.init(x0, P0, state_types);

}

//------------------------------------------------------------------------------
// Name:        iterate
// Description: Iterates the nav algorithm with IMU measurements of
//              angular velocity and specific force with a time step.
// Arguments:   - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame. (3x1)[rad/s]
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame. (3x1)[m/s^2]
//              - dt: Time step in seconds.
//              - p_b: Curvilinear position. (3x1)[rad,rad,m]
//------------------------------------------------------------------------------
void GyrocompassUkf2::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt,
    Vector3d p_b)
{

    // Update time
    t += dt;

    // Iterate b to i rotation matrix with gyro measurement
    Matrix3d I = Matrix3d::Identity();
    C_b_i = C_b_i*(I + avl::skew(w_ib_b)*dt);
    C_b_i = avl::orthonormalize(C_b_i);

    // Update acceleration due to gravity
    g_b_n = gravitational_acceleration(p_b);

    // Update n to n0 rotation matrix
    C_n_n0 = rotate_n_n0(p_b, p_b, t);

    // Execute the MUKF prediction step
    ukf.predict(f_func, Q);

    // Execute the MUKF update step
    auto h = std::bind(h_func, ph::_1, ph::_2, C_b_i, C_n_n0, g_b_n);
    ukf.update(h, R, f_ib_b);

    // Get state components
    VectorXd x = ukf.get_state();
    Vector3d theta_n0_i = x.segment(0,3);
    Matrix3d C_n0_i = avl::euler_to_matrix(theta_n0_i);

    // Update n to b rotation matrix
    C_n_b = C_b_i.transpose() * C_n0_i * C_n_n0;

}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector.
// Returns:     Most recent state vector.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf2::get_state()
{
    VectorXd x_full(9);
    x_full.segment(0,3) = avl::matrix_to_euler(C_b_i);
    x_full.segment(3,3) = avl::matrix_to_euler(C_n_b);
    x_full.segment(6,3) = ukf.get_state();
    return ukf.get_state();
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance matrix.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf2::get_cov()
{
    VectorXd P_full = VectorXd::Zero(9);
    P_full.segment(6,3) = ukf.get_cov().diagonal();
    return ukf.get_cov().diagonal();
}
