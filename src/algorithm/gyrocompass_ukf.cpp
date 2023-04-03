//==============================================================================
// Autonomous Vehicle Library
//
// Description: Gyrocompassing algorithm using the UKF-M.
//==============================================================================

#include <avl_navigation/algorithm/gyrocompass_ukf.h>

// MUKF class
#include <avl_navigation/filter/ukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/matrix.h>

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
VectorXd GyrocompassUkf::f_func(VectorXd x, VectorXd w, GyroInput omega)
{

    VectorXd x_prev = x;

    // Get state components
    Vector3d g_b_i = x.segment(0,3);
    Vector3d theta_n0_i = x.segment(3,3);
    Matrix3d C_n0_i = avl::euler_to_matrix(theta_n0_i);

    // Calculate acceleration due to gravity
    Vector3d g_b_n = gravitational_acceleration(omega.p_b);

    // Calculate n to n0 rotation matrix based on time
    Matrix3d C_n_n0 = rotate_n_n0(omega.p_b, omega.p_b, omega.t);

    // Process noise components
    Vector3d w_gbi = w.segment(0,3);

    // Update g_b_i
    g_b_i = C_n0_i*C_n_n0*-g_b_n + w_gbi;

    // Update C_n0_i
    // NOTE: C_n0_i modeled as constant, no need to update

    // Return updated states
    x.segment(0,3) = g_b_i;
    x.segment(3,3) = avl::matrix_to_euler(C_n0_i);
    x(3) = avl::wrap_to_pi(x(3));
    x(4) = avl::wrap_to_pi(x(4));
    x(5) = avl::wrap_to_2pi(x(5));
    return x;

}

//------------------------------------------------------------------------------
// Name:        h_func
// Description: Nonlinear measurement function.
// Arguments:   - x: Current state.
// Returns:     Masurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf::h_func(VectorXd x, VectorXd w)
{
    return x.segment(0,3) + w;
}

//------------------------------------------------------------------------------
// Name:        rotate_n_n0
// Description: Calculates the rotation matrix from the navigation frame to
//              the I frame.
// Arguments:   - t: Time in seconds.
// Returns:     Rotation matrix from the navigation frame to the I frame.
//------------------------------------------------------------------------------
Matrix3d GyrocompassUkf::rotate_n_n0(Vector3d p_b, Vector3d p_b0,
    double t)
{

    double Lt = p_b(0);
    double L0 = p_b0(0);

    Matrix3d C_n_n0;

    C_n_n0(0,0) =  cos(L0)*cos(Lt) + cos(omega_ie*t)*sin(L0)*sin(Lt);
    C_n_n0(0,1) =  sin(L0)*sin(omega_ie*t);
    C_n_n0(0,2) =  cos(Lt)*cos(omega_ie*t)*sin(L0) - cos(L0)*sin(Lt);

    C_n_n0(1,0) = -sin(Lt)*sin(omega_ie*t);
    C_n_n0(1,1) =  cos(omega_ie*t);
    C_n_n0(1,2) = -cos(Lt)*sin(omega_ie*t);

    C_n_n0(2,0) = -cos(Lt)*sin(L0) + cos(L0)*cos(omega_ie*t)*sin(Lt);
    C_n_n0(2,1) =  cos(L0)*sin(omega_ie*t);
    C_n_n0(2,2) =  cos(L0)*cos(Lt)*cos(omega_ie*t) + sin(L0)*sin(Lt);

    return C_n_n0;

}

//------------------------------------------------------------------------------
// Name:        GyrocompassUkf constructor
//------------------------------------------------------------------------------
GyrocompassUkf::GyrocompassUkf()
{

}

//------------------------------------------------------------------------------
// Name:        GyrocompassUkf destructor
//------------------------------------------------------------------------------
GyrocompassUkf::~GyrocompassUkf()
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
void GyrocompassUkf::init(VectorXd x0, MatrixXd P0, MatrixXd Q, MatrixXd R,
    Vector3d p_b0)
{

    // Reset time
    t = 0;

    // Reset rotation matrices
    C_b_i = Matrix3d::Identity();
    C_n_b = Matrix3d::Identity();

    // Save initial position
    this->p_b0 = p_b0;

    // Save measurement covariance matrix
    this->R = R;
    this->Q = Q;

    // Initialize the MUKF
    std::vector<StateType> state_types = {
        STATE_STANDARD,  STATE_STANDARD,  STATE_STANDARD,
        STATE_ANGLE_RAD, STATE_ANGLE_RAD, STATE_ANGLE_RAD};
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
void GyrocompassUkf::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt,
    Vector3d p_b)
{

    // Update time
    t += dt;

    // Iterate b to i rotation matrix with gyro measurement
    Matrix3d I = Matrix3d::Identity();
    C_b_i = C_b_i*(I + avl::skew(w_ib_b)*dt);
    C_b_i = avl::orthonormalize(C_b_i);

    // Construct the input
    GyroInput input;
    input.dt = dt;
    input.t = t;
    input.p_b = p_b;
    input.w_ib_b = w_ib_b;

    // Execute the MUKF prediction step
    auto f = std::bind(f_func, std::placeholders::_1, std::placeholders::_2, input);
    ukf.predict(f, Q);

    // Execute the MUKF update step
    ukf.update(h_func, R, C_b_i*f_ib_b);

    // Get state components
    VectorXd x = ukf.get_state();
    Vector3d theta_n0_i = x.segment(3,3);
    Matrix3d C_n0_i = avl::euler_to_matrix(theta_n0_i);

    // Update n to b rotation matrix
    Matrix3d C_n_n0 = rotate_n_n0(p_b, p_b, t);
    C_n_b = C_b_i.transpose() * C_n0_i * C_n_n0;

}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector.
// Returns:     Most recent state vector.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf::get_state()
{
    VectorXd x_full(12);
    x_full.segment(0,3) = avl::matrix_to_euler(C_b_i);
    x_full.segment(3,3) = avl::matrix_to_euler(C_n_b);
    x_full.segment(6,6) = ukf.get_state();
    return ukf.get_state();
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance matrix.
//------------------------------------------------------------------------------
VectorXd GyrocompassUkf::get_cov()
{
    VectorXd P_full = VectorXd::Zero(12);
    P_full.segment(6,6) = ukf.get_cov().diagonal();
    return ukf.get_cov().diagonal();
}
