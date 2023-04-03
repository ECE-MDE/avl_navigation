//==============================================================================
// Autonomous Vehicle Library
//
// Description: Gyrocompassing algorithm using the UKF-M.
//==============================================================================

#include <avl_navigation/algorithm/gyrocompass_mukf.h>

// MUKF class
#include <avl_navigation/filter/mukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/matrix.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        phi_func
// Description: Retraction function for 15 state navigation algorithm.
// Arguments:   - x: ???.
//              - xi: ???.
// Returns:     ???.
//------------------------------------------------------------------------------
GyroState GyrocompassMukf::phi_func(GyroState x, VectorXd xi)
{
    x.g_b_i += xi.segment(0,3);
    x.C_n0_i = avl::so3_exp(xi.segment(3,3)) * x.C_n0_i;
    return x;
}

//------------------------------------------------------------------------------
// Name:        phi_inv_func
// Description: Inverse retraction function for 15 state navigation
//              algorithm.
// Arguments:   - x: ???.
//              - x_hat: ???.
// Returns:     ???.
//------------------------------------------------------------------------------
VectorXd GyrocompassMukf::phi_inv_func(GyroState x, GyroState x_hat)
{
    VectorXd xi(6);
    xi.segment(0,3) = x_hat.g_b_i - x.g_b_i;
    xi.segment(3,3) = avl::so3_log(x_hat.C_n0_i * x.C_n0_i.transpose());
    return xi;
}

//------------------------------------------------------------------------------
// Name:        f_func
// Description: Nonlinear state propagation function for 15 state inertial
//              navigation algorithm.
// Arguments:   - state: Current state.
//              - omega: Input.
//              - w: Input noise vector.
// Returns:     New state.
//------------------------------------------------------------------------------
GyroState GyrocompassMukf::f_func(GyroState x, GyroInput omega, VectorXd w)
{

    // Iterate b to i rotation matrix with gyro measurement
    Matrix3d I = Matrix3d::Identity();
    x.C_b_i = x.C_b_i*(I + avl::skew(omega.w_ib_b)*omega.dt);
    x.C_b_i = avl::orthonormalize(x.C_b_i);

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Update states

    // Calculate acceleration due to gravity
    Vector3d g_b_n = gravitational_acceleration(omega.p_b);

    // Calculate n to n0 rotation matrix based on time
    Matrix3d C_n_n0 = rotate_n_n0(omega.p_b, omega.p_b, omega.t);

    // Process noise components
    Vector3d w_gbi = w.segment(0,3);

    // Update g_b_i
    x.g_b_i = x.C_n0_i*C_n_n0*-g_b_n + w_gbi;

    // Update C_n0_i
    // NOTE: C_n0_i modeled as constant, no need to update

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Update n to b rotation matrix
    x.C_n_b = x.C_b_i.transpose() * x.C_n0_i * C_n_n0;

    return x;

}

//------------------------------------------------------------------------------
// Name:        h_func
// Description: Nonlinear measurement function.
// Arguments:   - x: Current state.
// Returns:     Masurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd GyrocompassMukf::h_func(GyroState x)
{
    return x.C_b_i.transpose() * x.g_b_i;
}

//------------------------------------------------------------------------------
// Name:        rotate_n_n0
// Description: Calculates the rotation matrix from the navigation frame to
//              the I frame.
// Arguments:   - t: Time in seconds.
// Returns:     Rotation matrix from the navigation frame to the I frame.
//------------------------------------------------------------------------------
Matrix3d GyrocompassMukf::rotate_n_n0(Vector3d p_b, Vector3d p_b0,
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
// Name:        GyrocompassMukf constructor
//------------------------------------------------------------------------------
GyrocompassMukf::GyrocompassMukf()
{

}

//------------------------------------------------------------------------------
// Name:        GyrocompassMukf destructor
//------------------------------------------------------------------------------
GyrocompassMukf::~GyrocompassMukf()
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
void GyrocompassMukf::init(GyroState x0, MatrixXd P0, MatrixXd Q, MatrixXd R,
    Vector3d p_b0)
{

    // Reset time
    t = 0;

    // Save initial position
    this->p_b0 = p_b0;

    // Save measurement covariance matrix
    this->R = R;

    // Initialize the MUKF
    mukf.init(x0, P0, Q, phi_func, phi_inv_func);

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
void GyrocompassMukf::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt,
    Vector3d p_b)
{

    // Update time
    t += dt;

    // Construct the input
    GyroInput input;
    input.dt = dt;
    input.t = t;
    input.p_b = p_b;
    input.w_ib_b = w_ib_b;

    // Execute the MUKF prediction step
    mukf.predict(f_func, input);

    // Execute the MUKF update step
    mukf.update(f_ib_b, h_func, R);

}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector.
// Returns:     Most recent state vector.
//------------------------------------------------------------------------------
VectorXd GyrocompassMukf::get_state()
{
    return mukf.get_state().to_vector();
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance matrix.
//------------------------------------------------------------------------------
VectorXd GyrocompassMukf::get_cov()
{
    // Add covariance of non-states
    VectorXd P_full = VectorXd::Zero(12);
    P_full.segment(6,6) = mukf.get_cov().diagonal();
    return P_full;
}
