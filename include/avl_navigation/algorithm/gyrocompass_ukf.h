//==============================================================================
// Autonomous Vehicle Library
//
// Description: Gyrocompassing algorithm using the UKF.
//==============================================================================

#ifndef GYROCOMPASS_UKF_H
#define GYROCOMPASS_UKF_H

// MUKF class
#include <avl_navigation/filter/ukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/matrix.h>

// Navigation input struct definition
struct GyroInput
{
    double dt;
    double t;
    Vector3d p_b;
    Vector3d w_ib_b;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class GyrocompassUkf
{

public:

    //--------------------------------------------------------------------------
    // Name:        f_func
    // Description: Nonlinear state propagation function for 15 state inertial
    //              navigation algorithm.
    // Arguments:   - state: Current state.
    //              - omega: Input.
    //              - w: Input noise vector.
    // Returns:     New state.
    //--------------------------------------------------------------------------
    static VectorXd f_func(VectorXd x, VectorXd w, GyroInput omega);

    //--------------------------------------------------------------------------
    // Name:        h_func
    // Description: Nonlinear measurement function.
    // Arguments:   - x: Current state.
    // Returns:     Masurement corresponding to current state.
    //--------------------------------------------------------------------------
    static VectorXd h_func(VectorXd x, VectorXd w);

    //--------------------------------------------------------------------------
    // Name:        rotate_n_0
    // Description: Calculates the rotation matrix from the navigation frame to
    //              the I frame.
    // Arguments:   - t: Time in seconds.
    // Returns:     Rotation matrix from the navigation frame to the I frame.
    //--------------------------------------------------------------------------
    static Matrix3d rotate_n_n0(Vector3d p_b, Vector3d p_b0, double t);

public:

    //--------------------------------------------------------------------------
    // Name:        GyrocompassUkf constructor
    //--------------------------------------------------------------------------
    GyrocompassUkf();

    //--------------------------------------------------------------------------
    // Name:        GyrocompassUkf destructor
    //--------------------------------------------------------------------------
    ~GyrocompassUkf();

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the filter with an initial state, covariance,
    //              and process noise covariance.
    // Arguments:   - x0: Initial state.
    //              - P0: Initial state covariance matrix. (6x6)
    //              - Q: Process noise covariance matrix. (3x3)
    //              - R: Measurement noise covariance matrix. (3x3)
    //              - p_b0: Initial curvilinear position. (3x1)[rad,rad,m]
    //--------------------------------------------------------------------------
    void init(VectorXd x0, MatrixXd P0, MatrixXd Q, MatrixXd R, Vector3d p_b0);

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Iterates the nav algorithm with IMU measurements of
    //              angular velocity and specific force with a time step.
    // Arguments:   - w_ib_b: Angular velocity of the body frame w.r.t. the
    //                inertial frame. (3x1)[rad/s]
    //              - f_ib_b: Specific force of the body frame w.r.t. the
    //                inertial frame. (3x1)[m/s^2]
    //              - dt: Time step in seconds.
    //              - p_b: Curvilinear position. (3x1)[rad,rad,m]
    //--------------------------------------------------------------------
    //--------------------------------------------------------------------------
    void iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt, Vector3d p_b);

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the most recent state vector.
    // Returns:     Most recent state vector.
    //--------------------------------------------------------------------------
    VectorXd get_state();

    //--------------------------------------------------------------------------
    // Name:        get_cov
    // Description: Gets the diagonal terms of the most recent state covariance
    //              matrix.
    // Returns:     Diagonal terms of the most recent state covariance matrix.
    //--------------------------------------------------------------------------
    VectorXd get_cov();

private:

    // MUKF instance
    Ukf ukf;

    // Process covariance matrix
    MatrixXd Q;

    // Measurement covariance matrix
    MatrixXd R;

    // Initial curvilinear position of the body/navigation frame
    Vector3d p_b0;

    // Time in seconds since start of iteration
    double t = 0.0;

    Matrix3d C_b_i;
    Matrix3d C_n_b;

};

#endif // GYROCOMPASS_UKF_H
