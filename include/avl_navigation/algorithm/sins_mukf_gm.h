//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using a manifold UKF to correct
//              inertial navigation estimates with aiding sensor data. Draws
//              heavily from "Principles of GNSS, Inertial and Multisensor
//              Integrated Navigation Systems" by Paul D. Groves.
//==============================================================================

#ifndef SINS_MUKF_GM_H
#define SINS_MUKF_GM_H

// NavFilter base class
#include <avl_navigation/algorithm/nav_filter.h>

// MUKF class
#include <avl_navigation/filter/mukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/matrix.h>
#include <avl_core/util/geo.h>

// Alias for placeholders namespace
namespace ph = std::placeholders;

//==============================================================================
//                              STRUCT DEFINITION
//==============================================================================

// Navigation state struct definition
struct NavStateGm
{

    Matrix3d C_b_n =  Matrix3d::Identity();
    Vector3d v_eb_n = Vector3d::Zero();
    Vector3d p_b =    Vector3d::Zero();
    Vector3d b_g =    Vector3d::Zero();
    Vector3d b_a =    Vector3d::Zero();

    //---------------------------------------------------------------------------
    // Name:        to_vector
    // Description: Turns the nav state struct into a vector of states.
    // Returns:     Vector of nav states.
    //--------------------------------------------------------------------------
    VectorXd to_vector()
    {
        Matrix3d C_n_b = C_b_n.transpose();
        Vector3d theta_n_b = avl::matrix_to_euler(C_n_b);
        Vector3d v_eb_b = C_n_b*v_eb_n;
        VectorXd x(15);
        x << theta_n_b, v_eb_b, p_b, b_g, b_a;

        // Matrix3d C_n_b = C_b_n.transpose();
        // Vector3d theta_n_b = avl::matrix_to_euler(C_n_b);
        // VectorXd x(15);
        // x << theta_n_b, v_eb_n, p_b, b_g, b_a;
        return x;

    }

};

// Navigation input struct definition
struct NavInputGm
{
    double dt;
    Vector3d w_ib_b;
    Vector3d f_ib_b;
    double T_c_g;
    double T_c_a;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class SinsMukfGm : public NavFilter
{

public:

    //--------------------------------------------------------------------------
    // Name:        SinsMukfGm constructor
    //--------------------------------------------------------------------------
    SinsMukfGm();

    //--------------------------------------------------------------------------
    // Name:        SinsMukfGm destructor
    //--------------------------------------------------------------------------
    ~SinsMukfGm();

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the filter with an initial state, covariance,
    //              and process noise covariance.
    // Arguments:   - x0: Initial state vector.
    //              - P0: Initial state covariance matrix.
    //              - P0: Process noise covariance matrix.
    //--------------------------------------------------------------------------
    void init(VectorXd x0, MatrixXd P0, MatrixXd Q);

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Iterates the nav algorithm with IMU measurements of
    //              angular velocity and specific force with a time step.
    // Arguments:   - w_ib_b: Angular velocity of the body frame w.r.t. the
    //                inertial frame. (3x1)[rad/s]
    //              - f_ib_b: Specific force of the body frame w.r.t. the
    //                inertial frame. (3x1)[m/s^2]
    //              - dt: Time step in seconds.
    //--------------------------------------------------------------------------
    void iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt);

    //--------------------------------------------------------------------------
    // Name:        process_body_velocity
    // Description: Processes a body frame velocity measurement.
    // Arguments:   - v_eb_b_meas: Body frame velocity measurement. (3x1)[m/s]
    //              - w_ib_b: Angular velocity of the body frame w.r.t. the
    //                inertial frame, expressed in the body frame. (3x1)[rad/s]
    //              - R: Measurement noise covariance matrix. (3x3)[(m/s)^2 ]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    MeasInfo process_body_velocity(Vector3d v_eb_b_meas, Vector3d w_ib_b,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero());

    //--------------------------------------------------------------------------
    // Name:        process_depth
    // Description: Processes a depth measurement.
    // Arguments:   - depth_meas: Depth measurement. (1x1)[m]
    //              - alt_surface: Altitude of the surface. [m]
    //              - R: Measurement noise covariance matrix. (1x1)[m^2]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    MeasInfo process_depth(VectorXd depth_meas, double alt_surface,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero());

    //--------------------------------------------------------------------------
    // Name:        process_range
    // Description: Processes a range measurement.
    // Arguments:   - range_meas: Range measurement. (1x1)[m]
    //              - p_source: Lat, lon, and alt of position that range is
    //                measured to. (3x1)[rad,rad,m]
    //              - R: Measurement noise covariance matrix. (1x1)[m^2]
    //              - threshold: Rejection threshold. Range measurement will be
    //                rejected if innovation is greater than this multiple of
    //                the position stddev.
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    MeasInfo process_range(VectorXd range_meas, Vector3d p_source,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero());

    //--------------------------------------------------------------------------
    // Name:        process_position
    // Description: Processes a position measurement.
    // Arguments:   - p_b_meas: Curvilinear position measurement.
    //                (3x1)[rad,rad,m]
    //              - R: Measurement noise covariance matrix.
    //                (3x3)[(rad,rad,m)^2]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    MeasInfo process_position(Vector3d p_b_meas,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero());

    //--------------------------------------------------------------------------
    // Name:        process_gps
    // Description: Processes a GPS measurement with position and velocity.
    // Arguments:   - gps_meas: GPS measurement vector with elements:
    //                    {lat, lon, alt, vN, vE}
    //                and units:
    //                    {rad, rad, m, m/s, m/s}.
    //              - R: Measurement noise covariance matrix.
    //               (5x5)[(rad,rad,m,m/s,m/s)^2]
    //              - w_ib_b: Angular velocity of the body frame w.r.t. the
    //                inertial frame, expressed in the body frame. (3x1)[rad/s]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    MeasInfo process_gps(VectorXd gps_meas, Vector3d w_ib_b,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero());

    //--------------------------------------------------------------------------
    // Name:        valid
    // Description: Indicates whether the nav state estimate is valid.
    // Returns:     True if nav state estimates are valid, false otherwise.
    //--------------------------------------------------------------------------
    bool valid();

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the most recent state vector (15 x 1).
    // Returns:     Most recent state vector (15 x 1).
    //--------------------------------------------------------------------------
    VectorXd get_state();

    //--------------------------------------------------------------------------
    // Name:        get_cov
    // Description: Gets the diagonal terms of the most recent state covariance
    //              matrix.
    // Returns:     Diagonal terms of the most recent state covariance
    //              matrix.
    //--------------------------------------------------------------------------
    VectorXd get_cov();

public:

    //--------------------------------------------------------------------------
    // Name:        phi_func
    // Description: Retraction function for 15 state navigation algorithm.
    // Arguments:   - x: ???.
    //              - xi: ???.
    // Returns:     ???.
    //--------------------------------------------------------------------------
    static NavStateGm phi_func(NavStateGm x, VectorXd xi);

    //--------------------------------------------------------------------------
    // Name:        phi_inv_func
    // Description: Inverse retraction function for 15 state navigation
    //              algorithm.
    // Arguments:   - x: ???.
    //              - x_hat: ???.
    // Returns:     ???.
    //--------------------------------------------------------------------------
    static VectorXd phi_inv_func(NavStateGm x, NavStateGm x_hat);

    //--------------------------------------------------------------------------
    // Name:        f_func
    // Description: Nonlinear state propagation function for 15 state inertial
    //              navigation algorithm.
    // Arguments:   - state: Current state.
    //              - omega: Input.
    //              - w: Input noise vector.
    // Returns:     New state.
    //--------------------------------------------------------------------------
    static NavStateGm f_func(NavStateGm x, NavInputGm omega, VectorXd w);

    //--------------------------------------------------------------------------
    // Name:        h_func_pos
    // Description: Nonlinear measurement equation for a measurement of
    //              curvilinear position with lever arm correction.
    // Arguments:   - x: Current state.
    //              - l_bS_b: Lever arm from the body frame to the sensor frame,
    //                expressed in the body frame in meters.
    // Returns:     Position measurement corresponding to current state.
    //--------------------------------------------------------------------------
    static VectorXd h_func_pos(NavStateGm x, Vector3d l_bS_b);

    //--------------------------------------------------------------------------
    // Name:        h_func_vel
    // Description: Nonlinear measurement function for a measurement of linear
    //              velocity in the body frame with lever arm correction.
    // Arguments:   - x: Current state.
    //              - l_bS_b: Lever arm from the body frame to the sensor frame,
    //                expressed in the body frame in meters.
    //              - w_ib_b: Angular velocity of the body frame relative to the
    //                inertial frame, expressed in the body frame as measured by
    //                an IMU in rad/s.
    // Returns:     Body frame velocity measurement corresponding to current
    //              state.
    //--------------------------------------------------------------------------
    static VectorXd h_func_vel(NavStateGm x, Vector3d l_bS_b, Vector3d w_ib_b);

    //--------------------------------------------------------------------------
    // Name:        h_func_depth
    // Description: Nonlinear measurement function for a measurement of
    //              depth.
    // Arguments:   - x: Current state.
    //              - alt_surface: Altitude of the surface in meters.
    //              - l_bS_b: Lever arm from the body frame to the sensor frame,
    //                expressed in the body frame in meters.
    // Returns:     Depth measurement corresponding to current state.
    //--------------------------------------------------------------------------
    static VectorXd h_func_depth(NavStateGm x, double alt_surface, Vector3d l_bS_b);

    //--------------------------------------------------------------------------
    // Name:        h_func_range
    // Description: Nonlinear measurement function for a measurement of range
    //              from the sensor to a range measurement source location.
    // Arguments:   - x: Current state.
    //              - l_bS_b: Lever arm from the body frame to the sensor frame,
    //                expressed in the body frame in meters.
    // Returns:     Range measurement corresponding to current state.
    //--------------------------------------------------------------------------
    static VectorXd h_func_range(NavStateGm x, Vector3d p_source, Vector3d l_bS_b);

    //--------------------------------------------------------------------------
    // Name:        h_func_gps
    // Description: Nonlinear measurement equation for a GPS measurement with
    //              curvilinear position and NED velocity. Includes a lever arm
    //              correction.
    // Arguments:   - x: Current state.
    //              - l_bS_b: Lever arm from the body frame to the sensor frame,
    //                expressed in the body frame in meters.
    //              - w_ib_b: Angular velocity of the body frame relative to the
    //                inertial frame, expressed in the body frame as measured by
    //                an IMU in rad/s.
    // Returns:     Position measurement corresponding to current state.
    //--------------------------------------------------------------------------
    static VectorXd h_func_gps(NavStateGm x, Vector3d l_bS_b, Vector3d w_ib_b);

private:

    // MUKF instance
    Mukf<NavStateGm, NavInputGm> mukf;

    // MUKF initial covariance matrix
    NavStateGm state0;
    MatrixXd P0;
    MatrixXd Q;

};

#endif // SINS_MUKF_GM_H
