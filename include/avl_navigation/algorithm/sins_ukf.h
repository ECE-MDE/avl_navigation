//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using a UKF to correct inertial
//              navigation estimates with aiding sensor data. Draws heavily from
//              "Principles of GNSS, Inertial and Multisensor Integrated
//              Navigation Systems" by Paul D. Groves.
//==============================================================================

#ifndef SINS_UKF_H
#define SINS_UKF_H

// NavFilter base class
#include <avl_navigation/algorithm/nav_filter.h>

// Unscented Kalman filter class
#include <avl_navigation/filter/ukf.h>

// Alias for placeholders namespace
namespace ph = std::placeholders;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class SinsUkf : public NavFilter
{

public:

    //--------------------------------------------------------------------------
    // Name:        SinsUkf constructor
    //--------------------------------------------------------------------------
    SinsUkf();

    //--------------------------------------------------------------------------
    // Name:        SinsUkf destructor
    //--------------------------------------------------------------------------
    ~SinsUkf();

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
    //                inertial frame, expressed in the body frame in rad/s.
    //              - f_ib_b: Specific force of the body frame w.r.t. the
    //                inertial frame, expressed in the body frame in m/s.
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

private:

    // UKF instance for the error state UKF
    Ukf ukf;

    // Process noise covariance matrix;
    MatrixXd Q;

    // Altitude of sealevel in meters for depth measurement processing
    double alt_surface;

private:

    //------------------------------------------------------------------------------
    // Name:        f_ins_state
    // Description: Main inertial navigation function in state vector form. Updates
    //              attitude, velocity, and position by correcting and integrating
    //              the IMU measurements of angular velocity and specific force with
    //              a time step.
    // Arguments:   - x: Navigation state vector (15 x 1).
    //              - v: Noise vector (6 x 1)
    //              - w_ib_b: Angular velocity of the body frame w.r.t. the
    //                inertial frame, expressed in the body frame in rad/s.
    //              - f_ib_b: Specific force of the body frame w.r.t. the
    //                inertial frame, expressed in the body frame in m/s.
    //              - dt: Time step in seconds.
    //------------------------------------------------------------------------------
    VectorXd f_ins_state(VectorXd x, VectorXd v, Vector3d w_ib_b,
        Vector3d f_ib_b, double dt);

};

#endif // SINS_UKF_H
