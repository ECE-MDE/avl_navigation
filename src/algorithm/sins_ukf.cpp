//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using a UKF to correct inertial
//              navigation estimates with aiding sensor data. Draws heavily from
//              "Principles of GNSS, Inertial and Multisensor Integrated
//              Navigation Systems" by Paul D. Groves.
//==============================================================================

#include <avl_navigation/algorithm/sins_ukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>
#include <avl_navigation/algorithm/nav_sensor_models.h>

// Util functions
#include <avl_core/util/matrix.h>
#include <avl_core/util/geo.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        SinsUkf constructor
//------------------------------------------------------------------------------
SinsUkf::SinsUkf()
{

}

//------------------------------------------------------------------------------
// Name:        SinsUkf destructor
//------------------------------------------------------------------------------
SinsUkf::~SinsUkf()
{

}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the filter with an initial state, covariance,
//              and process noise covariance.
// Arguments:   - x0: Initial state vector.
//              - P0: Initial state covariance matrix.
//              - P0: Process noise covariance matrix.
//------------------------------------------------------------------------------
void SinsUkf::init(VectorXd x0, MatrixXd P0, MatrixXd Q)
{

    this->Q = Q;

    // Initialize the MUKF
    std::vector<StateType> state_types = {
        STATE_ANGLE_RAD, STATE_ANGLE_RAD, STATE_ANGLE_RAD,
        STATE_STANDARD,  STATE_STANDARD,  STATE_STANDARD,
        STATE_STANDARD,  STATE_STANDARD,  STATE_STANDARD,
        STATE_STANDARD,  STATE_STANDARD,  STATE_STANDARD,
        STATE_STANDARD,  STATE_STANDARD,  STATE_STANDARD
    };

    ukf.init(x0, P0, state_types);

}

//------------------------------------------------------------------------------
// Name:        iterate
// Description: Iterates the nav algorithm with IMU measurements of
//              angular velocity and specific force with a time step.
// Arguments:   - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in rad/s.
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in m/s.
//              - dt: Time step in seconds.
//------------------------------------------------------------------------------
void SinsUkf::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Perform the UKF prediction step
    auto f = std::bind(&SinsUkf::f_ins_state, this, ph::_1, ph::_2, w_ib_b, f_ib_b, dt);
    ukf.predict(f, Q);

    // Check for state divergence
    if (std::isnan(get_state().norm()))
        throw std::runtime_error("iterate: divergence detected");

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo SinsUkf::process_body_velocity(Vector3d v_eb_b_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(h_body_vel_vec, ph::_1, ph::_2, l_bS_b, w_ib_b);
    MeasInfo info = ukf.update(h, R, v_eb_b_meas, threshold);
    return info;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo SinsUkf::process_depth(VectorXd depth_meas, double alt_surface,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(h_depth_vec, ph::_1, ph::_2, l_bS_b, alt_surface);
    MeasInfo info = ukf.update(h, R, depth_meas, threshold);
    return info;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo SinsUkf::process_range(VectorXd range_meas, Vector3d p_source,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(h_range_vec, ph::_1, ph::_2, l_bS_b, p_source);
    MeasInfo info = ukf.update(h, R, range_meas, threshold);
    return info;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo SinsUkf::process_position(Vector3d p_b_meas,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(h_pos_vec, ph::_1, ph::_2, l_bS_b);
    MeasInfo info = ukf.update(h, R, p_b_meas, threshold);
    return info;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo SinsUkf::process_gps(VectorXd gps_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(h_gps_vec, ph::_1, ph::_2, l_bS_b, w_ib_b);
    MeasInfo info = ukf.update(h, R, gps_meas, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        valid
// Description: Indicates whether the nav state estimate is valid.
// Returns:     True if nav state estimates are valid, false otherwise.
//------------------------------------------------------------------------------
bool SinsUkf::valid()
{
    return true;
}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector (15 x 1).
// Returns:     Most recent state vector (15 x 1).
//------------------------------------------------------------------------------
VectorXd SinsUkf::get_state()
{
    return ukf.get_state();
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance
//              matrix.
//------------------------------------------------------------------------------
VectorXd SinsUkf::get_cov()
{
    return ukf.get_cov().diagonal();
}

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
VectorXd  SinsUkf::f_ins_state(VectorXd x, VectorXd v,
    Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Process noise components
    Vector3d w_g =  v.segment(0,3);
    Vector3d w_a =  v.segment(3,3);
    // Vector3d w_bg = v.segment(9,3);
    // Vector3d w_ba = v.segment(9,3);

    // Extract the states from the state vector
    Matrix3d C_n_b =  avl::euler_to_matrix<double>(x.segment(0, 3));
    Vector3d v_eb_n = x.segment(3, 3);
    Vector3d p_b =    x.segment(6, 3);
    Vector3d b_g =    x.segment(9, 3);
    Vector3d b_a =    x.segment(12, 3);

    // Add the IMU process noise to the IMU measurements and subtract the bias
    // b_g += w_bg;
    // b_a += w_ba;
    w_ib_b += w_g - b_g;
    f_ib_b += w_a - b_a;

    // Transform the states to work with the navigation frame
    // INS update function
    Matrix3d C_b_n = C_n_b.transpose();

    f_ins_hp(C_b_n, v_eb_n, p_b, w_ib_b, f_ib_b, dt);

    // Convert back to body frame
    C_n_b = C_b_n.transpose();

    // Update the state vector
    x.segment(0, 3) =  avl::matrix_to_euler(C_n_b);
    x.segment(3, 3) =  v_eb_n;
    x.segment(6, 3) =  p_b;
    x.segment(9, 3) =  b_g;
    x.segment(12, 3) = b_a;

    return x;

}
