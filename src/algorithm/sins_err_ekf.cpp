//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using an Error state EKF to
//              correct the navigation estimates of IMU integration. Draws
//              heavily from "Principles of GNSS, Inertial and Multisensor
//              Integrated Navigation Systems" by Paul D. Groves.
//==============================================================================

#include <avl_navigation/algorithm/sins_err_ekf.h>

// Util functions
#include <avl_core/util/matrix.h>

#include <iostream>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        SinsErrEkf constructor
//------------------------------------------------------------------------------
SinsErrEkf::SinsErrEkf()
{

}

//------------------------------------------------------------------------------
// Name:        SinsErrEkf destructor
//------------------------------------------------------------------------------
SinsErrEkf::~SinsErrEkf()
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
void SinsErrEkf::init(VectorXd x0, MatrixXd P0, MatrixXd Q)
{

    double cov_gyro = Q(0,0);
    double cov_accel = Q(3,3);
    double cov_bg = Q(6,6);
    double cov_ba = Q(9,9);
    double dt = 0.01;
    Q_err = Q_ins(cov_gyro, cov_accel, cov_bg, cov_ba, dt);

    // Construct the initial state
    Vector3d theta_n_b = x0.segment(0,3);
    C_b_n =  avl::euler_to_matrix(theta_n_b).transpose();
    v_eb_n = x0.segment(3,3);
    p_b =    x0.segment(6,3);
    b_g =    x0.segment(9,3);
    b_a =    x0.segment(12,3);

    // Initialize the error state EKF, starts with zero error
    ekf.init(VectorXd::Zero(NUM_STATES), P0);

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
//------------------------------------------------------------------------------
void SinsErrEkf::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Update the states from the IMU measurements
    w_ib_b -= b_g;
    f_ib_b -= b_a;
    f_ins(C_b_n, v_eb_n, p_b, w_ib_b, f_ib_b, dt);

    // Calculate the error state transition matrix
    MatrixXd Phi = phi_err_nC(C_b_n, v_eb_n, p_b, w_ib_b, f_ib_b, dt);

    // Perform the prediction step of the error state EKF
    ekf.predict(Phi, Q_err);

    // Correct the navigation estimate using the error state EKF states
    Vector3d dpsi_nb_n = ekf.x.segment(0,3);
    Vector3d dv_eb_n =   ekf.x.segment(3,3);
    Vector3d dr_b =      ekf.x.segment(6,3);
    Vector3d db_g =      ekf.x.segment(9,3);
    Vector3d db_a =      ekf.x.segment(12,3);
    correct_ins_nC(C_b_n, v_eb_n, p_b, b_g, b_a,
                   dpsi_nb_n, dv_eb_n, dr_b, db_g, db_a);

    // Set the error state EKF states to zero since we have just used
    // them to correct the navigation estimates
    ekf.x.setZero();

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
MeasInfo SinsErrEkf::process_body_velocity(Vector3d v_eb_b_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{

    // Rotate the body frame velocity measurement to the navigation frame
    Vector3d v_eb_n_meas = C_b_n * v_eb_b_meas;

    // Calculate the error from the estimated state and use it to update the
    // error state EKF
    VectorXd dz = v_eb_n_meas - v_eb_n;

    MeasInfo info = ekf.update(H_vel(C_b_n, v_eb_n), R, dz, threshold);
    info.y = v_eb_b_meas;
    info.y_bar = v_eb_n;
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
MeasInfo SinsErrEkf::process_depth(VectorXd depth_meas, double alt_surface,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{

    // Calculate the error from the estimated state and use it to update the
    // error state EKF
    VectorXd dz(1);
    dz << (alt_surface - depth_meas(0)) - p_b(2);
    return ekf.update(H_depth(), R, dz, threshold);

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
MeasInfo SinsErrEkf::process_range(VectorXd range_meas, Vector3d p_source,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    // Calculate the error from the estimated state and use it to update the
    // error state EKF
    VectorXd dz(1);
    dz << range_meas(0) - linear_dist(p_b, p_source);
    MatrixXd H = H_range(p_b, p_source);

    std::cout << "=========================" << std::endl << std::endl;
    std::cout << H << std::endl << std::endl;
    std::cout << dz << std::endl << std::endl;
    std::cout << R << std::endl << std::endl;
    std::cout << "=========================" << std::endl << std::endl;
    return ekf.update(H_range(p_b, p_source), R, dz, threshold);

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
MeasInfo SinsErrEkf::process_position(Vector3d p_b_meas,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{

    // Calculate the curvilinear to cartesian transformation matrix
    Matrix3d T_p_rn = curvilinear_to_cartesian(p_b);

    // Calculate error
    Vector3d dz = T_p_rn * (p_b_meas - p_b);

    // Update filter
    Matrix3d R2 = Vector3d({6.25, 6.25, 100}).asDiagonal();
    MeasInfo info = ekf.update(H_pos(), R2, dz, threshold);
    info.y = p_b_meas;
    info.y_bar = dz;
    return info;

}

//------------------------------------------------------------------------------
// Name:        process_gps
// Description: Processes a GPS measurement with position and velocity.
// Arguments:   - gps_meas: GPS measurement vector with elements:
//                    {lat, lon, alt, vN, vE}
//                and units:
//                    {rad, rad, m, m/s, m/s}.
//              - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame. (3x1)[rad/s]
//              - R: Measurement noise covariance matrix.
//               (5x5)[(rad,rad,m,m/s,m/s)^2]
//              - threshold: Measurement rejection threshold for each
//                measurement element. 0 disables measurement rejection.
//              - l_bS_b: Optional lever arm from the body frame to the
//                sensor frame, expressed in the body frame. (3x1)[m]
// Returns:     Measurement info struct.
//------------------------------------------------------------------------------
MeasInfo SinsErrEkf::process_gps(VectorXd gps_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{

    // Calculate the curvilinear to cartesian transformation matrix
    Matrix3d T_p_rn = curvilinear_to_cartesian(p_b);

    // Format the measurement vector
    Vector3d p_b_meas = gps_meas.segment(0,3);
    double ground_speed = gps_meas(3);
    double track_angle = gps_meas(4);

    double v_N = ground_speed * cos(track_angle);
    double v_E = ground_speed * sin(track_angle);

    VectorXd dz(5);
    dz.segment(0,3) = T_p_rn * (p_b_meas - p_b);
    dz(3) = v_N - v_eb_n(0);
    dz(4) = v_E - v_eb_n(1);

    MeasInfo info = ekf.update(H_gps(), R, dz, threshold);
    info.y = gps_meas;
    info.y_bar = dz;
    return info;

}

//------------------------------------------------------------------------------
// Name:        valid
// Description: Indicates whether the nav state estimate is valid.
// Returns:     True if nav state estimates are valid, false otherwise.
//------------------------------------------------------------------------------
bool SinsErrEkf::valid()
{
    return true;
}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector.
// Returns:     Most recent state vector. (15 x 1)
//------------------------------------------------------------------------------
VectorXd SinsErrEkf::get_state()
{

    Matrix3d C_n_b = C_b_n.transpose();
    Vector3d theta_n_b = avl::matrix_to_euler(C_n_b);
    Vector3d v_eb_b = C_n_b*v_eb_n;
    VectorXd x(NUM_STATES);
    x << theta_n_b, v_eb_b, p_b, b_g, b_a;
    return x;

}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance
//              matrix. (15x1)
//------------------------------------------------------------------------------
VectorXd SinsErrEkf::get_cov()
{
    return ekf.P.diagonal();
}
