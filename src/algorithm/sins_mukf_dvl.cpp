//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using a manifold UKF to correct
//              inertial navigation estimates with aiding sensor data. Draws
//              heavily from "Principles of GNSS, Inertial and Multisensor
//              Integrated Navigation Systems" by Paul D. Groves.
//==============================================================================

#include <avl_navigation/algorithm/sins_mukf_dvl.h>

// MUKF class
#include <avl_navigation/filter/mukf.h>

// Inertial navigation functions
#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/geo.h>
#include <avl_core/util/matrix.h>

// Alias for placeholders namespace
namespace ph = std::placeholders;

#include <iostream>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        SinsMukfDvl constructor
//------------------------------------------------------------------------------
SinsMukfDvl::SinsMukfDvl()
{

}

//------------------------------------------------------------------------------
// Name:        SinsMukfDvl destructor
//------------------------------------------------------------------------------
SinsMukfDvl::~SinsMukfDvl()
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
void SinsMukfDvl::init(VectorXd x0, MatrixXd P0, MatrixXd Q)
{

    // Construct the initial state
    NavStateDvl state0;
    state0.C_b_n =  avl::euler_to_matrix<double>(x0.segment(0,3)).transpose();
    state0.v_eb_n = x0.segment(3,3);
    state0.p_b =    x0.segment(6,3);
    state0.b_g =    x0.segment(9,3);
    state0.b_a =    x0.segment(12,3);

    // Initialize the MUKF
    mukf.init(state0, P0, Q, phi_func, phi_inv_func);

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
void SinsMukfDvl::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Construct the navigation input
    NavInputDvl input;
    input.dt = dt;
    input.w_ib_b = w_ib_b;
    input.f_ib_b = f_ib_b;
    input.T_c_g = 300.0;
    input.T_c_a = 300.0;
    input.T_c_v = 300.0;

    // Execute the MUKF prediction step
    mukf.predict(f_func, input);

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
void SinsMukfDvl::iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt, double T_c_g, double T_c_a, double T_c_v)
{

    // Construct the navigation input
    NavInputDvl input;
    input.dt = dt;
    input.w_ib_b = w_ib_b;
    input.f_ib_b = f_ib_b;
    input.T_c_g = T_c_g;
    input.T_c_a = T_c_a;
    input.T_c_v = T_c_v;

    // Execute the MUKF prediction step
    mukf.predict(f_func, input);

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
MeasInfo SinsMukfDvl::process_body_velocity(Vector3d v_eb_b_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_vel, ph::_1, w_ib_b, l_bS_b);
    MeasInfo info = mukf.update(v_eb_b_meas, h, R, threshold);
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
MeasInfo SinsMukfDvl::process_depth(VectorXd depth_meas, double alt_surface,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_depth, ph::_1, alt_surface, l_bS_b);
    MeasInfo info = mukf.update(depth_meas, h, R, threshold);
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
MeasInfo SinsMukfDvl::process_range(VectorXd range_meas, Vector3d p_source,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_range, ph::_1, p_source, l_bS_b);
    MeasInfo info = mukf.update(range_meas, h, R, threshold);
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
MeasInfo SinsMukfDvl::process_position(Vector3d p_b_meas,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_pos, ph::_1, l_bS_b);
    MeasInfo info = mukf.update(p_b_meas, h, R, threshold);
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
MeasInfo SinsMukfDvl::process_gps(VectorXd gps_meas, Vector3d w_ib_b,
    MatrixXd R, VectorXd threshold, Vector3d l_bS_b)
{
    auto h = std::bind(&h_func_gps, ph::_1, l_bS_b, w_ib_b);
    MeasInfo info = mukf.update(gps_meas, h, R, threshold);
    return info;
}

//------------------------------------------------------------------------------
// Name:        valid
// Description: Indicates whether the nav state estimate is valid.
// Returns:     True if nav state estimates are valid, false otherwise.
//------------------------------------------------------------------------------
bool SinsMukfDvl::valid()
{
    return true;
}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the most recent state vector (15 x 1).
// Returns:     Most recent state vector (15 x 1).
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::get_state()
{
    return mukf.get_state().to_vector();
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the diagonal terms of the most recent state covariance
//              matrix.
// Returns:     Diagonal terms of the most recent state covariance
//              matrix.
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::get_cov()
{
    return mukf.get_cov().diagonal();
}

//------------------------------------------------------------------------------
// Name:        phi_func
// Description: Retraction function for 15 state navigation algorithm.
// Arguments:   - x: ???.
//              - xi: ???.
// Returns:     ???.
//------------------------------------------------------------------------------
NavStateDvl SinsMukfDvl::phi_func(NavStateDvl x, VectorXd xi)
{
    x.C_b_n  = avl::so3_exp(xi.segment(0,3)) * x.C_b_n;
    x.v_eb_n += xi.segment(3,3);
    x.p_b    += xi.segment(6,3);
    x.b_g    += xi.segment(9,3);
    x.b_a    += xi.segment(12,3);
    x.b_v    += xi.segment(15,3);
    x.S_v    += xi.segment(18,3);
    return x;

    // Matrix3d dR = avl::so3_exp(xi.segment(0,3));
    // Matrix3d J = avl::so3_left_jacobian(xi.segment(0,3));
    // x_new.C_b_n =  x.C_b_n - dR;
    // x_new.v_eb_n = x_new.C_b_n*J*xi.segment(3,3) + x.v_eb_n;
    // x_new.p_b =    x_new.C_b_n*J*xi.segment(6,3) + x.p_b;
    // MatrixXd chi = avl::sek3_exp(xi.segment(0,9));

    // MatrixXd chi = state_to_se23(x) * avl::sek3_exp(xi.segment(0,9));
    // NavStateDvl x_new = se23_to_state(chi);
    // x_new.b_g =    x.b_g + xi.segment(9,3);
    // x_new.b_a =    x.b_a + xi.segment(12,3);
    // x_new.b_v =    x.b_v + xi.segment(15,3);
    // x_new.S_v =    x.S_v + xi.segment(18,3);



    // std::cout << "======================" << std::endl << std::endl;
    // std::cout << xi << std::endl << std::endl;
    // std::cout << avl::sek3_exp(xi.segment(0,9)) << std::endl << std::endl;
    // std::cout << chi << std::endl << std::endl;
    // // std::cout << xis_new << std::endl << std::endl;
    // std::cout << "======================" << std::endl << std::endl;

    // return x_new;

    // NavStateDvl x_new;
    // x_new.C_b_n =  x.C_b_n - dR;
    // x_new.v_eb_n = x_new.C_b_n*J*xi.segment(3,3) + x.v_eb_n;
    // x_new.p_b =    x_new.C_b_n*J*xi.segment(6,3) + x.p_b;
    // x_new.b_g =    x.b_g + xi.segment(9,3);
    // x_new.b_a =    x.b_a + xi.segment(12,3);
    // x_new.b_v =    x.b_v + xi.segment(15,3);
    // x_new.S_v =    x.S_v + xi.segment(18,3);
    // return x_new;

}

//------------------------------------------------------------------------------
// Name:        phi_inv_func
// Description: Inverse retraction function for 15 state navigation
//              algorithm.
// Arguments:   - x: ???.
//              - x_hat: ???.
// Returns:     ???.
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::phi_inv_func(NavStateDvl x, NavStateDvl x_hat)
{
    VectorXd xi(21);
    xi.segment(0,3)  = avl::so3_log(x_hat.C_b_n * x.C_b_n.transpose());
    xi.segment(3,3)  = x_hat.v_eb_n - x.v_eb_n;
    xi.segment(6,3)  = x_hat.p_b - x.p_b;
    xi.segment(9,3)  = x_hat.b_g - x.b_g;
    xi.segment(12,3) = x_hat.b_a - x.b_a;
    xi.segment(15,3) = x_hat.b_v - x.b_v;
    xi.segment(18,3) = x_hat.S_v - x.S_v;
    return xi;


    // MatrixXd chi = state_to_se23(x);
    // MatrixXd chi_hat = state_to_se23(x_hat);
    // VectorXd xi = VectorXd::Zero(21);
    // xi.segment(0,9) = avl::sek3_log(avl::sek3_inv(chi) * chi_hat);
    // xi.segment(9,3)  = x_hat.b_g - x.b_g;
    // xi.segment(12,3) = x_hat.b_a - x.b_a;
    // xi.segment(15,3) = x_hat.b_v - x.b_v;
    // xi.segment(18,3) = x_hat.S_v - x.S_v;
    // std::cout << chi << std::endl << std::endl;
    // std::cout << chi_hat << std::endl << std::endl;
    // std::cout << avl::sek3_inv(chi) * chi_hat << std::endl << std::endl;
    // std::cout << avl::sek3_log(avl::sek3_inv(chi) * chi_hat) << std::endl << std::endl;
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
NavStateDvl SinsMukfDvl::f_func(NavStateDvl x, NavInputDvl omega, VectorXd w)
{

    // Noise components
    Vector3d w_g =   w.segment(0,3);
    Vector3d w_a =   w.segment(3,3);
    Vector3d w_b_g = w.segment(6,3);
    Vector3d w_b_a = w.segment(9,3);
    Vector3d w_b_v = w.segment(12,3);
    Vector3d w_S_v = w.segment(15,3);

    // Propagate biases
    x.b_g = exp(-omega.dt/omega.T_c_g) * x.b_g + w_b_g;
    x.b_a = exp(-omega.dt/omega.T_c_a) * x.b_a + w_b_a;
    x.b_v = exp(-omega.dt/omega.T_c_v) * x.b_v + w_b_v;
    x.S_v = exp(-omega.dt/omega.T_c_v) * x.S_v + w_S_v;

    // Add bias and noise to IMU measurements
    omega.w_ib_b += -x.b_g + w_g;
    omega.f_ib_b += -x.b_a + w_a;

    // Propagate nav states
    f_ins_hp(x.C_b_n, x.v_eb_n, x.p_b, omega.w_ib_b, omega.f_ib_b, omega.dt);
    return x;

}

//------------------------------------------------------------------------------
// Name:        h_func_pos
// Description: Nonlinear measurement equation for a measurement of
//              curvilinear position with lever arm correction.
// Arguments:   - x: Current state.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
// Returns:     Position measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::h_func_pos(NavStateDvl x, Vector3d l_bS_b)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(x.p_b);

    // Construct the measurement vector with the sensor frame geodetic position
    // using the body frame position and the lever arm
    return x.p_b + T_rn_p * x.C_b_n * l_bS_b;

}

NavStateDvl SinsMukfDvl::se23_to_state(MatrixXd M)
{
    NavStateDvl state;
    state.C_b_n =  M.block(0,0,3,3);
    state.v_eb_n = M.block(0,3,3,1);
    state.p_b =    M.block(0,4,3,1);
    return state;
}

MatrixXd SinsMukfDvl::state_to_se23 (NavStateDvl state)
{
    MatrixXd M = MatrixXd::Identity(5,5);
    M.block(0,0,3,3) = state.C_b_n;
    M.block(0,3,3,1) = state.v_eb_n;
    M.block(0,4,3,1) = state.p_b;
    return M;
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::h_func_vel(NavStateDvl x, Vector3d l_bS_b, Vector3d w_ib_b)
{

    Matrix3d I = Matrix3d::Identity();
    Vector3d b_v = x.b_v;
    Matrix3d S_v = x.S_v.asDiagonal();

    // Navigation to body frame rotation matrix
    Matrix3d C_n_b = x.C_b_n.transpose();

    // Estimated body frame velocity
    Vector3d v_eb_b = C_n_b*x.v_eb_n;

    // Calculate the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the navigation frame
    Vector3d w_ie_n = earth_rotation_rate(x.p_b);

    // Calculate the angular rate of the sensor frame relative to the earth
    // frame, expresed in the sensor frame.
    Vector3d w_eS_S = w_ib_b - C_n_b*w_ie_n;

    // Construct the sensor frame velocity measurement according to the
    // measurement model
    Vector3d v_eS_S = (I + S_v) * (v_eb_b - w_eS_S.cross(l_bS_b)) + b_v;
    // Vector3d v_eS_S = v_eb_b - w_eS_S.cross(l_bS_b);
    return v_eS_S;

}

//------------------------------------------------------------------------------
// Name:        h_func_depth
// Description: Nonlinear measurement function for a measurement of
//              depth.
// Arguments:   - x: Current state.
//              - alt_surface: Altitude of the surface in meters.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
// Returns:     Depth measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::h_func_depth(NavStateDvl x, double alt_surface, Vector3d l_bS_b)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(x.p_b);

    // Calculate the sensor frame geodetic position from the body frame position
    // and the lever arm
    Vector3d p_S = x.p_b + T_rn_p * x.C_b_n * l_bS_b;

    // Construct the measurement vector with the depth as distance between the
    // surface alt and the sensor frame alt
    VectorXd y(1);
    y << alt_surface - p_S(2);
    return y;

}

//------------------------------------------------------------------------------
// Name:        h_func_range
// Description: Nonlinear measurement function for a measurement of range
//              from the sensor to a range measurement source location.
// Arguments:   - x: Current state.
//              - l_bS_b: Lever arm from the body frame to the sensor frame,
//                expressed in the body frame in meters.
// Returns:     Range measurement corresponding to current state.
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::h_func_range(NavStateDvl x, Vector3d p_source, Vector3d l_bS_b)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(x.p_b);

    // Calculate the sensor frame geodetic position from the body frame position
    // and the lever arm
    Vector3d p_S = x.p_b + T_rn_p * x.C_b_n * l_bS_b;

    // Check for NaN altitude. If one of them is NaN, do a 2D range calculation
    if (std::isnan(p_S(2)) || std::isnan(p_source(2)))
    {
        p_source(2) = 0.0;
        p_S(2) = 0.0;
    }

    // Construct the measurement vector with the range between the source
    // position and the sensor frame
    VectorXd y(1);
    y << linear_dist(p_S, p_source);
    return y;

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
VectorXd SinsMukfDvl::h_func_gps(NavStateDvl x, Vector3d l_bS_b, Vector3d w_ib_b)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(x.p_b);

    // Calculate the curvilinear position of the sensor frame
    Vector3d p_S = x.p_b + T_rn_p * x.C_b_n * l_bS_b;

    // Calculate the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the navigation frame
    Vector3d w_ie_n = earth_rotation_rate(x.p_b);

    // Calculate the velocity of the sensor frame relative to the earth frame,
    // expressed in the NED frame
    Vector3d v_eS_n = x.v_eb_n + x.C_b_n*(w_ib_b.cross(l_bS_b)) -
        avl::skew(w_ie_n)*x.C_b_n*l_bS_b;

    // Calculate the ground speed and track angle
    double ground_speed = v_eS_n.segment(0,2).norm();
    double track_angle = atan2(v_eS_n(1), v_eS_n(0));

    // Construct the measurement vector
    VectorXd y(5);
    y << p_S, ground_speed, track_angle;
    return y;

}
