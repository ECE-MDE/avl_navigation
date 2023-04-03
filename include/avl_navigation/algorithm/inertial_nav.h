//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides inertial navigation equation functions and other
//              navigation related helper functions, including sensor
//              measurement functions for use in Kalman filters. The algorithms
//              used are from "Principles of GNSS, Inertial, and Multisensor
//              Integrated Navigation Systems, Second Edition" by
//              Paul D. Groves. All equation numbers listed are from this book.
//              These functions assume the following error state vector:
//                  x_err = [dpsi_nb_n dv_eb_n dr_eb_n db_a db_g]^T
//              representing the error in attitude, velocity, NED cartesian
//              position, accelerometer bias, and gyroscope bias respectively.
//==============================================================================

#ifndef INERTIAL_NAV_H
#define INERTIAL_NAV_H

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                             DEFINES & CONSTANTS
//==============================================================================

// Earth ellipsoid parameters
const double R_0      = 6378137.0;           // WGS84 Equatorial radius in meters
const double R_P      = 6356752.31425;       // WGS84 Polar radius in meters
const double e        = 0.0818191908425;     // WGS84 eccentricity
const double f        = 1.0 / 298.257223563; // WGS84 flattening
const double mu       = 3.986004418E14;      // WGS84 Earth gravitational constant (m^3/s^2)
const double omega_ie = 7.292115E-5;         // Earth rotation rate (rad/s)

//==============================================================================
//							FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        cartesian_to_curvilinear
// Description: Calculates the cartesian to curvilinear transformation matrix
//              for transforming NED distance to lat/lon/alt distance.
// Arguments:   - p_b: Curvilinear position.
// Returns :    Cartesian to curvilinear transformation matrix.
//------------------------------------------------------------------------------
Matrix3d cartesian_to_curvilinear(const Vector3d& p_b);

//------------------------------------------------------------------------------
// Name:        curvilinear_to_cartesian
// Description: Calculates the curvilinear to cartesian transformation matrix
//              for transforming lat/lon/alt distance to NED distance.
// Arguments:   - p_b: Curvilinear position.
// Returns :    Curvilinear to cartesian transformation matrix.
//------------------------------------------------------------------------------
Matrix3d curvilinear_to_cartesian(const Vector3d& p_b);

//------------------------------------------------------------------------------
// Name:        body_to_horizontal
// Description: Calculates the body to horizontal frame rotation matrix using
//              roll and pitch.
// Arguments:   - roll: Roll in radians.
//              - pitch: Pitch in radians.
// Returns :    Body to horizontal frame rotation matrix.
//------------------------------------------------------------------------------
Matrix3d body_to_horizontal(const double& roll, const double& pitch);

//------------------------------------------------------------------------------
// Name:        attitude_from_imu
// Description: Calculates the roll, pitch, and yaw from a pair of accelerometer
//              and gyroscope measurements.
// Arguments:   - w_ib_b: IMU gyroscope measurement.
//              - f_ib_b: IMU accelerometer measurement.
// Returns :    Vector containing roll, pitch, and yaw in radians.
//------------------------------------------------------------------------------
Vector3d attitude_from_imu(const Vector3d& w_ib_b, const Vector3d& f_ib_b);

//------------------------------------------------------------------------------
// Name:        attitude_from_imu_mag
// Description: Calculates the roll, pitch, and yaw from accelerometer and
//              magnetometer measurements.
// Arguments:   - f_ib_b: Accelerometer measurement.
//              - m_b: Magnetometer measurement.
// Returns :    Vector containing roll, pitch, and yaw in radians.
//------------------------------------------------------------------------------
Vector3d attitude_from_imu_mag(const Vector3d& f_ib_b, const Vector3d& m_b);

//------------------------------------------------------------------------------
// Name:        transform_pos_b_to_n
// Description: Transforms a 3xN matrix of N body frame positions to navigation
//              frame positions.
// Arguments:   - p: 3xN matrix of N body frame positions in meters. Format is
//                     [xs; ys; zs]
//              - theta_n_b: Euler angles from navigation frame to body frame.
//              - p_b: Position of the body frame in the navigation frame.
// Returns :    Matrix of body frame positions transformed to the navigation
//              frame in radians/meters. Format is [lats; lons; alts]
//------------------------------------------------------------------------------
MatrixXd transform_pos_b_to_n(MatrixXd p, const Vector3d& theta_n_b,
    const Vector3d& p_b);

//------------------------------------------------------------------------------
// Name:        haversine_dist
// Description: Calculates the Haversine distance in meters between two points
//              on a sphere expressed in geodetic/spherical coordinates in
//              radians. The Haversine formula does not take into account
//              altitude.
// Arguments:   - pos1: Lat, lon, and alt of point 1 in radians and meters.
//              - pos2: Lat, lon, and alt of point 2 in radians and meters.
//              - degrees: True if lats and lons are in degrees, false if
//                radians.
// Returns :    Haversine distance between points 1 and 2 in meters.
//------------------------------------------------------------------------------
double haversine_dist(Vector3d pos1, Vector3d pos2, bool degrees=false);

//------------------------------------------------------------------------------
// Name:        linear_dist
// Description: Calculates the linear distance in meters between two points
//              expressed in geodetic/spherical coordinates.
// Arguments:   - pos1: Lat, lon, and alt of point 1 in radians and meters.
//              - pos1: Lat, lon, and alt of point 2 in radians and meters.
//              - degrees: True if lats and lons are in degrees, false if
//                radians.
// Returns :    Linear distance between points 1 and 2 in meters.
//------------------------------------------------------------------------------
double linear_dist(Vector3d pos1, Vector3d pos2, bool degrees=false);

//------------------------------------------------------------------------------
// Name:        radii_of_curvature
// Description: Calculates the Earth ellipsoid meridian and transverse radii of
//              curvature and the geocentric radius. (Equation 2.105,
//              Equation 2.106, and Equation 2.137)
// Arguments:   - p_b: Position (lat, lon, alt) in rad/rad/m.
//              - R_N: Reference to variable to store meridian radius of
//                curvature.
//              - R_E: Reference to variable to store transverse radius of
//                curvature.
//              - r_eS_e: Reference to variable to store geocentric radius.
//------------------------------------------------------------------------------
void radii_of_curvature(Vector3d p_b, double& R_N, double& R_E, double& r_eS_e);

//------------------------------------------------------------------------------
// Name:        earth_rotation_rate
// Description: Calculates rotation rate of the ECEF frame relative to the
//              ECI frame due to the earth's spin, expressed in the  navigation
//              frame in rad/s. (Equation 2.123)
// Arguments:   - p_b: Position (lat, lon, alt) in rad/rad/m.
// Returns :    Earth rotation rate in rad/s.
//------------------------------------------------------------------------------
Vector3d earth_rotation_rate(Vector3d p_b);

//------------------------------------------------------------------------------
// Name:        transport_rate
// Description: Calculates rotation rate of the NED frame w.r.t the ECEF frame,
//              expressed in the NED frame in rad/s. This rotation rate is due
//              to the movement of the NED frame across the surface of the
//              earth, and is called the transport rate. (Equation 5.44)
// Arguments:   - p_b: Position (lat, lon, alt) in rad/rad/m.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame.
// Returns :    Transport rate in rad/s.
//------------------------------------------------------------------------------
Vector3d transport_rate(Vector3d p_b, Vector3d v_eb_n);

//------------------------------------------------------------------------------
// Name:        somigliana_gravity
// Description: Calculates the acceleration due to gravity at the ellipsoid as a
//              function of latitude using the Somigliana model.
//              (Equation 2.134)
// Arguments:   - L_b: Latitude in radians.
// Returns :    Acceleration due to gravity at the ellipsoid in m/s^2.
//------------------------------------------------------------------------------
double somigliana_gravity(double L_b);

//------------------------------------------------------------------------------
// Name:        gravitational_acceleration
// Description: Calculates the linear acceleration of the body frame due to
//              gravity, expressed in the navigation frame in m/s^2. Uses the
//              gravity model presented in Section 2.4.7.
// Arguments:   - p_b: Position (lat, lon, alt) in rad/rad/m.
// Returns :    Gravitational acceleration in m/s^2.
//------------------------------------------------------------------------------
Vector3d gravitational_acceleration(Vector3d p_b);

//------------------------------------------------------------------------------
// Name:        rotate_n_n0
// Description: Calculates the rotation matrix from the navigation frame at time
//              t to the navigation frame at time t = 0.
// Arguments:   - p_b: Curvilinear position (lat [rad], lon [rad], alt [m])
//                at time t.
//              - p_b0: Curvilinear position (lat [rad], lon [rad], alt [m])
//                at time t = 0.
//              - t: Time in seconds.
// Returns:     Rotation matrix from the navigation frame at time t to the
//              navigation frame at time t = 0.
//------------------------------------------------------------------------------
Matrix3d rotate_n_n0(Vector3d p_b, Vector3d p_b0, double t);

//------------------------------------------------------------------------------
// Name:        f_ins_2d
// Description: 2D inertial navigation function. Updates attitude, velocity,
//              and position by correcting and integrating the IMU measurements
//              of angular velocity and specific force with a time step.
// Arguments:   - yaw: Yaw angle in radians.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - r_b: Cartesian position of the body frame in meters.
//              - w_ib_b_: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in rad/s.
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in m/s.
//              - dt: Time step in seconds.
//------------------------------------------------------------------------------
void f_ins_2d(double& yaw, Vector2d& v_eb_n, Vector2d& r_b,
    double w_ib_b_z, Vector2d f_ib_b, double dt);

//------------------------------------------------------------------------------
// Name:        f_ins_simple
// Description: Inertial navigation function. Updates attitude, velocity,
//              and position by correcting and integrating the IMU measurements
//              of angular velocity and specific force with a time step.
// Arguments:   - C_b_n: Body-to-NED frame transformation matrix.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - p_b: Curvilinear position of the body frame in radians
//                and meters.
//              - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in rad/s.
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in m/s.
//              - dt: Time step in seconds.
//------------------------------------------------------------------------------
void f_ins_simple(Matrix3d& C_b_n, Vector3d& v_eb_n, Vector3d& p_b,
    Vector3d w_ib_b, Vector3d f_ib_b, double dt);

//------------------------------------------------------------------------------
// Name:        f_ins
// Description: Inertial navigation function. Updates attitude, velocity,
//              and position by correcting and integrating the IMU measurements
//              of angular velocity and specific force with a time step.
// Arguments:   - C_b_n: Body-to-NED frame transformation matrix.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - p_b: Curvilinear position of the body frame in radians
//                and meters.
//              - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in rad/s.
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in m/s.
//              - dt: Time step in seconds.
//------------------------------------------------------------------------------
void f_ins(Matrix3d& C_b_n, Vector3d& v_eb_n, Vector3d& p_b,
    Vector3d w_ib_b, Vector3d f_ib_b, double dt);

//------------------------------------------------------------------------------
// Name:        f_ins_hp
// Description: High precision nertial navigation function. Updates attitude,
//              velocity, and position by correcting and integrating the IMU
//              measurements of angular velocity and specific force with a time
//              step.
// Arguments:   - C_b_n: Body-to-NED frame transformation matrix.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - p_b: Curvilinear position of the body frame in radians
//                and meters.
//              - w_ib_b: Angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in rad/s.
//              - f_ib_b: Specific force of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in m/s.
//              - dt: Time step in seconds.
//------------------------------------------------------------------------------
void f_ins_hp(Matrix3d& C_b_n, Vector3d& v_eb_n, Vector3d& p_b,
    Vector3d w_ib_b, Vector3d f_ib_b, double dt);

#endif // INERTIAL_NAV_H
