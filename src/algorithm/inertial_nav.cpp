//==============================================================================
// Autonomous Vehicle Library
//
// Description: Provides inertial navigation equation functions and other
//              navigation related helper functions, including sensor
//              measurement functions for use in Kalman filters. The algorithms
//              used are from "Principles of GNSS, Inertial, and Multisensor
//              Integrated Navigation Systems, Second Edition" by
//              Paul D. Groves. All equation numbers listed are from this book.
//==============================================================================

#include <avl_navigation/algorithm/inertial_nav.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/matrix.h>
using namespace avl;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        cartesian_to_curvilinear
// Description: Calculates the cartesian to curvilinear transformation matrix
//              for transforming a NED distance to lat/lon/alt distance.
// Arguments:   - p_b: Curvilinear position.
// Returns :    Cartesian to curvilinear transformation matrix.
//------------------------------------------------------------------------------
Matrix3d cartesian_to_curvilinear(const Vector3d& p_b)
{

    // Determine the radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Calculate the transformation matrix
    Matrix3d T_rn_p = Matrix3d::Zero();
    T_rn_p(0,0) = 1 / (R_N + p_b(2));
    T_rn_p(1,1) = 1 / ((R_E + p_b(2)) * cos(p_b(0)));
    T_rn_p(2,2) = -1;
    return T_rn_p;

}

//------------------------------------------------------------------------------
// Name:        curvilinear_to_cartesian
// Description: Calculates the curvilinear to cartesian transformation matrix
//              for transforming lat/lon/alt distance to NED distance.
// Arguments:   - p_b: Curvilinear position.
// Returns :    Curvilinear to cartesian transformation matrix.
//------------------------------------------------------------------------------
Matrix3d curvilinear_to_cartesian(const Vector3d& p_b)
{

    // Determine the radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Calculate the transformation matrix
    Matrix3d T_p_rn = Matrix3d::Zero();
    T_p_rn(0,0) = R_N + p_b(2);
    T_p_rn(1,1) = (R_E + p_b(2)) * cos(p_b(0));
    T_p_rn(2,2) = -1;
    return T_p_rn;

}

//------------------------------------------------------------------------------
// Name:        body_to_horizontal
// Description: Calculates the body to horizontal frame rotation matrix using
//              roll and pitch.
// Arguments:   - roll: Roll in radians.
//              - pitch: Pitch in radians.
// Returns :    Body to horizontal frame rotation matrix.
//------------------------------------------------------------------------------
Matrix3d body_to_horizontal(const double& roll, const double& pitch)
{
    Matrix3d R1, R2, R_b2h;
    R1 << 1.0, 0.0, 0.0, 0.0, cos(roll), sin(roll), 0.0, -sin(roll), cos(roll);
    R2 << cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0.0, sin(pitch), 0.0, cos(pitch);
    return R1*R2;
}

//------------------------------------------------------------------------------
// Name:        attitude_from_imu
// Description: Calculates the roll, pitch, and yaw from a pair of accelerometer
//              and gyroscope measurements.
// Arguments:   - w_ib_b: IMU gyroscope measurement.
//              - f_ib_b: IMU accelerometer measurement.
// Returns :    Vector containing roll, pitch, and yaw in radians.
//------------------------------------------------------------------------------
Vector3d attitude_from_imu(const Vector3d& w_ib_b, const Vector3d& f_ib_b)
{

    // Get components for ease of notation
    double wx = w_ib_b(0);
    double wy = w_ib_b(1);
    double wz = w_ib_b(2);
    double fx = f_ib_b(0);
    double fy = f_ib_b(1);
    double fz = f_ib_b(2);

    // Groves Equation 5.101
    double phi_nb =   atan2(-fy, -fz); // roll
    double theta_nb = atan2(fx, sqrt(fy*fy + fz*fz)); // pitch

    // Groves Equation 5.105
    double sin_psi_nb = -wy*cos(phi_nb) +
                         wz*sin(phi_nb);
    double cos_psi_nb =  wx*cos(theta_nb) +
                         wy*sin(phi_nb)*sin(theta_nb) +
                         wz*cos(phi_nb)*sin(theta_nb);
    double psi_nb = atan2(sin_psi_nb, cos_psi_nb); // yaw

    return {phi_nb, theta_nb, psi_nb};

}

//------------------------------------------------------------------------------
// Name:        attitude_from_imu_mag
// Description: Calculates the roll, pitch, and yaw from accelerometer and
//              magnetometer measurements.
// Arguments:   - f_ib_b: Accelerometer measurement.
//              - m_b: Magnetometer measurement.
// Returns :    Vector containing roll, pitch, and yaw in radians.
//------------------------------------------------------------------------------
Vector3d attitude_from_imu_mag(const Vector3d& f_ib_b, const Vector3d& m_b)
{

    // Get components for ease of notation
    double fx = f_ib_b(0);
    double fy = f_ib_b(1);
    double fz = f_ib_b(2);

    // Groves Equation 5.101
    double roll =  atan2(-fy, -fz);
    double pitch = atan2(fx, sqrt(fy*fy + fz*fz));

    // Rotate the magnetometer measurement into the horizontal frame
    Matrix3d R_b2h = body_to_horizontal(roll, pitch);
    Vector3d m_h = R_b2h*m_b;

    // Calculate yaw from magnetometer measurements
    double yaw = atan2(m_h(1), m_h(0));

    // Create and return the attitude vector
    Vector3d theta_n_b = {roll, pitch, yaw};
    return theta_n_b;

}

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
    const Vector3d& p_b)
{

    // Body frame to navigation frame rotation matrix
    Matrix3d C_n_b = avl::euler_to_matrix(theta_n_b);
    Matrix3d C_b_n = C_n_b.transpose();

    // Transform body frame beam positions to navigation frame positions
    Matrix3d T_rn_p = cartesian_to_curvilinear(p_b);
    return p_b.replicate(1, p.cols()) + T_rn_p*C_b_n*p;

}

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
double haversine_dist(Vector3d pos1, Vector3d pos2, bool degrees)
{

    // Convert degrees to radians if lats and lons are given in degrees
    if (degrees)
    {
        pos1[0] = avl::deg_to_rad(pos1[0]);
        pos1[1] = avl::deg_to_rad(pos1[1]);
        pos2[0] = avl::deg_to_rad(pos2[0]);
        pos2[1] = avl::deg_to_rad(pos2[1]);
    }

    double lat1 = pos1[0];
    double lon1 = pos1[1];
    double lat2 = pos2[0];
    double lon2 = pos2[1];
    double u, v;

    u = sin((lat2 - lat1) / 2);
    v = sin((lon2 - lon1) / 2);

    return 2.0 * R_0 * asin(sqrt(u * u + cos(lat1) * cos(lat2) * v * v));

}

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
double linear_dist(Vector3d pos1, Vector3d pos2, bool degrees)
{

    // Convert degrees to radians if lats and lons are given in degrees
    if (degrees)
    {
        pos1[0] = avl::deg_to_rad(pos1[0]);
        pos1[1] = avl::deg_to_rad(pos1[1]);
        pos2[0] = avl::deg_to_rad(pos2[0]);
        pos2[1] = avl::deg_to_rad(pos2[1]);
    }

    double lat1 = pos1[0];
    double lon1 = pos1[1];
    double alt1 = R_0 + pos1[2];
    double lat2 = pos2[0];
    double lon2 = pos2[1];
    double alt2 = R_0 + pos2[2];

    double X1 = alt1 * cos(lat1) * cos(lon1);
    double Y1 = alt1 * cos(lat1) * sin(lon1);
    double Z1 = alt1 * sin(lat1);

    double X2 = alt2 * cos(lat2) * cos(lon2);
    double Y2 = alt2 * cos(lat2) * sin(lon2);
    double Z2 = alt2 * sin(lat2);

    double dX = X2 - X1;
    double dY = Y2 - Y1;
    double dZ = Z2 - Z1;

    return sqrt(dX*dX + dY*dY + dZ*dZ);

}

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
void radii_of_curvature(Vector3d p_b, double& R_N, double& R_E, double& r_eS_e)
{

double L_b = p_b(0);

    // Calculate meridian radius of curvature (Equation 2.105)
    double sinsq_L = sin(L_b) * sin(L_b);
    double denom = 1.0 - e*e*sinsq_L;
    R_N = R_0 * (1.0 - e*e) / pow(denom, 1.5);

    // Calculate transverse radius of curvature (Equation 2.106)
    R_E = R_0 / sqrt(denom);

    // Calculate geocentric radius (Equation 2.137)
    r_eS_e = R_E * sqrt(cos(L_b)*cos(L_b) + (1-e*e)*(1-e*e)*sin(L_b)*sin(L_b));

}

//------------------------------------------------------------------------------
// Name:        earth_rotation_rate
// Description: Calculates rotation rate of the ECEF frame relative to the
//              ECI frame due to the earth's spin, expressed in the  navigation
//              frame in rad/s. (Equation 2.123)
// Arguments:   - p_b: Position (lat, lon, alt) in rad/rad/m.
// Returns :    Earth rotation rate in rad/s.
//------------------------------------------------------------------------------
Vector3d earth_rotation_rate(Vector3d p_b)
{

    double L_b = p_b(0);
    Vector3d omega_ie_n;
    omega_ie_n << omega_ie*cos(L_b), 0.0, -omega_ie*sin(L_b);
    return omega_ie_n;

}

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
Vector3d transport_rate(Vector3d p_b, Vector3d v_eb_n)
{

    double L_b = p_b(0);
    double h_b = p_b(2);

    // Determine the radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Calculate the transport rate (Equation 5.44)
    Vector3d omega_en_n;
    omega_en_n << v_eb_n(1) / (R_E + h_b),
                 -v_eb_n(0) / (R_N + h_b),
                 -v_eb_n(1) * tan(L_b) / (R_E + h_b);

    return omega_en_n;

}

//------------------------------------------------------------------------------
// Name:        somigliana_gravity
// Description: Calculates the acceleration due to gravity at the ellipsoid as a
//              function of latitude using the Somigliana model.
//              (Equation 2.134)
// Arguments:   - L_b: Latitude in radians.
// Returns :    Acceleration due to gravity at the ellipsoid in m/s^2.
//------------------------------------------------------------------------------
double somigliana_gravity(double L_b)
{
    double sinsq_L = sin(L_b) * sin(L_b);
    return 9.7803253359 * (1.0 + 0.001931853 * sinsq_L) / sqrt(1.0 - e*e * sinsq_L);
}

//------------------------------------------------------------------------------
// Name:        gravitational_acceleration
// Description: Calculates the linear acceleration of the body frame due to
//              gravity, expressed in the navigation frame in m/s^2. Uses the
//              gravity model presented in Section 2.4.7.
// Arguments:   - p_b: Position (lat, lon, alt) in rad/rad/m.
// Returns :    Gravitational acceleration in m/s^2.
//------------------------------------------------------------------------------
Vector3d gravitational_acceleration(Vector3d p_b)
{

    double L_b = p_b(0);
    double h_b = p_b(2);

    Vector3d g_b_n;

    // Calculate surface gravity using the Somigliana model
    double g_0 = somigliana_gravity(L_b);

    // Calculate north gravity using (2.140)
    g_b_n(0) = -8.08E-9 * h_b * sin(2 * L_b);

    // East gravity is zero
    g_b_n(1) = 0;

    // Calculate down gravity using (2.139)
    g_b_n(2) = g_0 * (1.0 - (2.0 / R_0) * (1.0 + f * (1.0 - 2.0 * sin(L_b) * sin(L_b)) +
        (omega_ie*omega_ie * R_0*R_0 * R_P / mu)) * h_b + (3.0 * h_b*h_b / (R_0*R_0)));

    return g_b_n;

}

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
Matrix3d rotate_n_n0(Vector3d p_b, Vector3d p_b0, double t)
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
    double w_ib_b_z, Vector2d f_ib_b, double dt)
{

    // Update yaw
    double yaw_dot = w_ib_b_z;
    yaw += yaw_dot * dt;

    Matrix2d C_b_n;
    C_b_n(0,0) =  cos(yaw);
    C_b_n(0,1) =  sin(yaw);
    C_b_n(1,0) = -sin(yaw);
    C_b_n(1,1) =  cos(yaw);

    // Update velocity
    Vector2d v_dot = C_b_n*f_ib_b;
    v_eb_n += v_dot * dt;

    // Update position
    Vector2d r_dot = v_eb_n;
    r_b += r_dot * dt;

}

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
    Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // =========================================================================
    // Preliminaries

    // Calculate gyro measurement skew-symmetric matrix
    Matrix3d W_ib_b = skew(w_ib_b);

    // Calculate the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the NED frame
    Matrix3d W_ie_n = skew(earth_rotation_rate(p_b));

    // Calculate the angular rate of the NED frame w.r.t the ECEF frame,
    // expressed in the NED frame. This is called the transort rate
    Matrix3d W_en_n = skew(transport_rate(p_b, v_eb_n));

    // Calculate the acceleration due to gravity expressed in the NED frame
    Vector3d g_b_n = gravitational_acceleration(p_b);

    // =========================================================================
    // Attitude Update

    // Update the attitude matrix (Equation 5.46)
    Matrix3d I = Matrix3d::Identity();
    C_b_n = C_b_n*(I + W_ib_b*dt) - (W_ie_n + W_en_n)*C_b_n*dt;

    // Re-orthonormalize the resulting rotation matrix (Equations 5.78 to 5.80)
    C_b_n = orthonormalize(C_b_n);

    // =========================================================================
    // Specific force frame transformation

    // Transform the specific force delta from the body frame to the NED frame
    // (Equation 5.47)
    Vector3d f_ib_n = C_b_n * f_ib_b;

    // =========================================================================
    // Velocity update

    // Update the velocity (Equation 5.54)
    v_eb_n = v_eb_n + ( f_ib_n + g_b_n - (W_en_n + 2*W_ie_n)*v_eb_n ) * dt;

    // =========================================================================
    // Curvilinear position update

    // Update the position (Equation 5.56)
    Matrix3d T_rn_p = cartesian_to_curvilinear(p_b);
    p_b = p_b + T_rn_p*v_eb_n*dt;

}

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
    Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Curvilinear position components
    double L_b =      p_b(0);
    double lambda_b = p_b(1);
    double h_b =      p_b(2);

    // =========================================================================
    // Preliminaries

    // Calculate gyro measurement skew-symmetric matrix
    Matrix3d W_ib_b = skew(w_ib_b);

    // Determine the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the NED frame
    Vector3d w_ie_n = earth_rotation_rate(p_b);
    Matrix3d W_ie_n = skew(w_ie_n);

    // Determine the angular rate of the NED frame w.r.t the ECEF frame,
    // expressed in the NED frame. This is called the transort rate
    Vector3d w_en_n = transport_rate(p_b, v_eb_n);
    Matrix3d W_en_n = skew(w_en_n);

    // Determine the acceleration due to gravity expressed in the NED frame
    Vector3d g_b_n = gravitational_acceleration(p_b);

    // =========================================================================
    // Attitude Update

    // Update the attitude matrix (Equation 5.46)
    Matrix3d C_b_n_old = C_b_n;
    Matrix3d I = Matrix3d::Identity();
    C_b_n = C_b_n * (I + W_ib_b * dt) - (W_ie_n + W_en_n) * C_b_n * dt;

    // Re-orthonormalize the resulting rotation matrix (Equations 5.78 to 5.80)
    // C_b_n = orthonormalize(C_b_n);

    // =========================================================================
    // Specific force frame transformation

    // Transform the specific force delta from the body frame to the NED frame
    // (Equation 5.47)
    Vector3d f_ib_n = 0.5 * (C_b_n_old + C_b_n) * f_ib_b;

    // =========================================================================
    // Velocity update

    // Update the velocity (Equation 5.54)
    Vector3d v_eb_n_old = v_eb_n;
    v_eb_n = v_eb_n + ( f_ib_n + g_b_n - (W_en_n + 2*W_ie_n)*v_eb_n ) * dt;

    // =========================================================================
    // Curvilinear position update

    // Calculate the meridian and transverse radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Update height (Equation 5.56)
    double h_b_old = h_b;
    h_b = h_b - 0.5*dt * (v_eb_n_old(2) + v_eb_n(2));
    p_b(2) = h_b;

    // Update latitude (Equation 5.56)
    double L_b_old = L_b;
    L_b = L_b + 0.5*dt * ( v_eb_n_old(0) / (R_N + h_b_old) +
                           v_eb_n(0) /     (R_N + h_b) );
    p_b(0) = L_b;

    // Save the old radius of curvature for use in integration
    double R_E_old = R_E;

    // Calculate the new meridian and transverse radii of curvature
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Update longitude (Equation 5.56)
    double term_old = v_eb_n_old(1) / ( (R_E_old + h_b_old) * cos(L_b_old) );
    double term_new = v_eb_n(1)     / ( (R_E     + h_b)     * cos(L_b) );
    lambda_b = lambda_b + 0.5*dt * (term_old + term_new);
    p_b(1) = lambda_b;


}

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
    Vector3d w_ib_b, Vector3d f_ib_b, double dt)
{

    // Curvilinear position components
    double L_b =      p_b(0);
    double lambda_b = p_b(1);
    double h_b =      p_b(2);

    // =========================================================================
    // Preliminaries

    // Identity matrix
    Matrix3d I = Matrix3d::Identity();

    // Calculate attitude increment, magnitude, and skew-symmetric matrix
    Vector3d a_ib_b = w_ib_b * dt;
    double mag_a = a_ib_b.norm();
    Matrix3d A_ib_b = skew(a_ib_b);
    Matrix3d W_ib_b = skew(w_ib_b);

    // Determine the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the NED frame
    Vector3d w_ie_n = earth_rotation_rate(p_b);
    Matrix3d W_ie_n = skew(w_ie_n);

    // Determine the angular rate of the NED frame w.r.t the ECEF frame,
    // expressed in the NED frame. This is called the transort rate
    Vector3d w_en_n = transport_rate(p_b, v_eb_n);
    Matrix3d W_en_n = skew(w_en_n);

    // Determine the acceleration due to gravity expressed in the NED frame
    Vector3d g_b_n = gravitational_acceleration(p_b);

    // =========================================================================
    // Specific force frame transformation

    // Calculate the average rotation matrix change over the integration
    // period using either the precision equation or the approximate
    // equation based on the magnitude of alpha. (Precision: Equations 5.84 and 5.86,
    // Approximate: 5.46)
    Matrix3d C_bb_bm;
    if (mag_a > 1.0e-8)
    {
        Matrix3d term1 = (1.0 - cos(mag_a))/(mag_a*mag_a)*A_ib_b;
        Matrix3d term2 = (1.0/(mag_a*mag_a))*(1.0 - sin(mag_a)/mag_a)*(A_ib_b*A_ib_b);
        C_bb_bm = I + term1 + term2;
    }
    else
    {
        C_bb_bm = I + W_ib_b*dt;
    }

    // Calculate the average body frame to navigation frame rotation matrix
    // over the integration period (Equation 5.86). Note that the example
    // code that comes with the book appears to be missing dt term.
    Matrix3d Cbar_b_n = C_b_n*C_bb_bm - 0.5*(W_ie_n + W_en_n)*C_b_n*dt;

    // Transform the specific force delta from the body frame to the NED frame
    // (Equation 5.86)
    Vector3d f_ib_n = Cbar_b_n * f_ib_b;

    // =========================================================================
    // Velocity update

    // Save the old velocity for use in integration
    Vector3d v_eb_n_old = v_eb_n;

    // Update the velocity with the specific force delta (Equation 5.54)
    v_eb_n = v_eb_n + ( f_ib_n + g_b_n - (W_en_n + 2.0*W_ie_n)*v_eb_n ) * dt;

    // =========================================================================
    // Curvilinear position update

    // Calculate the meridian and transverse radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Save the old position for use in integration
    double L_b_old = L_b;
    double h_b_old = h_b;

    // Update height (Equation 5.56)
    h_b = h_b - 0.5*dt * (v_eb_n_old(2) + v_eb_n(2));
    p_b(2) = h_b;

    // Update latitude (Equation 5.56)
    L_b = L_b + 0.5*dt * (v_eb_n_old(0)/(R_N + h_b) + v_eb_n(0) / (R_N + h_b));
    p_b(0) = L_b;

    // Save the old radius of curvature for use in integration
    double R_E_old = R_E;

    // Calculate the new meridian and transverse radii of curvature
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Update longitude (Equation 5.56)
    double term_old = v_eb_n_old(1) / ((R_E_old + h_b_old) * cos(L_b_old));
    double term_new = v_eb_n(1) / ((R_E + h_b) * cos(L_b));
    lambda_b = lambda_b + dt/2.0 * (term_old + term_new);
    p_b(1) = lambda_b;

    // =========================================================================
    // Attitude update

    // Save the old transport rate
    Matrix3d W_en_n_old = W_en_n;

    // Determine the new transport rate using the new location and new velocity
    w_en_n = transport_rate(p_b, v_eb_n);
    W_en_n = skew(w_en_n);

    // Calculate the transformation matrix from the new attitude w.r.t. an
    // inertial frame to the old using Rodrigues' formula (Equation 5.73)
    // NOTE: This doesn't work correctly for some reason
    // Matrix3d C_bp_bm;
    // if (mag_a > 1.0e-8)
    //     C_bp_bm = I + sin(mag_a)/mag_a*A_ib_b +
    //          (1.0 - cos(mag_a))/(mag_a*mag_a)*A_ib_b*A_ib_b;
    // else
    //     C_bp_bm = I + A_ib_b;

    // Calculate the transformation matrix from the new attitude w.r.t. an
    // inertial frame (simplified)
    Matrix3d C_bp_bm = I + A_ib_b;

    // Update attitude (Equation 5.76) NOTE: Fixed per errata
    // C_b_n = C_b_n*C_bp_bm - (W_ie_n_old + W_en_n_old)*C_b_n*dt;

    // Update attitude (Equation 5.77) NOTE: Fixed per errata
    C_b_n = (I - (W_ie_n + 0.5*W_en_n_old + 0.5*W_en_n) * dt) * C_b_n * C_bp_bm;

    // Re-orthonormalize the resulting rotation matrix (Equations 5.78 to 5.80)
    C_b_n = orthonormalize(C_b_n);

}
