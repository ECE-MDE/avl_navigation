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

// // Util functions
// #include <avl_core/util/math.h>
#include <avl_core/util/matrix.h>
using namespace avl;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                            FUNCTION DEFINITIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Q_ins
// Description: Calculates the INS system noise covariance from the IMU
//              measurement noise properties. (Equation 14.82, Equation 14.83,
//              and Equation 14.84)
// Arguments:   - cov_gyro: Gyroscope noise covariance in (rad/s)^2.
//              - cov_accel: Accelerometer noise covariance in (m/s^2)^2.
//              - cov_bg: Gyroscope bias noise covariance in (rad/s)^2.
//              - cov_ba: Accelerometer bias noise covariance in (m/s^2)^2.
//              - dt: Propagation interval in seconds.
// Returns:     INS system noise covariance matrix.
//------------------------------------------------------------------------------
MatrixXd Q_ins(double cov_gyro, double cov_accel, double cov_bg, double cov_ba,
               double dt)
{

    // Equation 14.83
    double S_rg = cov_gyro * dt;
    double S_ra = cov_accel * dt;

    // Equation 14.84
    double S_bgd = cov_bg / dt;
    double S_bad = cov_ba / dt;

    // Assemble the INS system noize covariance matrix (Equation 14.82)
    // Gyro and accel biad order are swapped.
    MatrixXd Q = MatrixXd::Zero(15,15);
    Matrix3d I = Matrix3d::Identity();
    Q.block(0,0,3,3)   = S_rg  * I;
    Q.block(3,3,3,3)   = S_ra  * I;
    Q.block(9,9,3,3)   = S_bgd * I;
    Q.block(12,12,3,3) = S_bad * I;
    return Q;

}

//------------------------------------------------------------------------------
// Name:        phi_err_n
// Description: Calculates the discrete error state transition matrix in the NED
//              frame. (Equations 14.64 through 14.72)
// Arguments:   - C_b_n: Body-to-NED frame transformation matrix.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - p_b: Curvilinear position of the body frame in radians
//                and meters.
//              - w_ib_b: angular velocity of the body frame w.r.t. the
//                inertial frame, expressed in the body frame in rad/s
//              - f_ib_b: Specific force of the body frame w.r.t. the inertial
//                frame, expressed in the body frame in m/s^2
//              - dt: time step in seconds
//------------------------------------------------------------------------------
MatrixXd phi_err_n(Matrix3d &C_b_n, Vector3d &v_eb_n, Vector3d &p_b,
    Vector3d w_ib_b, Vector3d f_ib_b, const double &dt)
{

    // Velocity components
    double vN = v_eb_n(0);
    double vE = v_eb_n(1);
    double vD = v_eb_n(2);

    // Curvilinear position components
    double L_b = p_b(0);
    double h_b = p_b(2);

    // Determine the radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Calculate surface gravity using the Somigliana model
    double g_0 = somigliana_gravity(L_b);

    // Determine the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the NED frame
    Vector3d omega_ie_n = earth_rotation_rate(p_b);

    // Determine the angular rate of the NED frame w.r.t the ECEF frame,
    // expressed in the NED frame. This is called the transort rate
    Vector3d omega_en_n = transport_rate(p_b, v_eb_n);

    // Calculate the angular rate of the NED frame w.r.t the inertial frame
    // expressed in the NED frame
    Vector3d omega_in_n = omega_ie_n + omega_en_n;

    // Variables for notational convenience
    double REh = R_E + h_b;
    double RNh = R_N + h_b;

    // Construct the block matrices

    // (Equation 14.64)
    Matrix3d F11 = -skew(omega_in_n);

    // (Equation 14.65)
    Matrix3d F12 = Matrix3d::Zero();
    F12(0,1) = -1 / REh;
    F12(1,0) =  1 / RNh;
    F12(2,1) = tan(L_b) / REh;

    // (Equation 14.66)
    Matrix3d F13 = Matrix3d::Zero();
    F13(0,0) = omega_ie * sin(L_b);
    F13(0,2) = vE / (REh*REh);
    F13(1,2) = -vN / (RNh*RNh);
    F13(2,0) = omega_ie*cos(L_b) + vE / (RNh*cos(L_b)*cos(L_b));
    F13(2,2) = -vE * tan(L_b) / (REh*REh);

    // (Equation 14.67)
    Matrix3d F21 = -skew(C_b_n * f_ib_b);

    // (Equation 14.68)
    Matrix3d F22 = Matrix3d::Zero();
    F22(0,0) = vD / RNh;
    F22(0,1) = -2*vE*tan(L_b) / REh - 2*omega_ie*sin(L_b);
    F22(0,2) = vN / RNh;
    F22(1,0) = vE*tan(L_b)/REh + 2*omega_ie*sin(L_b);
    F22(1,1) = (vN*tan(L_b) + vD) / REh;
    F22(1,2) = vE / REh + 2*omega_ie*cos(L_b);
    F22(2,0) = -2*vN / RNh;
    F22(2,1) = -2*vE / REh - 2*omega_ie*cos(L_b);

    // (Equation 14.69)
    Matrix3d F23 = Matrix3d::Zero();
    F23(0,0) = -vE*vE*avl::sec(L_b)*avl::sec(L_b) / REh - 2*vE*omega_ie*cos(L_b);
    F23(0,2) = vE*vE*tan(L_b) / (REh*REh) - vN*vD / (RNh*RNh);
    F23(1,0) = vN*vE*avl::sec(L_b)*avl::sec(L_b) / REh + 2*vN*omega_ie*cos(L_b) - 2*vD*omega_ie*sin(L_b);
    F23(1,2) = -(vN*vE*tan(L_b) + vE*vD) / (REh*REh);
    F23(2,0) = 2*vE*omega_ie*sin(L_b);
    F23(2,2) = vE*vE / (REh*REh) + vN*vN / (RNh*RNh) - 2*g_0 / r_eS_e;

    // (Equation 14.70)
    Matrix3d F32 = Matrix3d::Zero();
    F32(0,0) = 1.0 / RNh;
    F32(1,1) = 1.0 / (RNh*cos(L_b));
    F32(2,2) = -1.0;

    // (Equation 14.71)
    Matrix3d F33 = Matrix3d::Zero();
    F33(0,2) = -vN / (RNh*RNh);
    F33(1,0) = vE*sin(L_b) / (REh*cos(L_b)*cos(L_b));
    F33(1,2) = -vE / (REh*REh*cos(L_b));

    // Assemble the block elements into the discrete error state transition
    // matrix (Equation 14.72)
    MatrixXd Phi_ins_n = MatrixXd::Identity(15,15);

    Phi_ins_n.block(0,0,3,3)  += F11*dt;
    Phi_ins_n.block(0,3,3,3)  += F12*dt;
    Phi_ins_n.block(0,6,3,3)  += F13*dt;
    Phi_ins_n.block(0,9,3,3)  += C_b_n*dt; // Order of bias states swapped

    Phi_ins_n.block(3,0,3,3)  += F21*dt;
    Phi_ins_n.block(3,3,3,3)  += F22*dt;
    Phi_ins_n.block(3,6,3,3)  += F23*dt;
    Phi_ins_n.block(3,12,3,3) += C_b_n*dt; // Order of bias states swapped

    Phi_ins_n.block(6,3,3,3)  += F32*dt;
    Phi_ins_n.block(6,6,3,3)  += F33*dt;

    return Phi_ins_n;

}

//------------------------------------------------------------------------------
// Name:        phi_err_nC
// Description: Calculates the discrete error state transition matrix in the NED
//              cartesian frame as presented in Appendix I.2.2. (Equations 14.64
//              through 14.72 and Equations I.14 through I.17)
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
MatrixXd phi_err_nC(Matrix3d &C_b_n, Vector3d &v_eb_n, Vector3d &p_b,
                   Vector3d w_ib_b, Vector3d f_ib_b, const double &dt)
{

    // Velocity components
    double vN = v_eb_n(0);
    double vE = v_eb_n(1);
    double vD = v_eb_n(2);

    // Curvilinear position components
    double L_b = p_b(0);
    double h_b = p_b(2);

    // Determine the radii of curvature
    double R_N, R_E, r_eS_e;
    radii_of_curvature(p_b, R_N, R_E, r_eS_e);

    // Calculate surface gravity using the Somigliana model
    double g_0 = somigliana_gravity(L_b);

    // Determine the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the NED frame
    Vector3d omega_ie_n = earth_rotation_rate(p_b);

    // Determine the angular rate of the NED frame w.r.t the ECEF frame,
    // expressed in the NED frame. This is called the transort rate
    Vector3d omega_en_n = transport_rate(p_b, v_eb_n);

    // Calculate the angular rate of the NED frame w.r.t the inertial frame
    // expressed in the NED frame
    Vector3d omega_in_n = omega_ie_n + omega_en_n;

    // Variables for notational convenience
    double REh = R_E + h_b;
    double RNh = R_N + h_b;

    // Construct the block matrices

    // (Equation 14.64)
    Matrix3d F11 = -skew(omega_in_n);

    // (Equation 14.65)
    Matrix3d F12 = Matrix3d::Zero();
    F12(0,1) = -1 / REh;
    F12(1,0) =  1 / RNh;
    F12(2,1) = tan(L_b) / REh;

    // (Equation I.14)
    Matrix3d F13 = Matrix3d::Zero();
    F13(0,0) = omega_ie * sin(L_b) / RNh;
    F13(0,2) = -vE / (REh*REh);
    F13(1,2) =  vN / (RNh*RNh);
    F13(2,0) = omega_ie*cos(L_b) / RNh + vE / (RNh*REh*cos(L_b)*cos(L_b));
    F13(2,2) = vE * tan(L_b) / (REh*REh);

    // (Equation 14.67)
    Matrix3d F21 = -skew(C_b_n * f_ib_b);

    // (Equation 14.68)
    Matrix3d F22 = Matrix3d::Zero();
    F22(0,0) = vD / RNh;
    F22(0,1) = -2*vE*tan(L_b) / REh - 2*omega_ie*sin(L_b);
    F22(0,2) = vN / RNh;
    F22(1,0) = vE*tan(L_b)/REh + 2*omega_ie*sin(L_b);
    F22(1,1) = (vN*tan(L_b) + vD) / REh;
    F22(1,2) = vE / REh + 2*omega_ie*cos(L_b);
    F22(2,0) = -2*vN / RNh;
    F22(2,1) = -2*vE / REh - 2*omega_ie*cos(L_b);

    // (Equation I.15)
    Matrix3d F23 = Matrix3d::Zero();
    F23(0,0) = -vE*vE*avl::sec(L_b)*avl::sec(L_b) / (RNh*REh) - 2*vE*omega_ie*cos(L_b) / RNh;
    F23(0,2) = -vE*vE*tan(L_b) / (REh*REh) + vN*vD / (RNh*RNh);
    F23(1,0) = vN*vE*avl::sec(L_b)*avl::sec(L_b) / (RNh*REh) + 2*(omega_ie*vN*cos(L_b) - vD*sin(L_b)) / RNh;
    F23(1,2) = (vN*vE*tan(L_b) + vE*vD) / (REh*REh);
    F23(2,0) = 2*vE*omega_ie*sin(L_b) / RNh;
    F23(2,2) = -vE*vE / (REh*REh) - vN*vN / (RNh*RNh) + 2*g_0 / r_eS_e;

    // (Equation I.16)
    Matrix3d F33 = Matrix3d::Zero();
    F33(0,2) = vN / RNh;
    F33(1,0) = vE*tan(L_b) / RNh;
    F33(1,2) = vE / REh;

    // Assemble the block elements into the discrete error state transition
    // matrix (Equation I.17)
    MatrixXd Phi_ins_nC = MatrixXd::Identity(15,15);

    Phi_ins_nC.block(0,0,3,3)  += F11*dt;
    Phi_ins_nC.block(0,3,3,3)  += F12*dt;
    Phi_ins_nC.block(0,6,3,3)  += F13*dt;
    Phi_ins_nC.block(0,9,3,3)  += C_b_n*dt; // Order of bias states swapped

    Phi_ins_nC.block(3,0,3,3)  += F21*dt;
    Phi_ins_nC.block(3,3,3,3)  += F22*dt;
    Phi_ins_nC.block(3,6,3,3)  += F23*dt;
    Phi_ins_nC.block(3,12,3,3) += C_b_n*dt; // Order of bias states swapped

    Phi_ins_nC.block(6,3,3,3)  += Matrix3d::Identity()*dt;
    Phi_ins_nC.block(6,6,3,3)  += F33*dt;

    return Phi_ins_nC;

}

//------------------------------------------------------------------------------
// Name:        correct_ins_n
// Description: Performs closed loop NED frame INS correction by subtracting the
//              estimated error from the INS estimate and resetting the error
//              states to zero. (Equations 14.7 through 14.9)
// Arguments:   - C_b_n: Body-to-NED frame transformation matrix.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - p_b: Curvilinear position of the body frame in radians
//                and meters.
//              - b_g: Gyroscope bias in rad/s.
//              - b_a: Accelerometer bias in m/s^2.
//              - dpsi_nb_n: Attitude error vector in radians.
//              - dv_eb_n: Velocity error vector in m/s.
//              - dp_b: Position error vector in radians and meters.
//              - db_g: Gyroscope bias error vector in rad/s.
//              - db_a: Accelerometer bias error vector in m/s^2.
//------------------------------------------------------------------------------
void correct_ins_n(Matrix3d& C_b_n, Vector3d& v_eb_n, Vector3d& p_b,
    Vector3d& b_g, Vector3d& b_a, Vector3d& dpsi_nb_n, Vector3d& dv_eb_n,
    Vector3d& dp_b, Vector3d& db_g, Vector3d& db_a)
{

    // 3x3 Identity matrix
    Matrix3d I = Matrix3d::Identity();

    // Perform the closed loop correction of INS states
    C_b_n = (I - skew(dpsi_nb_n)) * C_b_n; // (Equation 14.7)
    v_eb_n = v_eb_n - dv_eb_n;             // (Equation 14.8)
    p_b = p_b - dp_b;                      // (Equation 14.10)
    b_g = b_g + db_g;
    b_a = b_a + db_a;

    // Set the error states to zero
    dpsi_nb_n.setZero();
    dv_eb_n.setZero();
    dp_b.setZero();
    db_g.setZero();
    db_a.setZero();

}

//------------------------------------------------------------------------------
// Name:        correct_ins_nC
// Description: Performs closed loop NED cartesian frame INS correction by
//              subtracting the estimated error from the INS estimate and
//              resetting the error states to zero.
//              (Equations 14.7 through 14.9)
// Arguments:   - C_b_n: Body-to-NED frame transformation matrix.
//              - v_eb_n: Velocity of the body frame w.r.t the ECEF frame
//                expressed in the NED frame in m/s.
//              - p_b: Curvilinear position of the body frame in radians
//                and meters.
//              - b_g: Gyroscope bias in rad/s.
//              - b_a: Accelerometer bias in m/s^2.
//              - dpsi_nb_n: Attitude error vector in radians.
//              - dv_eb_n: Velocity error vector in m/s.
//              - dr_b: Position error vector in meters.
//              - db_g: Gyroscope bias error vector in rad/s.
//              - db_a: Accelerometer bias error vector in m/s^2.
//------------------------------------------------------------------------------
void correct_ins_nC(Matrix3d& C_b_n, Vector3d& v_eb_n, Vector3d& p_b,
    Vector3d& b_g, Vector3d& b_a, Vector3d& dpsi_nb_n, Vector3d& dv_eb_n,
    Vector3d& dr_b, Vector3d& db_g, Vector3d& db_a)
{

    // 3x3 Identity matrix
    Matrix3d I = Matrix3d::Identity();

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(p_b);

    // Perform the closed loop correction of INS states
    C_b_n = (I - skew(dpsi_nb_n)) * C_b_n; // (Equation 14.7)
    v_eb_n = v_eb_n - dv_eb_n;             // (Equation 14.8)
    p_b = p_b - T_rn_p * dr_b;             // (Equation I.11)
    b_g = b_g + db_g;
    b_a = b_a + db_a;

    // Set the error states to zero
    dpsi_nb_n.setZero();
    dv_eb_n.setZero();
    dr_b.setZero();
    db_g.setZero();
    db_a.setZero();

}

//------------------------------------------------------------------------------
// Name:        H_vel
// Description: Calculates the linearized measurement matrix for a body frame
//              velocity measurement.
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_vel(Matrix3d& C_b_n, Vector3d& v_eb_n)
{
    MatrixXd H = MatrixXd::Zero(3, 15);
    H.block(0,3,3,3) = -Matrix3d::Identity();
    return H;
}

//------------------------------------------------------------------------------
// Name:        H_depth
// Description: Calculates the linearized measurement matrix for a depth
//              measurement.
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_depth()
{
    MatrixXd H = MatrixXd::Zero(1, 15);
    H(0,8) = 1;
    return H;
}

//------------------------------------------------------------------------------
// Name:        H_range
// Description: Calculates the linearized measurement matrix for a range
//              measurement.
// Arguments:   - p_src: Position of the range measurement source.
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_range(Vector3d p_b, Vector3d p_src)
{
    MatrixXd H = MatrixXd::Zero(1, 15);
    Matrix3d T_p_rn = curvilinear_to_cartesian(p_b);
    Vector3d u = p_src - p_b;
    Vector3d h = T_p_rn * u;
    // h /= h.norm();
    H(0,6) = h(0);
    H(0,7) = h(1);
    H(0,8) = h(2);
    return H;
}

//------------------------------------------------------------------------------
// Name:        H_gps
// Description: Calculates the linearized measurement matrix for a GPS
//              measurement of position and NED velocity. (Equation I.87)
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_gps()
{
    MatrixXd H = MatrixXd::Zero(5, 15);
    H.block(0,6,3,3) = -Matrix3d::Identity();
    H.block(3,3,2,2) = -Matrix2d::Identity();
    return H;
}

//------------------------------------------------------------------------------
// Name:        H_pos
// Description: Calculates the linearized measurement matrix for a  measurement
//              of position. (Equation I.87)
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_pos()
{
    MatrixXd H = MatrixXd::Zero(3, 15);
    H.block(0,6,3,3) = -Matrix3d::Identity();
    return H;
}
