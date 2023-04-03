//==============================================================================
// Autonomous Vehicle Library
//
// Description: Error state SINS helper functions. State vector is the
//              following:
//              x = [psi_nb_n; v_eb_n; p_b; b_g; b_a]
//==============================================================================

#ifndef ERROR_STATE_H
#define ERROR_STATE_H

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//							FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Q_ins
// Description: Calculates the error state INS system noise covariance from IMU
//              measurement noise properties. (Equation 14.82, Equation 14.83,
//              and Equation 14.84)
// Arguments:   - cov_gyro: Gyroscope noise covariance in (rad/s)^2.
//              - cov_accel: Accelerometer noise covariance in (m/s^2)^2.
//              - cov_bg: Gyroscope bias noise covariance in (rad/s)^2.
//              - cov_ba: Accelerometer bias noise covariance in (m/s^2)^2.
//              - dt: Propagation interval in seconds.
// Returns:     Error state INS system noise covariance matrix.
//------------------------------------------------------------------------------
MatrixXd Q_ins(double cov_gyro, double cov_accel, double cov_bg, double cov_ba,
               double dt);

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
   Vector3d w_ib_b, Vector3d f_ib_b, const double &dt);

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
    Vector3d w_ib_b, Vector3d f_ib_b, const double &dt);

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
    Vector3d& dp_b, Vector3d& db_g, Vector3d& db_a);

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
    Vector3d& dr_b, Vector3d& db_g, Vector3d& db_a);

//------------------------------------------------------------------------------
// Name:        H_vel
// Description: Calculates the linearized measurement matrix for a body frame
//              velocity measurement.
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_vel(Matrix3d& C_b_n, Vector3d& v_eb_n);

//------------------------------------------------------------------------------
// Name:        H_depth
// Description: Calculates the linearized measurement matrix for a depth
//              measurement.
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_depth();

//------------------------------------------------------------------------------
// Name:        H_range
// Description: Calculates the linearized measurement matrix for a range
//              measurement.
// Arguments:   - p_src: Position of the range measurement source.
//              - p_b: Position of the body frame.
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_range(Vector3d p_b, Vector3d p_src);

//------------------------------------------------------------------------------
// Name:        H_gps
// Description: Calculates the linearized measurement matrix for a GPS
//              measurement of position and NED velocity. (Equation I.87)
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_gps();

//------------------------------------------------------------------------------
// Name:        H_pos
// Description: Calculates the linearized measurement matrix for a  measurement
//              of position. (Equation I.87)
// Returns:     Linearized measurement matrix.
//------------------------------------------------------------------------------
MatrixXd H_pos();

#endif // ERROR_STATE_H
