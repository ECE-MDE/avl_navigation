//==============================================================================
// Autonomous Vehicle Library
//
// Description: Pure virtual base class for navigation filters.
//==============================================================================

#ifndef NAV_FILTER_H
#define NAV_FILTER_H

// Terrain map class
#include "terrain_map.h"

// Measurement info struct
#include "../filter/meas_info.h"

// Eigen includes
#include <Eigen/Dense>
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class NavFilter
{

public:

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the filter with an initial state, covariance,
    //              and process noise covariance.
    // Arguments:   - x0: Initial state vector.
    //              - P0: Initial state covariance matrix.
    //              - P0: Process noise covariance matrix.
    //--------------------------------------------------------------------------
    virtual void init(VectorXd x0, MatrixXd P0, MatrixXd Q) = 0;

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
    virtual void iterate(Vector3d w_ib_b, Vector3d f_ib_b, double dt) = 0;

    //--------------------------------------------------------------------------
    // Name:        process_attitude
    // Description: Processes an attitude measurement.
    // Arguments:   - theta_n_b_meas: Attitude measurement. (3x1)[rad]
    //              - R: Measurement noise covariance matrix. (3x3)[rad^2]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    virtual MeasInfo process_attitude(Vector3d theta_n_b_meas,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

    //--------------------------------------------------------------------------
    // Name:        process_mag_flux
    // Description: Processes a magnetometer measurement.
    // Arguments:   - m_b_meas: Magnetic flux measurement. (3x1)[mF]
    //              - R: Measurement noise covariance matrix. (3x3)[mF^2]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    virtual MeasInfo process_mag_flux(Vector3d m_b_meas,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

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
    virtual MeasInfo process_body_velocity(Vector3d v_eb_b_meas, Vector3d w_ib_b,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

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
    virtual MeasInfo process_depth(VectorXd depth_meas, double alt_surface,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

    //--------------------------------------------------------------------------
    // Name:        process_sonar
    // Description: Processes a sonar measurement.
    // Arguments:   - ranges: Beam range measurements. (Nx1)[m]
    //              - p_beams_b: Beam positions in the body frame. (3xN)[m]
    //              - map: Pointer to terrain map to query.
    //              - R: Measurement noise covariance matrix. (NxN)[m^2]
    //              - threshold: Measurement rejection threshold for each
    //                measurement element. 0 disables measurement rejection.
    //              - l_bS_b: Optional lever arm from the body frame to the
    //                sensor frame, expressed in the body frame. (3x1)[m]
    // Returns:     Measurement info struct.
    //--------------------------------------------------------------------------
    virtual MeasInfo process_sonar(VectorXd ranges, MatrixXd p_beams_b,
        TerrainMap* map, MatrixXd R, VectorXd threshold,
        Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

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
    virtual MeasInfo process_range(VectorXd range_meas, Vector3d p_source,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

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
    virtual MeasInfo process_position(Vector3d p_b_meas,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

    //--------------------------------------------------------------------------
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
    //--------------------------------------------------------------------------
    virtual MeasInfo process_gps(VectorXd gps_meas, Vector3d w_ib_b,
        MatrixXd R, VectorXd threshold, Vector3d l_bS_b=Vector3d::Zero())
    {
        return MeasInfo();
    }

    //--------------------------------------------------------------------------
    // Name:        valid
    // Description: Indicates whether the nav state estimate is valid.
    // Returns:     True if nav state estimates are valid, false otherwise.
    //--------------------------------------------------------------------------
    virtual bool valid() = 0;

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the most recent state vector.
    // Returns:     Most recent state vector. (15 x 1)
    //--------------------------------------------------------------------------
    virtual VectorXd get_state() = 0;

    //--------------------------------------------------------------------------
    // Name:        get_cov
    // Description: Gets the diagonal terms of the most recent state covariance
    //              matrix.
    // Returns:     Diagonal terms of the most recent state covariance
    //              matrix. (15x1)
    //--------------------------------------------------------------------------
    virtual VectorXd get_cov() = 0;

};

#endif // NAV_FILTER_H
