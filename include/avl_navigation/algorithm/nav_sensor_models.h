//==============================================================================
// Autonomous Vehicle Library
//
// Description: Error state SINS helper functions. State vector is the
//              following:
//              x = [psi_nb_n; v_eb_n; p_b; b_g; b_a]
//==============================================================================

#ifndef SENSOR_MODELS_H
#define SENSOR_MODELS_H

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>

//==============================================================================
//							STRUCT DEFINITION
//==============================================================================

struct MeasInput
{
    Matrix3d C_b_n;
    Vector3d v_eb_n;
    Vector3d p_b;
    VectorXd eta;
    Vector3d l_bS_b = Vector3d::Zero();

    void from_vector(VectorXd x)
    {
        Vector3d theta_n_b = x.segment(0, 3);
        Matrix3d C_n_b = avl::euler_to_matrix(theta_n_b);
        C_b_n = C_n_b.transpose();
        v_eb_n = x.segment(3, 3);
        p_b =    x.segment(6, 3);
    }

};

//==============================================================================
//							FUNCTION DECLARATIONS
//==============================================================================

//------------------------------------------------------------------------------
// Name:        h_body_vel
// Description: Nonlinear measurement equation for a measurement of the velocity
//              of the body frame relative to the earth frame, expressed in the
//              body frame.
// Arguments:   - input: Navigation state inputs.
//              - w_ib_b: Angular velocity of the body frame relative to the
//                inertial frame, expressed in the body frame as measured by an
//                IMU in rad/s.
// Returns :    Body frame velocity measurement corresponding to nav input. (3x1)
//------------------------------------------------------------------------------
VectorXd h_body_vel(MeasInput input, Vector3d w_ib_b)
{

    // Calculate the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the navigation frame
    Vector3d w_ie_n = earth_rotation_rate(input.p_b);

    // Calculate the angular rate of the sensor frame relative to the earth
    // frame, expresed in the sensor frame.
    Vector3d w_eS_S = w_ib_b - input.C_b_n.transpose()*w_ie_n;

    // Construct the measurement vector with the linear velocity of the sensor
    // frame relative to the earth frame
    return input.C_b_n.transpose()*input.v_eb_n - w_eS_S.cross(input.l_bS_b) + input.eta;

}

//------------------------------------------------------------------------------
// Name:        h_ned_vel
// Description: Nonlinear measurement equation for a measurement of the velocity
//              of the body frame relative to the earth frame, expressed in the
//              NED frame.
// Arguments:   - input: Navigation state inputs.
//              - w_ib_b: Angular velocity of the body frame relative to the
//                inertial frame, expressed in the body frame as measured by an
//                IMU in rad/s.
// Returns :    NED frame velocity measurement corresponding to nav input. (3x1)
//------------------------------------------------------------------------------
VectorXd h_ned_vel(MeasInput input, Vector3d w_ib_b)
{

    // Calculate the angular rate of the ECEF frame w.r.t the ECI frame,
    // expressed in the navigation frame
    Vector3d w_ie_n = earth_rotation_rate(input.p_b);

    // Calculate the angular rate of the sensor frame relative to the earth
    // frame, expresed in the sensor frame.
    Vector3d w_eS_S = w_ib_b - input.C_b_n.transpose()*w_ie_n;

    // Construct the measurement vector with the linear velocity of the sensor
    // frame relative to the earth frame
    return input.v_eb_n - input.C_b_n*w_eS_S.cross(input.l_bS_b) + input.eta;

}

//------------------------------------------------------------------------------
// Name:        h_depth
// Description: Nonlinear measurement equation for a water depth measurement
//              relative to the water's surface with lever arm correction.
// Arguments:   - input: Navigation state inputs.
//              - alt_surface: Altitude of the water's surface in meters.
// Returns :    Depth measurement generated from state vector. (1x1)
//------------------------------------------------------------------------------
VectorXd h_depth(MeasInput input, double alt_surface)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(input.p_b);

    // Calculate the depth sensor frame geodetic position from the body frame
    // position and the lever arm
    Vector3d p_S = input.p_b + T_rn_p*input.C_b_n*input.l_bS_b;

    // Calculate the depth from the surface altitude and the sensor frame
    // altitude
    VectorXd depth(1);
    depth << alt_surface - p_S(2);
    return depth + input.eta;

}

//------------------------------------------------------------------------------
// Name:        h_range
// Description: Nonlinear measurement equation for an acoustic range measurement
//              which measures range between a known beacon position and the
//              vehicle with a lever arm correction.
// Arguments:   - input: Navigation state inputs.
//              - p_source: Lat, lon, and alt of position that range is
//                measured to in radians and meters.
// Returns :    Range measurement generated from state vector. (1x1)
//------------------------------------------------------------------------------
VectorXd h_range(MeasInput input, Vector3d p_source)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(input.p_b);

    // Calculate the range sensor frame geodetic position from the body frame
    // position and the lever arm
    Vector3d p_S = input.p_b + T_rn_p*input.C_b_n*input.l_bS_b;

    // Calculate the range between the source position and the sensor frame
    VectorXd range(1);
    range << linear_dist(p_S, p_source);
    return range + input.eta;

}

//------------------------------------------------------------------------------
// Name:        h_pos
// Description: Nonlinear measurement equation for a GPS that provides geodetic
//              position measurements in radians and meters with lever arm
//              correction.
// Arguments:   - input: Navigation state inputs.
// Returns :    Position measurement vector generated from state vector. (3x1)
//------------------------------------------------------------------------------
VectorXd h_pos(MeasInput input)
{

    // Calculate the cartesian to curvilinear transformation matrix
    Matrix3d T_rn_p = cartesian_to_curvilinear(input.p_b);

    // Calculate the GPS sensor frame geodetic position from the body frame
    // position and the lever arm
    return input.p_b + T_rn_p*input.C_b_n*input.l_bS_b + input.eta;

}

//------------------------------------------------------------------------------
// Name:        h_gps
// Description: Nonlinear measurement equation for a GPS that provides geodetic
//              position measurements in radians and meters with lever arm
//              correction.
// Arguments:   - input: Navigation state inputs.
// Returns :    Position measurement vector generated from state vector. (3x1)
//------------------------------------------------------------------------------
VectorXd h_gps(MeasInput input, Vector3d w_ib_b)
{
    VectorXd y(5);
    y.segment(0,3) = input.p_b;
    y.segment(3,2) = input.v_eb_n.segment(0,2);
    return y + input.eta;
}

//------------------------------------------------------------------------------
// Name:        h_body_vel
// Description: Nonlinear measurement equation for a measurement of the velocity
//              of the body frame relative to the earth frame, expressed in the
//              body frame.
// Arguments:   - input: Navigation state inputs.
//              - w_ib_b: Angular velocity of the body frame relative to the
//                inertial frame, expressed in the body frame as measured by an
//                IMU in rad/s.
// Returns :    Body frame velocity measurement corresponding to nav input. (3x1)
//------------------------------------------------------------------------------
VectorXd h_body_vel_vec(VectorXd x, VectorXd eta, Vector3d l_bS_b, Vector3d w_ib_b)
{
    MeasInput input;
    input.from_vector(x);
    input.eta = eta;
    input.l_bS_b = l_bS_b;
    return h_body_vel(input, w_ib_b);
}

//------------------------------------------------------------------------------
// Name:        h_ned_vel_vec
// Description: Nonlinear measurement equation for a measurement of the velocity
//              of the body frame relative to the earth frame, expressed in the
//              NED frame.
// Arguments:   - input: Navigation state inputs.
//              - w_ib_b: Angular velocity of the body frame relative to the
//                inertial frame, expressed in the body frame as measured by an
//                IMU in rad/s.
// Returns :    NED frame velocity measurement corresponding to nav input. (3x1)
//------------------------------------------------------------------------------
VectorXd h_ned_vel_vec(VectorXd x, VectorXd eta, Vector3d l_bS_b, Vector3d w_ib_b)
{
    MeasInput input;
    input.from_vector(x);
    input.eta = eta;
    input.l_bS_b = l_bS_b;
    return h_ned_vel(input, w_ib_b);
}

//------------------------------------------------------------------------------
// Name:        h_depth_vec
// Description: Nonlinear measurement equation for a water depth measurement
//              relative to the water's surface with lever arm correction.
// Arguments:   - input: Navigation state inputs.
//              - alt_surface: Altitude of the water's surface in meters.
// Returns :    Depth measurement generated from state vector. (1x1)
//------------------------------------------------------------------------------
VectorXd h_depth_vec(VectorXd x, VectorXd eta, Vector3d l_bS_b, double alt_surface)
{
    MeasInput input;
    input.from_vector(x);
    input.eta = eta;
    input.l_bS_b = l_bS_b;
    return h_depth(input, alt_surface);
}

//------------------------------------------------------------------------------
// Name:        h_range_vec
// Description: Nonlinear measurement equation for an acoustic range measurement
//              which measures range between a known beacon position and the
//              vehicle with a lever arm correction.
// Arguments:   - input: Navigation state inputs.
//              - p_source: Lat, lon, and alt of position that range is
//                measured to in radians and meters.
// Returns :    Range measurement generated from state vector. (1x1)
//------------------------------------------------------------------------------
VectorXd h_range_vec(VectorXd x, VectorXd eta, Vector3d l_bS_b, Vector3d p_source)
{
    MeasInput input;
    input.from_vector(x);
    input.eta = eta;
    input.l_bS_b = l_bS_b;
    return h_range(input, p_source);
}

//------------------------------------------------------------------------------
// Name:        h_pos_vec
// Description: Nonlinear measurement equation for a GPS that provides geodetic
//              position measurements in radians and meters with lever arm
//              correction.
// Arguments:   - input: Navigation state inputs.
// Returns :    Position measurement vector generated from state vector. (3x1)
//------------------------------------------------------------------------------
VectorXd h_pos_vec(VectorXd x, VectorXd eta, Vector3d l_bS_b)
{
    MeasInput input;
    input.from_vector(x);
    input.eta = eta;
    input.l_bS_b = l_bS_b;
    return h_pos(input);
}

//------------------------------------------------------------------------------
// Name:        h_gps_vec
// Description: Nonlinear measurement equation for a GPS that provides geodetic
//              position measurements in radians and meters with lever arm
//              correction.
// Arguments:   - input: Navigation state inputs.
// Returns :    Position measurement vector generated from state vector. (3x1)
//------------------------------------------------------------------------------
VectorXd h_gps_vec(VectorXd x, VectorXd eta, Vector3d l_bS_b, Vector3d w_ib_b)
{
    MeasInput input;
    input.from_vector(x);
    input.eta = eta;
    input.l_bS_b = l_bS_b;
    return h_gps(input, w_ib_b);
}

#endif // SENSOR_MODELS_H
