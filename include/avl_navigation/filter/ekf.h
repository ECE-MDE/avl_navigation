//==============================================================================
// Autonomous Vehicle Library
//
// Description: Basic Extended Kalman Filter (EKF) implementation using the
//              Eigen linear algebra library.
//==============================================================================

#ifndef EKF_H
#define EKF_H

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Measurement info struct
#include "meas_info.h"

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Ekf
{

public:

    // State vector
    VectorXd x;

    // State covariance matrix
    MatrixXd P;

    // Number of states
    int N;

public:

    //--------------------------------------------------------------------------
    // Name:        Ekf constructor
    //--------------------------------------------------------------------------
    Ekf();

    //--------------------------------------------------------------------------
    // Name:        Ekf destructor
    //--------------------------------------------------------------------------
    virtual ~Ekf();

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the EKF with initial state and covariance,
    //              state transition matrix, and process noise covariance.
    // Arguments:   - x0: initial state vector (N x 1)
    //              - P0: initial state covariance matrix (N x N)
    //--------------------------------------------------------------------------
    void init(VectorXd x0, MatrixXd P0);

    //--------------------------------------------------------------------------
    // Name:        predict
    // Description: Predicts the next state and state covariance using the
    //              state transition matrix.
    // Arguments:   - F: state transition matrix (N x N)
    //              - Q: process noise covariance matrix (N x N)
    //--------------------------------------------------------------------------
    void predict(MatrixXd F, MatrixXd Q);

    //--------------------------------------------------------------------------
    // Name:        update
    // Description: Updates the state and state covariance using the given
    //              measurement matrix and its noise covariance matrix.
    // Arguments:   - H: measurement matrix (M x N)
    //              - R: measurement noise covariance matrix (M x M)
    //              - z: measurement value (M x 1)
    //--------------------------------------------------------------------------
    MeasInfo update(const MatrixXd &H, const MatrixXd &R, const VectorXd &z,
        VectorXd threshold);

};

#endif // EKF_H
