//==============================================================================
// Autonomous Vehicle Library
//
// Description: Basic Unscented Kalman Filter (UKF) implementation using the
//              Eigen linear algebra library. This UKF implementation assumes
//              additive process and measurement noise.
//==============================================================================

#ifndef UKF_H
#define UKF_H

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// State type enum
#include "state_type.h"

// Measurement info struct
#include "meas_info.h"

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class Ukf
{

public:

    //--------------------------------------------------------------------------
    // Name:        Ekf constructor
    //--------------------------------------------------------------------------
    Ukf();

    //--------------------------------------------------------------------------
    // Name:        Ekf destructor
    //--------------------------------------------------------------------------
    virtual ~Ukf();

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the UKF with initial state and its covariance.
    // Arguments:   - x0: initial state vector (N x 1)
    //              - P0: initial state covariance matrix (N x N)
    //--------------------------------------------------------------------------
    void init(const VectorXd &x0, const MatrixXd &P0);

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the UKF with initial state and its covariance.
    // Arguments:   - x0: initial state vector (N x 1)
    //              - P0: initial state covariance matrix (N x N)
    //              - state_types: Vector of state types indicating the type of
    //                each state (standard or angle) (N)
    //--------------------------------------------------------------------------
    void init(const VectorXd &x0, const MatrixXd &P0,
        std::vector<StateType> state_types);

    //--------------------------------------------------------------------------
    // Name:        predict
    // Description: Predicts the next state and state covariance using the
    //              nonlinear state transition function. The state transition
    //              function must accept a VectorXd (the current state vector)
    //              and return a VectorXd (the predicted state vector). Extra
    //              arguments can be passed by using std::bind and a
    //              placeholder.
    // Arguments:   - f: nonlinear state transition function
    //                (in: N x 1, out: N x 1)
    //              - Q: process noise covariance matrix (N x N)
    //--------------------------------------------------------------------------
    void predict(std::function<VectorXd(VectorXd, VectorXd)> f,
        const MatrixXd &Pv);

    //--------------------------------------------------------------------------
    // Name:        update
    // Description: Updates the state and state covariance from a measurement
    //              using the given nonlinear measurement function and its noise
    //              covariance matrix. The state measurement function must
    //              accept a VectorXd (the current state vector) and return a
    //              VectorXd (the measurement vector). Extra arguments can be
    //              passed by using std::bind and a placeholder.
    // Arguments:   - h: nonlinear measurement function (in: N x 1, out: M x 1)
    //              - R: measurement noise covariance matrix (M x M)
    //              - z: measurement value (M x 1)
    // Returns:     Structure containing info about the measurement update step.
    //--------------------------------------------------------------------------
    MeasInfo update(const std::function<VectorXd(VectorXd, VectorXd)> h,
        const MatrixXd &Pn, const VectorXd &z);

    //--------------------------------------------------------------------------
    // Name:        update
    // Description: Updates the state and state covariance from a measurement
    //              using the given nonlinear measurement function and its noise
    //              covariance matrix. The state measurement function must
    //              accept a VectorXd (the current state vector) and return a
    //              VectorXd (the measurement vector). Extra arguments can be
    //              passed by using std::bind and a placeholder.
    // Arguments:   - h: nonlinear measurement function (in: N x 1, out: M x 1)
    //              - R: measurement noise covariance matrix (M x M)
    //              - z: measurement value (M x 1)
    // Returns:     Structure containing info about the measurement update step.
    //--------------------------------------------------------------------------
    MeasInfo update(const std::function<VectorXd(VectorXd, VectorXd)> h,
        const MatrixXd &Pn, const VectorXd &z, VectorXd threshold);

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the UKF state vector.
    // Returns:     UKF state vector.
    //--------------------------------------------------------------------------
    VectorXd get_state() const;

    //--------------------------------------------------------------------------
    // Name:        get_cov
    // Description: Gets the UKF state covariance matrix.
    // Returns:     UKF state covariance matrix.
    //--------------------------------------------------------------------------
    MatrixXd get_cov() const;

private:

    // State vector (L x 1) and covariance matrix (L x L)
    VectorXd x;
    MatrixXd P;

    // Vector of state types. Empty if all states are standard states
    std::vector<StateType> state_types;

    // Sigma point distribution and scaling parameters
    double a = 1.0e-3;
    double B = 2.0;
    double k = 0.0;

private:

    //--------------------------------------------------------------------------
    // Name:        update_sigma_points
    // Description: Updates the standard 2N+1 sigma points and weight vectors
    //              used to represent the mean and covariance of the state
    //              vector.
    //--------------------------------------------------------------------------
    MatrixXd get_sigma_points(VectorXd xs, MatrixXd Ps, VectorXd& Wm,
        VectorXd& Wc);

};

#endif // UKF_H
