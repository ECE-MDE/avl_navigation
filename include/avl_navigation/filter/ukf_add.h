//==============================================================================
// Autonomous Vehicle Library
//
// Description: Basic Unscented Kalman Filter (UKF) implementation assuming
//              additive process and measurement noise. Uses the Eigen linear
//              algebra library.
//==============================================================================

#ifndef UKF_ADD_H
#define UKF_ADD_H

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class UkfAdd
{

public:

    //--------------------------------------------------------------------------
    // Name:        UkfAdd constructor
    //--------------------------------------------------------------------------
    UkfAdd();

    //--------------------------------------------------------------------------
    // Name:        UkfAdd destructor
    //--------------------------------------------------------------------------
    virtual ~UkfAdd();

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the UKF with initial state and its covariance.
    // Arguments:   - x0: Initial state vector (N x 1).
    //              - P0: Initial state covariance matrix (N x N).
    //--------------------------------------------------------------------------
    void init(const VectorXd &x0, const MatrixXd& P0);

    //--------------------------------------------------------------------------
    // Name:        predict
    // Description: Predicts the next state and state covariance using the
    //              nonlinear state transition function. The state transition
    //              function must accept a VectorXd (the current state vector)
    //              and return a VectorXd (the predicted state vector). Extra
    //              arguments can be passed by using std::bind and a
    //              placeholder.
    // Arguments:   - f: Nonlinear state transition function
    //                   (in: N x 1, out: N x 1).
    //              - Q: Process noise covariance matrix (N x N).
    //--------------------------------------------------------------------------
    void predict(std::function<VectorXd(VectorXd)> f, const MatrixXd& Q);

    //--------------------------------------------------------------------------
    // Name:        update
    // Description: Updates the state and state covariance from a measurement
    //              using the given nonlinear measurement function and its noise
    //              covariance matrix. The state measurement function must
    //              accept a VectorXd (the current state vector) and return a
    //              VectorXd (the measurement vector). Extra arguments can be
    //              passed by using std::bind and a placeholder.
    // Arguments:   - h: Nonlinear measurement function (in: N x 1, out: M x 1).
    //              - R: Measurement noise covariance matrix (M x M).
    //              - z: Measurement value (M x 1).
    //--------------------------------------------------------------------------
    void update(const std::function<VectorXd(VectorXd)> h, const MatrixXd& R,
        const VectorXd& z);

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the UKF state vector.
    // Returns:     UKF state vector.
    //--------------------------------------------------------------------------
    VectorXd get_state() const;

    //--------------------------------------------------------------------------
    // Name:        get_state_covariance
    // Description: Gets the UKF state covariance matrix.
    // Returns:     UKF state covariance matrix.
    //--------------------------------------------------------------------------
    MatrixXd get_state_covariance() const;

private:

    // State vector (L x 1) and covariance matrix (L x L)
    VectorXd x;
    MatrixXd P;

    // Sigma point distribution and scaling parameters
    double a = 1.0e-3;
    double B = 2.0;
    double k = 0.0;
    double lambda;

    // Sigma point mean and covariance weight vectors
    VectorXd Wm;
    VectorXd Wc;

private:

    //--------------------------------------------------------------------------
    // Name:        get_sigma_points
    // Description: Updates the standard 2L+1 sigma points and weight vectors
    //              used to represent the mean and covariance of the state
    //              vector.
    // Arguments:   - xs: State vector to generate sigma points for (L x 1).
    //              - Ps: State covarianve matrix used to disperse sigma
    //                    points (L x L).
    // Returns:     Matrix of sigma points (L x 2L+1).
    //--------------------------------------------------------------------------
    MatrixXd get_sigma_points(VectorXd xs, MatrixXd Ps);

};

#endif // UKF_ADD_H
