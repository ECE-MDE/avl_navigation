//==============================================================================
// Autonomous Vehicle Library
//
// Description: Basic Unscented Kalman Filter (UKF) implementation assuming
//              additive process and measurement noise. Uses the Eigen linear
//              algebra library.
//==============================================================================

#include <avl_navigation/filter/ukf_add.h>

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        UkfAdd constructor
//------------------------------------------------------------------------------
UkfAdd::UkfAdd()
{

}

//------------------------------------------------------------------------------
// Name:        UkfAdd destructor
//------------------------------------------------------------------------------
UkfAdd::~UkfAdd()
{

}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the UKF with initial state and its covariance.
// Arguments:   - x0: Initial state vector (N x 1).
//              - P0: Initial state covariance matrix (N x N).
//------------------------------------------------------------------------------
void UkfAdd::init(const VectorXd& x0, const MatrixXd& P0)
{

    // Number of elements in the state vector
    int L = x0.size();


    // Ensure that P0 has the same number of rows and columns as the number of
    // elements in x0
    if (P0.rows() != L || P0.cols() != L)
        throw std::runtime_error("P must have the same number of rows and "
            "columns as x has elements");

    // Initialize the state vector and covariance matrix
    x = x0;
    P = P0;

    // Calculate the mean and covariance weight vectors
    lambda = a*a * (L + k) - L;
    Wm = 1.0 / (2.0*(L + lambda)) * MatrixXd::Ones(2*L+1, 1);
    Wc = Wm;
    Wm(0) = lambda / (L + lambda);
    Wc(0) = lambda / (L + lambda) + (1.0 - a*a + B);

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void UkfAdd::predict(std::function<VectorXd(VectorXd)> f, const MatrixXd& Q)
{

    // Calculate the sigma points
    MatrixXd X = get_sigma_points(x, P);

    // Tranform the sigma points through the nonlinear prediction function
    for (int i = 0; i < X.cols(); i++)
        X.col(i) = f(X.col(i));

    // Calculate the predicted state mean using the weighted sum
    x.setZero();
    for (int i = 0; i < X.cols(); i++)
        x += Wm(i)*X.col(i);

    // Calculate the predicted state covariance using the weighted sum
    P.setZero();
    for (int i = 0; i < X.cols(); i++)
        P += Wc(i)*(X.col(i) - x)*((X.col(i) - x).transpose());

    // Add the process noise
    P += Q;

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void UkfAdd::update(std::function<VectorXd(VectorXd)> h, const MatrixXd& R,
    const VectorXd& z)
{

    // Calculate the sigma points
    MatrixXd X = get_sigma_points(x, P);

    // Tranform the sigma points through the nonlinear measurement function
    MatrixXd Y(z.size(), X.cols());
    for (int i = 0; i < X.cols(); i++)
        Y.col(i) = h(X.col(i));

    // Calculate the predicted measurement mean using the weighted sum
    VectorXd y = VectorXd::Zero(z.size());
    for (int i = 0; i < Y.cols(); i++)
        y += Wm(i)*Y.col(i);

    // Calculate the predicted measurement covariance and cross-covariance using
    // the weighted sum
    MatrixXd Pyy = MatrixXd::Zero(z.size(), z.size());
    MatrixXd Pxy = MatrixXd::Zero(X.rows(), z.size());
    for (int i = 0; i < Y.cols(); i++)
    {
        Pyy += Wc(i)*(Y.col(i) - y)*((Y.col(i) - y).transpose());
        Pxy += Wc(i)*(X.col(i) - x)*((Y.col(i) - y).transpose());
    }

    // Add the measurement noise
    Pyy += R;

    // Calculate the Kalman gain and update the state vector and its
    // covariance matrix
    MatrixXd K = Pxy*Pyy.inverse();
    x = x + K*(z - y);
    P = P - K*Pyy*(K.transpose());

}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the UKF state vector.
// Returns:     UKF state vector.
//------------------------------------------------------------------------------
VectorXd UkfAdd::get_state() const
{
    return x;
}

//------------------------------------------------------------------------------
// Name:        get_state_covariance
// Description: Gets the UKF state covariance matrix.
// Returns:     UKF state covariance matrix.
//------------------------------------------------------------------------------
MatrixXd UkfAdd::get_state_covariance() const
{
    return P;
}

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
MatrixXd UkfAdd::get_sigma_points(VectorXd xs, MatrixXd Ps)
{

    // Number of elements in the state vector
    int L = xs.size();

    // Ensure that P has the same number of rows and columns as the number of
    // elements in x
    if (Ps.rows() != L || Ps.cols() != L)
        throw std::runtime_error("P must have the same number of rows and "
            "columns as x has elements");

    // Calculate the vector of sigma points (L x 2L+1)
    MatrixXd X(L, 2*L+1);
    MatrixXd A = (L + lambda)*Ps;
    A = A.llt().matrixL();
    X << xs, xs.replicate(1, A.cols()) + A, xs.replicate(1, A.cols()) - A;

    return X;

}
