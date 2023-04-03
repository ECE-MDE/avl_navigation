//==============================================================================
// Autonomous Vehicle Library
//
// Description: Basic Extended Kalman Filter (EKF) implementation using the
//              Eigen linear algebra library.
//==============================================================================

#include <avl_navigation/filter/ekf.h>

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Ekf constructor
//------------------------------------------------------------------------------
Ekf::Ekf() {}

//------------------------------------------------------------------------------
// Name:        Ekf destructor
//------------------------------------------------------------------------------
Ekf::~Ekf() {}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the EKF with initial state and covariance,
//              state transition matrix, and process noise covariance.
// Arguments:   - x0: initial state vector (N x 1)
//              - P0: initial state covariance matrix (N x N)
//------------------------------------------------------------------------------
void Ekf::init(VectorXd x0, MatrixXd P0)
{

    // Get number of states
    N = (int)x0.rows();

    // Ensure that input matrices have the correct dimmensions
    if (P0.cols() != N && P0.rows() != N)
        throw std::runtime_error("initial state covariance must be N x N");

    this->x = x0;
    this->P = P0;

}

//------------------------------------------------------------------------------
// Name:        predict
// Description: Predicts the next state and state covariance using the
//              state transition matrix.
// Arguments:   - F: state transition matrix (N x N)
//              - Q: process noise covariance matrix (N x N)
//------------------------------------------------------------------------------
void Ekf::predict(MatrixXd F, MatrixXd Q)
{

    // Ensure that input matrices have the correct dimmensions
    if (F.cols() != N && F.rows() != N)
        throw std::runtime_error("state transition matrix must be N x N");

    if (Q.cols() != N && Q.rows() != N)
        throw std::runtime_error("process noise covariance matrix N x N");

    // Step the state and covariance forward using the state transition and
    // the previous states
    x = F*x;
    P = F*P*F.transpose() + Q;

}

//------------------------------------------------------------------------------
// Name:        update
// Description: Updates the state and state covariance using the given
//              measurement matrix and its noise covariance matrix.
// Arguments:   - H: measurement matrix (M x N)
//              - R: measurement noise covariance matrix (M x M)
//              - z: measurement value (M x 1)
//------------------------------------------------------------------------------
MeasInfo Ekf::update(const MatrixXd &H, const MatrixXd &R,
    const Eigen::VectorXd &z, VectorXd threshold)
{

    // Ensure that input matrices have the correct dimmensions
    if (H.cols() != N)
        throw std::runtime_error("measurement matrix must have N columns");

    if (R.cols() != H.rows() && R.rows() != H.rows())
        throw std::runtime_error("measurement noise covariance matrix must be M x M");

    if (H.rows() != z.size())
        throw std::runtime_error("measurement value and measurement matrix must have same number of rows");

    // Define an identity matrix to shorten notation
    MatrixXd I = MatrixXd::Identity(N, N);

    // Calculate the innovation
    VectorXd y = z - H*x;

    // Calculate the innovation covariance
    MatrixXd S = H*P*H.transpose() + R;

    // Calculate the optimal Kalman gain
    MatrixXd K = P * H.transpose() * S.inverse();

    // Determine whether the measurement should be accepted based on the
    // measurement stddev and the threshold. Each measurement element will
    // be checked individually with its corresponding threshold
    VectorXd meas_stddev = S.diagonal().cwiseSqrt();
    bool accepted = true;
    for (int i = 0; i < z.size(); i++)
    {
        if (threshold(i) != 0 && abs(y(i)) > threshold(i)*meas_stddev(i))
        {
            accepted = false;
            break;
        }
    }

    // Only update the state and covariance if the measurement is accepted
    if (accepted)
    {

        // Update the state prediction
        x = x + K*y;

        // Update the covariance prediction
        P = (I - K*H)*P;

        // Check for divergence
        if (!P.allFinite())
            throw std::runtime_error("update: divergence detected");

    }

    MeasInfo info;
    info.y = z;
    info.y_bar = H*x;
    info.P_yy = S;
    info.innovation = y;
    info.threshold = threshold;
    info.accepted = accepted;
    return info;

}
