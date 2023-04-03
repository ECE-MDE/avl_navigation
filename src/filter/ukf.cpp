//==============================================================================
// Autonomous Vehicle Library
//
// PURPOSE: Basic C++ Unscented Kalman Filter (UKF) implementation using the
//          Eigen linear algebra library. This UKF implementation assumes
//          additive process and measurement noise.
//
// AUTHOR:  stkrauss
//==============================================================================

#include <avl_navigation/filter/ukf.h>

// Eigen includes
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        Ukf constructor
//------------------------------------------------------------------------------
Ukf::Ukf()
{

}

//------------------------------------------------------------------------------
// Name:        Ukf destructor
//------------------------------------------------------------------------------
Ukf::~Ukf()
{

}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the UKF with initial state and its covariance.
// Arguments:   - x0: initial state vector (N x 1)
//              - P0: initial state covariance matrix (N x N)
//------------------------------------------------------------------------------
void Ukf::init(const VectorXd &x0, const MatrixXd &P0)
{
    std::vector<StateType> state_types;
    init(x0, P0, state_types);
}

//------------------------------------------------------------------------------
// Name:        init
// Description: Initializes the UKF with initial state and its covariance.
// Arguments:   - x0: initial state vector (N x 1)
//              - P0: initial state covariance matrix (N x N)
//              - state_types: Vector of state types indicating the type of
//                each state (standard or angle) (N)
//------------------------------------------------------------------------------
void Ukf::init(const VectorXd &x0, const MatrixXd &P0,
    std::vector<StateType> state_types)
{

    // Number of elements in the state vector
    int L = x0.size();

    // Ensure that P0 has the same number of rows and columns as the number of
    // elements in x0
    if (P0.rows() != L || P0.cols() != L)
    throw std::runtime_error("P must have the same number of rows and "
    "columns as x has elements");

    // Ensure that the vector of state types has the same number of elements
    // as there are states
    if (!state_types.empty() && static_cast<int>(state_types.size()) != L)
        throw std::runtime_error("state types must have the same number of "
            "elements as the state");

    // Initialize the state vector and covariance matrix
    this->state_types = state_types;
    x = x0;
    P = P0;

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
void Ukf::predict(std::function<VectorXd(VectorXd, VectorXd)> f,
    const MatrixXd &Pv)
{

    // std::cout << "================================================" << std::endl << std::endl;
    // std::cout << "Before Predict" << std::endl << std::endl;
    // std::cout << "x: " << std::endl <<  x << std::endl << std::endl;
    // std::cout << "P: " << std::endl <<  P << std::endl << std::endl;
    // std::cout << "================================================" << std::endl << std::endl;

    // Ensure that Pv is a square matrix
    if (Pv.rows() != Pv.cols())
        throw std::runtime_error("Pv must be a square matrix");

    // Number of elements in the state vector and process noise matrix
    int L = x.size();
    int V = Pv.rows();

    // Create the augmented state vector
    VectorXd xa(L+V);
    xa.head(L) = x;
    xa.tail(V) = VectorXd::Zero(V);

    // Create the augmented covariance matrix
    MatrixXd Pa = MatrixXd::Zero(L+V, L+V);
    Pa.topLeftCorner(L,L) = P;
    Pa.bottomRightCorner(V,V) = Pv;

    // Calculate the sigma points and weights for the augmented state vector
    VectorXd Wm, Wc;
    MatrixXd Xa = get_sigma_points(xa, Pa, Wm, Wc);

    // std::cout << "================================================" << std::endl << std::endl;
    // std::cout << "Sigmas" << std::endl << std::endl;
    // std::cout << "Xa: " << std::endl << Xa << std::endl << std::endl;
    // std::cout << "Wm: " << std::endl << Wm << std::endl << std::endl;
    // std::cout << "Wc: " << std::endl << Wc << std::endl << std::endl;
    // std::cout << "================================================" << std::endl << std::endl;

    // Tranform the sigma points through the nonlinear prediction function
    MatrixXd Xx(L, Xa.cols());
    for (int i = 0; i < Xa.cols(); i++)
        Xx.col(i) = f(Xa.col(i).head(L), Xa.col(i).tail(V));

    // Address angle wrapping of any state that is marked as an angle
    if (!state_types.empty())
    {
        for (int i = 0; i < L; i++)
            if (state_types.at(i) == STATE_ANGLE_RAD)
                for (int j = 1; j < Xx.cols(); j++)
                    Xx(i,j) = avl::wrap_to_pi(Xx(i,j) - Xx(i,0)) + Xx(i,0);
            else if (state_types.at(i) == STATE_ANGLE_DEG)
                for (int j = 1; j < Xx.cols(); j++)
                    Xx(i,j) = avl::wrap_to_180(Xx(i,j) - Xx(i,0)) + Xx(i,0);
    }

    // Calculate the predicted state mean using the weighted sum
    x.setZero();
    for (int i = 0; i < Xx.cols(); i++)
        x = x + Wm(i)*Xx.col(i);

    // Calculate the predicted state covariance using the weighted sum
    P.setZero();
    for (int i = 0; i < Xx.cols(); i++)
        P = P + Wc(i)*(Xx.col(i) - x)*((Xx.col(i) - x).transpose());

    // std::cout << "================================================" << std::endl << std::endl;
    // std::cout << "After Predict" << std::endl << std::endl;
    // std::cout << "x: " << std::endl <<  x << std::endl << std::endl;
    // std::cout << "P: " << std::endl <<  P << std::endl << std::endl;
    // std::cout << "================================================" << std::endl << std::endl;

    // Check for divergence
    if (!P.allFinite())
        throw std::runtime_error("predict: divergence detected");

}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo Ukf::update(std::function<VectorXd(VectorXd, VectorXd)> h,
    const MatrixXd &Pn, const VectorXd &z)
{
    int Z = z.size();
    return update(h, Pn, z, VectorXd::Zero(Z));
}

//------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------
MeasInfo Ukf::update(std::function<VectorXd(VectorXd, VectorXd)> h,
    const MatrixXd &Pn, const VectorXd &z, VectorXd threshold)
{

    // std::cout << "================================================" << std::endl << std::endl;
    // std::cout << "Before Update " << std::endl << std::endl;
    // std::cout << "x: " << std::endl <<  x << std::endl << std::endl;
    // std::cout << "P: " << std::endl <<  P << std::endl << std::endl;
    // std::cout << "z: " << std::endl <<  z << std::endl << std::endl;
    // std::cout << "================================================" << std::endl << std::endl;

    // Ensure that Pn is a square matrix
    if (Pn.rows() != Pn.cols())
        throw std::runtime_error("Pn must be a square matrix");

    // Number of elements in the state vector, measurement vector, and
    // measurement noise matrix
    int L = x.size();
    int Z = z.size();
    int N = Pn.rows();

    // Create the augmented state vector
    VectorXd xa(L+N);
    xa.head(L) = x;
    xa.tail(N) = VectorXd::Zero(N);

    // Create the augmented covariance matrix
    MatrixXd Pa = MatrixXd::Zero(L+N, L+N);
    Pa.topLeftCorner(L,L) = P;
    Pa.bottomRightCorner(N,N) = Pn;

    // Calculate the sigma points and weights for the augmented state vector
    VectorXd Wm, Wc;
    MatrixXd Xa = get_sigma_points(xa, Pa, Wm, Wc);
    MatrixXd Xx = Xa.block(0,0,L,Xa.cols());

    // std::cout << "================================================" << std::endl << std::endl;
    // std::cout << "Sigmas" << std::endl << std::endl;
    // std::cout << "Xa: " << std::endl << Xa << std::endl << std::endl;
    // std::cout << "Wm: " << std::endl << Wm << std::endl << std::endl;
    // std::cout << "Wc: " << std::endl << Wc << std::endl << std::endl;
    // std::cout << "================================================" << std::endl << std::endl;

    // Tranform the sigma points through the nonlinear measurement function
    MatrixXd Y(Z, Xa.cols());
    for (int i = 0; i < Xa.cols(); i++)
        Y.col(i) = h(Xa.col(i).head(L), Xa.col(i).tail(Z));

    // Ensure that the meaurement function output matches the measurement
    if (Y.rows() != z.size())
        throw std::runtime_error("update: measurement function must return same number of states as measurement");

    // Calculate the predicted measurement mean using the weighted sum
    VectorXd y = VectorXd::Zero(Y.rows());
    for (int i = 0; i < Y.cols(); i++)
        y = y + Wm(i)*Y.col(i);

    // Calculate the predicted measurement covariance and cross-covariance using
    // the weighted sum
    MatrixXd Pyy = MatrixXd::Zero(Y.rows(), Y.rows());
    MatrixXd Pxy = MatrixXd::Zero(Xx.rows(), Y.rows());
    for (int i = 0; i < Y.cols(); i++)
    {
        Pyy = Pyy + Wc(i)*(Y.col(i) - y)*((Y.col(i) - y).transpose());
        Pxy = Pxy + Wc(i)*(Xx.col(i) - x)*((Y.col(i) - y).transpose());
    }

    // Kalman gain
    MatrixXd K = Pxy*Pyy.inverse();

    // Determine whether the measurement should be accepted based on the
    // measurement stddev and the threshold. Each measurement element will
    // be checked individually with its corresponding threshold
    VectorXd inno = z - y;
    VectorXd meas_stddev = Pyy.diagonal().cwiseSqrt();
    bool accepted = true;
    for (int i = 0; i < Z; i++)
    {
        if (threshold(i) != 0 && abs(inno(i)) > threshold(i)*meas_stddev(i))
        {
            accepted = false;
            break;
        }
    }

    // Only update the state and covariance if the measurement is accepted
    if (accepted)
    {

        // Update the state and covariance
        x = x + K*(z - y);
        P = P - K*Pyy*(K.transpose());

        // Check for divergence
        if (!P.allFinite())
            throw std::runtime_error("update: divergence detected");

    }

    // std::cout << "================================================" << std::endl << std::endl;
    // std::cout << "After Update" << std::endl << std::endl;
    // std::cout << "x: " << std::endl <<  x << std::endl << std::endl;
    // std::cout << "P: " << std::endl <<  P << std::endl << std::endl;
    // std::cout << "================================================" << std::endl << std::endl;

    // Construct and return the measurement info struct
    MeasInfo info;
    info.y = z;
    info.y_bar = y;
    info.P_yy = Pyy;
    info.innovation = inno;
    info.threshold = threshold;
    info.accepted = accepted;
    return info;

}

//------------------------------------------------------------------------------
// Name:        get_state
// Description: Gets the UKF state vector.
// Returns:     UKF state vector.
//------------------------------------------------------------------------------
VectorXd Ukf::get_state() const
{
    return x;
}

//------------------------------------------------------------------------------
// Name:        get_cov
// Description: Gets the UKF state covariance matrix.
// Returns:     UKF state covariance matrix.
//------------------------------------------------------------------------------
MatrixXd Ukf::get_cov() const
{
    return P;
}

//--------------------------------------------------------------------------
// Name:        get_sigma_points
// Description: Updates the standard 2N+1 sigma points and weight vectors
//              used to represent the mean and covariance of the state
//              vector.
//--------------------------------------------------------------------------
MatrixXd Ukf::get_sigma_points(VectorXd xs, MatrixXd Ps, VectorXd& Wm,
    VectorXd& Wc)
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
    double lambda = a*a * (L + k) - L;
    MatrixXd A = (L + lambda)*Ps;
    A = A.llt().matrixL();
    X << xs, xs.replicate(1, A.cols()) + A, xs.replicate(1, A.cols()) - A;

    // Calculate the mean and covariance weight vectors

    Wm = 1.0 / (2.0*(L + lambda)) * MatrixXd::Ones(2*L+1, 1);
    Wc = Wm;
    Wm(0) = lambda / (L + lambda);
    Wc(0) = lambda / (L + lambda) + (1.0 - a*a + B);

    // std::cout << "L: " <<  L << std::endl << std::endl;
    // std::cout << "lambda: " <<  lambda << std::endl << std::endl;
    // std::cout << "a: " <<  a << std::endl << std::endl;
    // std::cout << "k: " <<  k << std::endl << std::endl;
    // std::cout << "B: " <<  B << std::endl << std::endl;



    return X;

}
