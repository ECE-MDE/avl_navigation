//==============================================================================
// Autonomous Vehicle Library
//
// Description: Manifold Unscented Kalman Filter (MUKF) implementation using the
//              Eigen linear algebra library. The manifold UKF is described in
//              the following paper:
//
//    M. Brossard, A. Barrau and S. Bonnabel, "A Code for Unscented Kalman
//    Filtering on Manifolds (UKF-M)," 2020 IEEE International Conference on
//    Robotics and Automation (ICRA), Paris, France, 2020, pp. 5701-5708,
//    doi: 10.1109/ICRA40945.2020.9197489.
//
//              The code is adapted from the MATLAB implementation:
//                  https://github.com/CAOR-MINES-ParisTech/ukfm
//==============================================================================

#ifndef MUKF_H
#define MUKF_H

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Measurement info struct
#include "meas_info.h"

#include <iostream>

//==============================================================================
//                              STRUCT DECLARATION
//==============================================================================

// Struct containing UKF weights
struct UkfWeights
{
    double lambda;
    double sqrt_lambda;
    double wj;
    double wm0;
    double wc0;
};

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================
template <typename State, typename Input>
class Mukf
{

    // Typedefs for function pointers
    typedef std::function<State(State, VectorXd)> PhiFunc;
    typedef std::function<VectorXd(State, State)> PhiInvFunc;
    typedef std::function<State(State, Input, VectorXd)> FFunc;
    typedef std::function<VectorXd(State)> HFunc;

public:

    //--------------------------------------------------------------------------
    // Name:        Mukf constructor
    //--------------------------------------------------------------------------
    Mukf()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        Mukf destructor
    //--------------------------------------------------------------------------
    virtual ~Mukf()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the MUKF with initial state, state covariance.
    // Arguments:   - state0: Initial state.
    //              - P0: Initial state covariance matrix.
    //              - Q: Process noise covariance matrix.
    //              - phi: Retraction function.
    //              - phi_inv: Inverse retraction function.
    //--------------------------------------------------------------------------
    void init(const State &state0, const MatrixXd &P0, const MatrixXd& Q,
        PhiFunc phi, PhiInvFunc phi_inv)
    {

        // Initialize variables
        state = state0;
        P = P0;
        Q_chol = chol(Q);
        this->phi = phi;
        this->phi_inv = phi_inv;

        // Matrix sizes
        d = P.rows();
        q = Q.rows();

        // ---------------------------------------------------------------------
        // Construct the UKF weights

        w_d.lambda = (alpha*alpha - 1.0) * d;
        w_d.sqrt_lambda = sqrt(d + w_d.lambda);
        w_d.wj = 1.0 / (2.0*(d + w_d.lambda));
        w_d.wm0 = w_d.lambda/(w_d.lambda + d);
        w_d.wc0 = w_d.lambda/(w_d.lambda + d) + 3.0 - alpha*alpha;

        w_q.lambda = (alpha*alpha - 1.0) * q;
        w_q.sqrt_lambda = sqrt(q + w_q.lambda);
        w_q.wj = 1.0 / (2.0*(q + w_q.lambda));
        w_q.wm0 = w_q.lambda/(w_q.lambda + q);
        w_q.wc0 = w_q.lambda/(w_q.lambda + q) + 3.0 - alpha*alpha;

        w_u.lambda = (alpha*alpha - 1.0) * d;
        w_u.sqrt_lambda = sqrt(d + w_u.lambda);
        w_u.wj = 1.0 / (2.0*(d + w_u.lambda));
        w_u.wm0 = w_u.lambda/(w_u.lambda + d);
        w_u.wc0 = w_u.lambda/(w_u.lambda + d) + 3.0 - alpha*alpha;

    }

    //--------------------------------------------------------------------------
    // Name:        predict
    // Description: Predicts the next state and state covariance using the
    //              nonlinear state transition function.
    // Arguments:   - f: nonlinear state transition function
    //                (in: N x 1, out: N x 1)
    //              - Q: process noise covariance matrix (N x N)
    //--------------------------------------------------------------------------
    void predict(FFunc f, Input input)
    {

        // Ensure positive-definiteness of state covariance matrix
        // P += EPS * MatrixXd::Identity(d,d);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Propagate the state mean

        VectorXd w = VectorXd::Zero(q);
        State state_new = f(state, input, w);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Calculate state covariance

        // Get sigma point distribution
        MatrixXd xis = w_d.sqrt_lambda * chol(P);

        // Retract sigma points onto manifold and propagate them
        MatrixXd xis_new = MatrixXd::Zero(d, 2*d);
        for (int j = 0; j < d; j++)
        {
            State s_j_p = phi(state,  xis.col(j));
            State s_j_m = phi(state, -xis.col(j));
            State s_j_p_new = f(s_j_p, input, w);
            State s_j_m_new = f(s_j_m, input, w);
            xis_new.col(j)     = phi_inv(state_new, s_j_p_new);
            xis_new.col(d + j) = phi_inv(state_new, s_j_m_new);
        }

        // std::cout << "======================" << std::endl << std::endl;
        // std::cout << P << std::endl << std::endl;
        // std::cout << chol(P) << std::endl << std::endl;
        // std::cout << xis << std::endl << std::endl;
        // std::cout << xis_new << std::endl << std::endl;
        // std::cout << "======================" << std::endl << std::endl;

        // Compute sigma point mean
        VectorXd xi_mean = w_d.wj * xis_new.rowwise().sum();

        // Calculate state covariance
        xis_new = xis_new.colwise() - xi_mean;
        MatrixXd P_new = w_d.wj  * xis_new*xis_new.transpose() +
                         w_d.wc0 * xi_mean*xi_mean.transpose();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Calculate state covariance due to input noise

        // Compute covariance w.r.t. noise
        xis_new = MatrixXd::Zero(d, 2*q);

        // Rectract sigma points onto manifold
        for (int j = 0; j < q; j++)
        {
            VectorXd w_p =  w_q.sqrt_lambda * Q_chol.col(j);
            VectorXd w_m = -w_q.sqrt_lambda * Q_chol.col(j);
            State s_j_p_new = f(state, input, w_p);
            State s_j_m_new = f(state, input, w_m);
            xis_new.col(j)     = phi_inv(state_new, s_j_p_new);
            xis_new.col(q + j) = phi_inv(state_new, s_j_m_new);
        }

        // Compute sigma point mean
        xi_mean = w_q.wj * xis_new.rowwise().sum();

        // Compute noise covariance
        xis_new = xis_new.colwise() - xi_mean;
        MatrixXd Q_new = w_q.wj  * xis_new*xis_new.transpose() +
                         w_q.wc0 * xi_mean*xi_mean.transpose();

        // Update state mean and covariance
        P = P_new + Q_new;
        state = state_new;

        // Check for divergence
        if (!P.allFinite())
            throw std::runtime_error("predict: divergence detected");

    }

    //--------------------------------------------------------------------------
    // Name:        update
    // Description: Updates the state and state covariance from a measurement
    //              using the given nonlinear measurement function and its noise
    //              covariance matrix.
    // Arguments:   - y: Measurement value.
    //              - h: nonlinear measurement function.
    //              - R: Measurement noise covariance matrix.
    // Returns:     Structure containing info about the measurement update step.
    //--------------------------------------------------------------------------
    MeasInfo update(const VectorXd &y, HFunc h, const MatrixXd &R)
    {
        VectorXd threshold = VectorXd::Zero(y.size());
        return update(y, h, R, threshold);
    }

    //--------------------------------------------------------------------------
    // Name:        update
    // Description: Updates the state and state covariance from a measurement
    //              using the given nonlinear measurement function and its noise
    //              covariance matrix.
    // Arguments:   - y: Measurement value.
    //              - h: nonlinear measurement function.
    //              - R: Measurement noise covariance matrix.
    // Returns:     Structure containing info about the measurement update step.
    //--------------------------------------------------------------------------
    MeasInfo update(const VectorXd &y, HFunc h, const MatrixXd &R,
        VectorXd threshold)
    {

        // Variable sizes
        int l = y.size();

        // Check that the measurement vector and covariance matrix sizes match
        if (R.rows() != l || R.cols() != l)
            throw std::runtime_error("update: invalid dimensions of y and R");

        // Check that the threshold vector and measurement vector sizes match
        if (threshold.size() != l)
            throw std::runtime_error(
                "update: threshold dimension (" +
                std::to_string(threshold.size()) +
                ") must match measurement dimension (" +
                std::to_string(l));

        // Ensure positive-definiteness of state covariance matrix
        // P += EPS * MatrixXd::Identity(d,d);

        // Get sigma points
        MatrixXd xis = w_u.sqrt_lambda * chol(P);

        // Compute measurement sigma points
        MatrixXd ys = MatrixXd::Zero(l, 2*d);
        VectorXd y_hat = h(state);
        for (int j = 0; j < d; j++)
        {
            State chi_j_p = phi(state,  xis.col(j));
            State chi_j_m = phi(state, -xis.col(j));
            ys.col(j)     = h(chi_j_p);
            ys.col(d + j) = h(chi_j_m);
        }

        // Measurement mean
        VectorXd y_bar = w_u.wm0*y_hat + w_u.wj*ys.rowwise().sum();

        // Prune mean before computing covariance
        ys = ys.colwise() - y_bar;
        y_hat = y_hat - y_bar;

        // Compute covariance and cross covariance matrices
        MatrixXd P_yy = w_u.wj  * ys*ys.transpose() +
                        w_u.wc0 * y_hat*y_hat.transpose() +
                        R;

        MatrixXd A = MatrixXd::Zero(d, 2*d);
        A.block(0,0,d,d) =  xis;
        A.block(0,d,d,d) = -xis;
        MatrixXd P_xiy = w_u.wj*A*ys.transpose();

        // Kalman gain
        MatrixXd K = P_xiy*P_yy.inverse();

        // std::cout << "==============================" << std::endl << std::endl;
        // std::cout << ys << std::endl << std::endl;
        // std::cout << P << std::endl << std::endl;
        // std::cout << K << std::endl << std::endl;
        // std::cout << P_yy << std::endl << std::endl;
        // std::cout << K * P_yy * K.transpose() << std::endl << std::endl;
        // std::cout << "==============================" << std::endl << std::endl;

        // Determine whether the measurement should be accepted based on the
        // measurement stddev and the threshold. Each measurement element will
        // be checked individually with its corresponding threshold
        VectorXd inno = y - y_bar;
        VectorXd meas_stddev = P_yy.diagonal().cwiseSqrt();
        bool accepted = true;
        for (int i = 0; i < l; i++)
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

            // Update state
            VectorXd xi_plus = K * inno;
            state = phi(state, xi_plus);

            // Update covariance
            P = P - K * P_yy * K.transpose();
            P = (P + P.transpose()) / 2.0;

            // Check for divergence
            if (!P.allFinite())
                throw std::runtime_error("update: divergence detected");

        }

        // Construct and return the measurement info struct
        MeasInfo info;
        info.y = y;
        info.y_bar = y_bar;
        info.P_yy = P_yy;
        info.innovation = inno;
        info.threshold = threshold;
        info.accepted = accepted;
        return info;

    }

    //--------------------------------------------------------------------------
    // Name:        get_state
    // Description: Gets the MUKF state.
    // Returns:     MUKF state.
    //--------------------------------------------------------------------------
    State get_state() const
    {
        return state;
    }

    //--------------------------------------------------------------------------
    // Name:        get_cov
    // Description: Gets the MUKF state covariance matrix.
    // Returns:     MUKF state covariance matrix.
    //--------------------------------------------------------------------------
    MatrixXd get_cov()
    {
        return P;
    }

private:

    // State mean and covariance matrix tracked by the MUKF
    State state;
    MatrixXd P;

    // Retraction and inverse retraction functions
    PhiFunc phi;
    PhiInvFunc phi_inv;

    // Cholesky decomposition of input noise covariance matrix
    MatrixXd Q_chol;

    // Sizes of state and input noise covariance matrices
    int d;
    int q;

    // UKF weights for prediction and update steps
    UkfWeights w_d;
    UkfWeights w_q;
    UkfWeights w_u;

    // Sigma point distribution and scaling parameters
    double alpha = 1.0e-3;

    // Constant added to state covariance matrix for ensuring that it is
    // positive definite
    // double EPS = std::numeric_limits<double>::epsilon();
    double EPS = 1.0e-16;

private:

    //--------------------------------------------------------------------------
    // Name:        chol
    // Description: Performs a Cholesky decomposition of a symmetric, positive
    //              definite matrix A such that A = LL^T, where L is lower
    //              triangular.
    // Arguments:   - A: Symmetric, positive definite matrix to get Cholesky
    //                   decomposition of.
    // Returns :    Lower triangular Cholesky decomposition of A.
    //--------------------------------------------------------------------------
    MatrixXd chol(MatrixXd A)
    {
        return A.llt().matrixL();
    }

};

#endif // MUKF_H
