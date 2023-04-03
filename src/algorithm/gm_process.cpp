//==============================================================================
// Autonomous Vehicle Library
//
// Description: Inertial navigation algorithm using a manifold UKF to correct
//              inertial navigation estimates with aiding sensor data. Draws
//              heavily from "Principles of GNSS, Inertial and Multisensor
//              Integrated Navigation Systems" by Paul D. Groves.
//==============================================================================

#include <avl_navigation/algorithm/gm_process.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        GmProcess constructor
//------------------------------------------------------------------------------
GmProcess::GmProcess(double cov, double T, double dt) : cov(cov), T(T), dt(dt)
{

    // Calculate driving noise covariance
    if (T != 0)
        cov_w = cov * (1.0 - exp(-2.0*dt/T));
    else
        cov_w = 0.0;

}

//------------------------------------------------------------------------------
// Name:        GmProcess destructor
//------------------------------------------------------------------------------
GmProcess::~GmProcess()
{

}

//------------------------------------------------------------------------------
// Name:        get_driving_cov
// Description: Gets the GM process driving noise covariance.
// Returns:     GM process driving noise covariance.
//------------------------------------------------------------------------------
double GmProcess::get_driving_cov()
{
    return cov_w;
}

//------------------------------------------------------------------------------
// Name:        Generate
// Description: Generates a GM process realization.
// Arguments:   - num_states: Number of GM process states to generate (rows
//                in output).
//              - N: Number of iterations to generate.
// Returns:     Matrix of GM process states. Each row is a state.
//------------------------------------------------------------------------------
MatrixXd GmProcess::generate(int num_states, int N)
{

    // Generate initial state from Gaussian distribution
    MatrixXd x = MatrixXd::Zero(num_states, N);
    x.col(0) = avl::randn(num_states, 1, cov);

    // Time constant zero indicates a constant
    if (T !=  0)
    {

        // Generate driving noise
        MatrixXd w = avl::randn(num_states, N, cov_w);

        // Propagate GM process
        for (int i = 1; i < N; i++)
            x.col(i) = exp(-dt/T) * x.col(i-1) + w.col(i);

    }
    else
    {
        x = x.col(0).replicate(1,N);
    }

    return x;

}
