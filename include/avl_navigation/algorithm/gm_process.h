//==============================================================================
// Autonomous Vehicle Library
//
// Description: Pure virtual base class for navigation filters.
//==============================================================================

#ifndef GM_PROCESS_H
#define GM_PROCESS_H

// Util functions
#include <avl_core/util/matrix.h>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class GmProcess
{

public:

    //--------------------------------------------------------------------------
    // Name:        GmProcess constructor
    //--------------------------------------------------------------------------
    GmProcess(double cov, double T, double dt);

    //--------------------------------------------------------------------------
    // Name:        GmProcess destructor
    //--------------------------------------------------------------------------
    ~GmProcess();

    //--------------------------------------------------------------------------
    // Name:        get_driving_cov
    // Description: Gets the GM process driving noise covariance.
    // Returns:     GM process driving noise covariance.
    //--------------------------------------------------------------------------
    double get_driving_cov();

    //--------------------------------------------------------------------------
    // Name:        Generate
    // Description: Generates a GM process realization.
    // Arguments:   - num_states: Number of GM process states to generate (rows
    //                in output).
    //              - N: Number of iterations to generate.
    // Returns:     Matrix of GM process states. Each row is a state.
    //--------------------------------------------------------------------------
    MatrixXd generate(int num_states, int N);

private:

    // Process covariance
    double cov;

    // Process time constant
    double T;

    // Time step
    double dt;

    // Driving noise covariance
    double cov_w;

};

#endif // GM_PROCESS_H
