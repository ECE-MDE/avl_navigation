//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a simple moving average.
//==============================================================================

#include <avl_navigation/algorithm/moving_avg.h>

#include <iostream>
#include <math.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        MovingAvg constructor
//------------------------------------------------------------------------------
MovingAvg::MovingAvg(size_t N_max)
{
    samples = boost::circular_buffer<double>(N_max);
}

//------------------------------------------------------------------------------
// Name:        MovingAvg destructor
//------------------------------------------------------------------------------
MovingAvg::~MovingAvg()
{

}

//------------------------------------------------------------------------------
// Name:        add_sample
// Description: Adds a sample to the moving average.
// Arguments:   - sample: Sample t be added to the boving average.
// Returns:     New value of the moving average.
//------------------------------------------------------------------------------
double MovingAvg::add_sample(double sample)
{
    samples.push_back(sample);
    return get_average();
}

//------------------------------------------------------------------------------
// Name:        get_average
// Description: Gets the current value of the moving average.
// Returns:     The current value of the moving average.
//------------------------------------------------------------------------------
double MovingAvg::get_average()
{

    double avg = 0.0;
    double N = static_cast<double>(samples.size());

    for (double i : samples)
        avg += i;
    avg /= N;

    return avg;

}

//--------------------------------------------------------------------------
// Name:        get_stddev
// Description: Gets the current standard deviation for the moving average.
// Returns:     The current standard deviation of the moving average.
//--------------------------------------------------------------------------
double MovingAvg::get_stddev()
{
    double avg = this->get_average();
    double N = static_cast<double>(samples.size());
    double variance = 0;
    for(double sample : samples) {
        variance += pow(sample - avg, 2) / N;
    }
    return sqrt(variance);
}

//------------------------------------------------------------------------------
// Name:        reset
// Description: Resets the moving average.
// Arguments:   - N_max: Maximum window size.
//------------------------------------------------------------------------------
void MovingAvg::reset(size_t N_max)
{
    samples = boost::circular_buffer<double>(N_max);
}
