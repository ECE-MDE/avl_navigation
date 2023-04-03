//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a simple moving average.
//==============================================================================

#ifndef MOVING_AVG_H
#define MOVING_AVG_H

// Boost circular buffer for storing samples
#include <boost/circular_buffer.hpp>

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class MovingAvg
{

public:

    //--------------------------------------------------------------------------
    // Name:        MovingAvg constructor
    //--------------------------------------------------------------------------
    MovingAvg(size_t N_max);

    //--------------------------------------------------------------------------
    // Name:        MovingAvg destructor
    //--------------------------------------------------------------------------
    ~MovingAvg();

    //--------------------------------------------------------------------------
    // Name:        add_sample
    // Description: Adds a sample to the moving average.
    // Arguments:   - sample: Sample t be added to the boving average.
    // Returns:     New value of the moving average.
    //--------------------------------------------------------------------------
    double add_sample(double sample);

    //--------------------------------------------------------------------------
    // Name:        get_average
    // Description: Gets the current value of the moving average.
    // Returns:     The current value of the moving average.
    //--------------------------------------------------------------------------
    double get_average();

    //--------------------------------------------------------------------------
    // Name:        reset
    // Description: Resets the moving average.
    // Arguments:   - N_max: Maximum window size.
    //--------------------------------------------------------------------------
    void reset(size_t N_max);

private:

    // Buffer to store samples
    boost::circular_buffer<double> samples;

};

#endif // MOVING_AVG_H
