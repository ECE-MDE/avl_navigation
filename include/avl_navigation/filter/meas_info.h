//==============================================================================
// Autonomous Vehicle Library
//
// Description: Struct defining info returned when a filter measurement is
//              processed.
//==============================================================================

#ifndef MEAS_INFO_H
#define MEAS_INFO_H

// Eigen includes
#include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

// C++ includes
#include <sstream>
#include <iomanip>

// Util functions
#include <avl_core/util/matrix.h>

//==============================================================================
//                              STRUCT DEFINITION
//==============================================================================

// Struct containing measurement info
struct MeasInfo
{

    VectorXd y;          // Measurement vector
    VectorXd y_bar;      // Estimated measurement vector
    MatrixXd P_yy;       // Measurement covariance matrix
    VectorXd innovation; // Measurement innovation (error) vector
    VectorXd threshold;  // Measurement rejection threshold multiplier
    bool accepted;       // Measurement accepted or rejected

    //--------------------------------------------------------------------------
    // Name:        to_string
    // Description: Returns a string representation of the measurement info.
    // Returns:     String representation of the measurement info.
    //--------------------------------------------------------------------------
    std::string to_string()
    {
        std::stringstream ss;
        ss << std::setprecision(9)
           <<        avl::to_string(y, 9)
           << " " << avl::to_string(y_bar, 9)
           << " " << avl::to_string(P_yy.diagonal(), 9)
           << " " << avl::to_string(innovation, 9)
           << " " << avl::to_string(threshold, 9)
           << " " << accepted;
       return ss.str();
    }

};

#endif // MEAS_INFO_H
