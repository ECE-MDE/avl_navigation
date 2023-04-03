//==============================================================================
// Autonomous Vehicle Library
//
// Description: Class representing a map of terrain altitudes. Provides
//              functions for loading and querying a terrain map. File format is
//              a matrix in csv file form with the following structure:
//                  [NaN(1x1)   Lons(1xN)
//                   Lats(Mx1)  Altitudes(MxN)]
//              Minimum latitude and longitude occurs at the bottom left of the
//              matrix.
//==============================================================================

#ifndef TERRAIN_MAP_H
#define TERRAIN_MAP_H

// Util functions
#include "avl_core/util/matrix.h"

// Eigen includes
#include <Eigen/Dense>
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class TerrainMap
{

public:

    //--------------------------------------------------------------------------
    // Name:        TerrainMap constructor
    //--------------------------------------------------------------------------
    TerrainMap()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        TerrainMap constructor
    //--------------------------------------------------------------------------
    TerrainMap(std::string filepath)
    {
        load(filepath);
    }

    //--------------------------------------------------------------------------
    // Name:        TerrainMap destructor
    //--------------------------------------------------------------------------
    virtual ~TerrainMap()
    {

    }

    //--------------------------------------------------------------------------
    // Name:        load
    // Description: Loads depth map data from a file.
    // Arguments:   - filepath: Path to the depth map file.
    //--------------------------------------------------------------------------
    void load(std::string filepath)
    {
        data = avl::csv_to_matrix(filepath, ' ');
    }

    //--------------------------------------------------------------------------
    // Name:        alt
    // Description: Gets an interpolated altitude at a given lat/lon point.
    // Arguments:   - lat: Latitude in radians.
    //              - lon: Longitude in radians.
    // Returns:     Interpolated altitude at point.
    //--------------------------------------------------------------------------
    double alt(double lat, double lon)
    {

        // For ease of notation
        double x = lon;
        double y = lat;

        // Find the four points that surround the point to be interpolated to

        // Find the index of the row and column that is the lower left of the
        // four points surrounding the point to be interpolated to
        int i, j;
        for (i = 1; data(i, 0)   > lat; i++) { }
        for (j = 1; data(0, j+1) < lon; j++) { }


        // Calculate the width and height of the four points surrounding the
        // point to be interpolated to
        double x1 = data(0,j);
        double y1 = data(i,0);
        double x2 = data(0,j+1);
        double y3 = data(i-1,0);

        double dx = x2 - x1;
        double dy = y3 - y1;

        // x and y slope for linear interpolation
        double x_bar = dx == 0.0 ? 0.0 : (x - x1) / dx;
        double y_bar = dy == 0.0 ? 0.0 : (y - y1) / dy;

        // Get the altitude of the four points surrounding the point to be
        // interpolated to
        double z1 = data(i,   j);
        double z2 = data(i,   j+1);
        double z3 = data(i-1, j);
        double z4 = data(i-1, j+1);

        // Interpolate linearly depending on whether the interpolation point is
        // in the upper left or lower right corner of the box
        if (x_bar <= y_bar)
            return z1 + (z4 - z3)*x_bar + (z3 - z2)*y_bar;
        else
            return z1 + (z2 - z1)*x_bar + (z4 - z2)*y_bar;

    }

private:

    // Terrin map data
    MatrixXd data;

};

#endif // TERRAIN_MAP_H
