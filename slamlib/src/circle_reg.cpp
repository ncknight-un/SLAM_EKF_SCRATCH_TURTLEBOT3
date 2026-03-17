#include "slamlib/circle_reg.hpp"

namespace slamlib
{
    // Define the constructor:
    CircleReg::CircleReg(vector<turtlelib::Point2D> points) {
        // Construct the data matrix Z from the given points:
        Z = arma::zeros(points.size(), 3);
        for (size_t i = 0; i < points.size(); i++) {
            Z(i, 0) = points.at(i).x;
            Z(i, 1) = points.at(i).y;
            Z(i, 2) = 1;
        }

        // Compute the moment matrix M:
        M = Z.t() * Z;

        // Compute the constraint matrix H:
        H = arma::zeros(3, 3);
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 0;
    }

    // Define ComputeCentroid:
    turtlelib::Point2D CircleReg::computeCentroid() {
        // Compute the centroid of the points in the cluster as the mean of the x and y
    }

    // Define shiftPoints() 
    vector<turtlelib::Point2D> shiftPoints(turtlelib::Point2D centroid) {
        // Compute the vector of shifted points to the origin:
        vector<turtlelib::Point2D> shifted_points;
        
    }

    // Define ComputeRadius: 
    double computeRadius(vector<turtlelib::Point2D> shifted_points) { 
        // Initialize the radius to be caluclated for the cluster:
        auto radius = 0;

        // Return the cluster radius:
        return radius;
    }
    
    // Compute the Circle Parameters a and b for the equation of the circle:
    turtlelib::Point2D computeCircleParams(vector<turtlelib::Point2D> shifted_points){
        // Initialize the circle parameters:
        auto a = 0;
        auto b = 0;
        
        // Return the circle parameters as a point for easy extraction
        return (a,b);
    }

} // End of namespace slamlib