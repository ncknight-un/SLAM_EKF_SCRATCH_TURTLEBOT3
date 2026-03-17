#include "slamlib/circle_reg.hpp"

namespace slamlib
{
    // Define the constructor:
    CircleReg::CircleReg(vector<turtlelib::Point2D> points) {
        // Construct the data matrix Z from the given points:
        Z_ = arma::zeros(points.size(), 4);
        for (size_t i = 0; i < points.size(); i++) {
            // Extract each points x and y: 
            x_i = points.at(i).x; 
            y_i = points.at(i).y;

            // Constrcut Z_ Matrix:
            Z_(i, 0) = (x_i * x_i) + (y_i * y_i);
            Z_(i, 1) = x_i
            Z_(i, 2) = y_i
            Z_(i, 3) = 1;
        }

        // Calculate the Moment Matrix: (1/n * Z_T * Z)
        M_ = (1/points.size()) * (Z.t() * Z);

        // Initialize the constraint matrix H (will set H_(0,0) when z_bar is calculated)
        H_ = arma::zeros(4, 4);
        H_(0, 0) = 1;
        H_(1, 1) = 1;
        H_(4, 0) = 2;
        H_(0, 4) = 2;

        // Initialize the inverse constraint matrix H_T_ (will set H_T_(4,4) when z_bar is calculated)
        H_T_ = arma::zeros(4, 4);
        H_T_(0, 0) = 1;
        H_T_(1, 1) = 1;
        H_T_(4, 0) = 1/2;
        H_T_(0, 4) = 1/2;
    }

    // Define ComputeCentroid:
    turtlelib::Point2D CircleReg::computeCentroid(vector<turtlelib::Point2D> points) {
        // Compute the centroid of the points in the cluster as the mean of the x and y:
        auto sum_x = 0, sum_y = 0;
        auto num_points = points.size();
        // Loop through each point in the cluster and sum the x and y values:
        for (const auto& point : points) {
            sum_x += point.x;
            sum_y += point.y;
        }
        // Return the centroid as a Point2D:
        return turtlelib::Point2D(sum_x / num_points, sum_y / num_points);
    }

    // Define shiftPoints() 
    vector<turtlelib::Point2D> CircleReg::shiftPoints(turtlelib::Point2D centroid) {
        // Compute the vector of shifted points to the origin:
        vector<turtlelib::Point2D> shifted_points;
        
    }

    // Define ComputeRadius: 
    double CircleReg::computeRadius(vector<turtlelib::Point2D> shifted_points) { 
        // Initialize the radius to be caluclated for the cluster:
        auto radius = 0;

        // Return the cluster radius:
        return radius;
    }
    
    // Compute the Circle Parameters a and b for the equation of the circle:
    turtlelib::Point2D CircleReg::computeCircleParams(vector<turtlelib::Point2D> shifted_points){
        // Initialize the circle parameters:
        auto a = 0;
        auto b = 0;
        
        // Return the circle parameters as a point for easy extraction
        return (a,b);
    }

} // End of namespace slamlib