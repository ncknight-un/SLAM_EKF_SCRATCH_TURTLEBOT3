#ifndef SLAMLIB_CIRCLE_REG_HPP_INCLUDE_GUARD
#define SLAMLIB_CIRCLE_REG_HPP_INCLUDE_GUARD
/// \file
/// \brief Circle fitting implementation for SLAM.
/// \author Nolan Knight
/// 
/// This file contains the implementation of a Circle Regression Algorithm for use in SLAM.

#include <sstream>
#include <format>
#include <stdexcept>
#include <cmath>
#include <iomanip>
#include <iosfwd>
#include <vector>
#include <armadillo>
#include <turtlelib/angle.hpp>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>

namespace slamlib {
    /// \brief A class for performing Circle Regression for SLAM
    class CircleReg {
    public:
        /// \brief Construct a CircleReg object
        CircleReg(vector<turtlelib::Point2D> points);

        /// \brief Compute the x and y coordinates of the centroid of the given points in a cluster
        /// \param points: The points in the cluster to compute the centroid of
        /// \return The centroid of the points in the cluster as a Point2D  (Equation 1/2 in tmp/circle_reg.pdf)
        turtlelib::Point2D computeCentroid(vector<turtlelib::Point2D> points);

        /// \brief Shift the points, so that the centroid is at the origin, to prepare for circle fitting
        /// \param Centroid: The calculated centroid of the points in the cluster
        /// \return A vector of the shifted points (Equation 3/4 in tmp/circle_reg.pdf)
        vector<turtlelib::Point2D> shiftPoints(turtlelib::Point2D centroid);

        /// \brief Calculate the radius of the circle that best fits the given points that have been shifted.
        /// \param shifted_points: The points that have been shifted so that the centroid is at the origin 
        /// \return The radius of the circle that best fits the given points (Equation 5 in tmp/circle_reg.pdf)
        double computeRadius(vector<turtlelib::Point2D> shifted_points);

        /// \brief Update Constraint Matrix and its invers:
        /// \param cluster_radius: The mean cluster_radius prediction
        /// \return None - Updates Private H and H.T matrices.
        void updateConstraitMatrices(double cluster_radius);

        /// \brief calculate circle parameters a and b that best fit the given points that have been shifted.
        /// \param shifted_points: The points that have been shifted so that the centroid is at the origin 
        /// \param radius: The radius of the circle that best fits the given points
        /// \return The parameters a and b of the circle that best fits the given points (Equations 6-14 in tmp/circle_reg.pdf)
        turtlelib::Point2D computeCircleParams(vector<turtlelib::Point2D> shifted_points);
    private: 
        // State matrices for the circle regression:
        arma:mat Z_;    // Data Matrix
        arma:mat M_;    // Moment Matrix 
        arma:mat H_;    // Constraint Matrix
        arma:mat H_T_;  // Inversr Constraint Matrix
    };
}