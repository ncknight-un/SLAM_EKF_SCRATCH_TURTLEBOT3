#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief turtlelib Trasformation conversion to SVG formats

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <string>
#include <vector>

namespace turtlelib
{
    /// \brief a represenation of Points Vectors and Frames in a vector-graphics file format
    class Svg {
        public:
            /// \brief Create an an SVG Object
            Svg();

            /// \brief Draw a Point
            /// \param cx - The x coordinate of the circle's center in the viewBox frame.
            /// \param cy - The y coordinate of the circle's center in the viewBox frame.
            /// \param stroke - The color of the outline of the circle.
            /// \param fill - the color of the fill of the circle.
            void draw_point(double cx, double cy, std::string stroke, std::string fill);
            
            /// \brief Draw a vector: 
            /// \param x1 - The head's x coordinate in the viewBox frame.
            /// \param y1 - The head's y coordinate in the viewBox frame.
            /// \param x2 - The tail's x coordinate in the viewBox frame.
            /// \param y2 - The tail's y coordinate in the viewBox frame.
            void draw_vector(double x1, double y1, double x2, double y2, std::string stroke);

            /// \brief Draw a Frame:
            /// \param v1 - the first vector to draw in the viewbox
            /// \param v2 - the second vector to draw in the viewbox
            /// \param frame_id -The body of the <text> tag contains the text to display
            void draw_frame(Vector2D v1, Vector2D, char frame_id);

            /// \brief Writes a valid SVG file to an inputted filename
            /// \param filename - name of file to save the svg.
            void write_to_file(const std::string & filename) const;

        private:
            // Declare Private variable to hold Vectors, Points, and Frames as a list of strings in SVG Format
            std::vector<std::string> commands_; 
            double to_svg_coords_x(const double & x) const; // Conversion of x coords
            double to_svg_coords_y(const double & y) const; // Conversion of y coords
    };
}
#endif // TURTLELIB_SVG_INCLUDE_GUARD_HPP