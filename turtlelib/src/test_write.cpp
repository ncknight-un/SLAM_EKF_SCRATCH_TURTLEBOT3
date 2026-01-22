#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include <iostream>

int main() {
    turtlelib::Svg svg;

    // Draw a point at (1.0, -1.25) in meters, purple
    svg.draw_point(1.0, -1.25, "purple", "purple");

    // Draw a vector from (0,0) to (1,1), purple
    svg.draw_vector(1.0, 1.0, 0.0, 0.0, "purple");

    // Draw a frame at the origin with x = (1,0), y = (0,1), id = 'a'
    turtlelib::Vector2D x_axis{1.0, 0.0};
    turtlelib::Vector2D y_axis{0.0, 1.0};
    svg.draw_frame(x_axis, y_axis, 'a');

    // Write to the SVG file
    svg.write_to_file("test.svg");

    // Let the user know that the function completed and that the file was written.
    std::cout << "SVG file 'test.svg' has been generated." << std::endl;
    return 0;
}
