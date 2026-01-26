/// \file
/// \brief Program for creating and visualizing transforms, Points, and vectors!
///
/// This program prompts the user to enter two transforms then computes the resulting transform tables, 
/// following this, the user is prompted to enter a vector and point, where visualization will be computed. 
/// The user will have the ability to save the SVG of the resulting output from the input.
///
/// Usage:
///
/// Example:
///   Input:  
///   Output: 
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <iostream>
#include <string>

turtlelib::Transform2D input_transform(const std::string &name) {
    double trans_x, trans_y, rot;

    // Get the data from the user:
    std::cerr << "Enter the transform " << name << ":\n";
    std::cerr << "Translation x y: ";
    std::cin >> trans_x >> trans_y;
    std::cerr << "Rotation (radians): ";
    std::cin >> rot;

    // Construct Transform2D using Vector2D for translation
    return turtlelib::Transform2D(turtlelib::Vector2D{trans_x, trans_y}, rot);
} // End of input_transform

turtlelib::Point2D input_point(const std::string &name) {
    double p_x, p_y;

    // Get the data from the user:
    std::cerr << "Please Enter Point " << name << ":\n";
    std::cerr << "Point x y: ";
    std::cin >> p_x >> p_y;

    // Construct Transform2D using Vector2D for translation
    return turtlelib::Point2D(p_x, p_y);
} // End of input_point

void print_tf(turtlelib::Transform2D tf) {
    // Print the Transform
    std::cout << "x: " << tf.translation().x << ", y: " << tf.translation().y << std::endl;
    std:: cout << "rot: " << tf.rotation() << std::endl;
    std::cout << std::endl;
} // End of print_tf

void draw_tf_frame(turtlelib::Svg &svg, const turtlelib::Transform2D &tf, char id) {
    // Unit axes
    turtlelib::Vector2D x_axis{1.0, 0.0};
    turtlelib::Vector2D y_axis{0.0, 1.0};

    double c = std::cos(tf.rotation());
    double s = std::sin(tf.rotation());

    // Rotate axes using rotation matrix
    turtlelib::Vector2D x_rotated{
        c * x_axis.x - s * x_axis.y,
        s * x_axis.x + c * x_axis.y
    };

    turtlelib::Vector2D y_rotated{
        c * y_axis.x - s * y_axis.y,
        s * y_axis.x + c * y_axis.y
    };

    // Origin of the frame relative to world origin
    turtlelib::Vector2D origin = tf.translation();

    // Draw frame: check that axes unit length and orthogonal
    svg.draw_frame(origin, x_rotated, y_rotated, id);
} // End of draw_tf_frames


int main() {
    // Get the First Transform:
    turtlelib::Transform2D Tab = input_transform("Tab");

    // Get the Second Transform:
    turtlelib::Transform2D Tbc = input_transform("Tbc");

    // Calcualte the Resulting Transforms:
    turtlelib::Transform2D Tac = Tab;  // make a copy of Tab
    Tac *= Tbc;                         // Calculate Tac
    turtlelib::Transform2D Tba = Tab;  // make a copy of Tab
    Tba = Tba.inv();                    // Get inverse to set Tba
    turtlelib::Transform2D Tcb = Tbc;  // make a copy of Tbc
    Tcb = Tbc.inv();                    // Get inverse to set Tcb
    turtlelib::Transform2D Tca = Tac;  // make a copy of Tab
    Tca = Tca.inv();                     // Get inverse to set Tcb
    
    // Output the Resulting Transforms:
    std::cout << "Tab:" << std::endl;
    print_tf(Tab);
    std::cout << "Tba:" << std::endl;
    print_tf(Tba);
    std::cout << "Tbc:" << std::endl;
    print_tf(Tbc);
    std::cout << "Tcb:" << std::endl;
    print_tf(Tcb);
    std::cout << "Tac:" << std::endl;
    print_tf(Tac);
    std::cout << "Tca:" << std::endl;
    print_tf(Tca);

    // Iniitialize the svg file construct:
    turtlelib::Svg svg;

    // Set Frame a to (0,0):
    // Draw a frame at the origin with x = (1,0), y = (0,1), id = 'a'
    turtlelib::Vector2D x_axis{1.0, 0.0};
    turtlelib::Vector2D y_axis{0.0, 1.0};
    svg.draw_frame(turtlelib::Vector2D{0.0,0.0}, x_axis, y_axis, 'a');
    // Draw Frame b:
    draw_tf_frame(svg, Tab, 'b');
    // Draw fram c:
    draw_tf_frame(svg, Tac, 'c');

    // Ask the user for a point Pa in {a}:
    turtlelib::Point2D pa = input_point("pa");

    // Compute the point in B and c: 
    turtlelib::Point2D pb = pa; // copy pa
    turtlelib::Point2D pc = pa; // copy pa
    pb = Tba(pa);
    pc = Tca(pa);

    // Insert the points into the SVG:
    svg.draw_point(pa.x, pa.y, "purple", "purple");
    svg.draw_point(pb.x, pb.y, "brown", "brown");
    svg.draw_point(pc.x, pc.y, "orange", "orange");

    // Close and save the SVG:
    // Build the file: 
    std::string file_contents = svg.build_file();
    // Write to the SVG file
    svg.write_to_file("frames.svg", file_contents);



    return 0;
}
