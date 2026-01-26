/// \file
/// \brief Program for creating and visualizing transforms, Points, and vectors!
///
/// This program prompts the user to enter two transforms then computes the resulting transform Tables, 
/// following this, the user is prompted to enter a vector and point, where visualization will be computed. 
/// The user will have the ability to save the SVG of the resulting output from the input.
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <iostream>
#include <string>

/// \brief Prompts the user to input a 2D transform and returns it.
/// \param name A string used to identify the transform in the user prompt.
/// \return A `turtlelib::Transform2D` object corresponding to the user's input.
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

/// \brief Prompts the user to input a 2D point and returns it.
/// \param name A string used to identify the point in the user prompt.
/// \return A `turtlelib::Point2D` object corresponding to the user's input.
turtlelib::Point2D input_point(const std::string &name) {
    double p_x, p_y;

    // Get the data from the user:
    std::cerr << "Please Enter Point " << name << ":\n";
    std::cerr << "Point x y: ";
    std::cin >> p_x >> p_y;

    // Construct Transform2D using Vector2D for translation
    return turtlelib::Point2D(p_x, p_y);
} // End of input_point

/// \brief Prompts the user to input a 2D vector and returns it.
/// \param name A string used to identify the vector in the user prompt.
/// \return A `turtlelib::Vector2D` object corresponding to the user's input.
turtlelib::Vector2D input_vector(const std::string &name) {
    double v_x, v_y;

    // Get the data from the user:
    std::cerr << "Please Enter Vector " << name << ":\n";
    std::cerr << "Vector x y: ";
    std::cin >> v_x >> v_y;

    // Construct Transform2D using Vector2D for translation
    return turtlelib::Vector2D(v_x, v_y);
} // End of input_vector

/// \brief Prints the translation and rotation components of a 2D transform.
/// \param tf The 2D transform to be printed.
void print_tf(turtlelib::Transform2D& tf) {
    // Print the Transform
    std::cout << "TF -> x: " << tf.translation().x << ", y: " << tf.translation().y << std::endl;
    std:: cout << "rot: " << tf.rotation() << std::endl;
    std::cout << std::endl;
} // End of print_tf

/// \brief Prints the components of a 2D point.
/// \param p The 2D point to be printed.
void print_point(const turtlelib::Point2D& p) {
    std::cout << "Point -> x: " << p.x << ", y: " << p.y << std::endl;
    std::cout << std::endl;
}

/// \brief Prints the components of a 2D vector.
/// \param v The 2D vector to be printed.
void print_vector(const turtlelib::Vector2D& v) {
    std::cout << "Vector -> x: " << v.x << ", y: " << v.y << std::endl;
    std::cout << std::endl;
}

/// \brief Draws a 2D coordinate frame on an SVG canvas.
/// \param svg Reference to an `Svg` object where the frame will be drawn.
/// \param tf The 2D transform representing the frame's origin and orientation.
/// \param id A character used to label or identify the frame in the SVG.
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

    // Plot all the points with respect to world: 
    turtlelib::Point2D pa_world = pa;         // already in world
    turtlelib::Point2D pb_world = Tab(pb);    // map from B tp world
    turtlelib::Point2D pc_world = Tac(pc);    // map from C to world

    // Print the location of the points w.r.t the world:
    std::cout << "pa:" << std::endl;
    print_point(pa_world);
    std::cout << "pb:" << std::endl;
    print_point(pb_world);
    std::cout << "pc:" << std::endl;
    print_point(pc_world);

    // Insert the points into the SVG:
    svg.draw_point(pa_world.x, pa_world.y, "purple", "purple");
    svg.draw_point(pb_world.x, pb_world.y, "brown", "brown");
    svg.draw_point(pc_world.x, pc_world.y, "orange", "orange");

    // Prompt the user to enter a vector Vb in frame b:
    turtlelib::Vector2D vb = input_vector("vb");

    // Normalize the vector: 
    turtlelib::Vector2D vb_norm = vb;
    vb_norm = turtlelib::normalize(vb_norm);

    // Plot the vector and normalized vector in frame b:
    svg.draw_vector(Tab.translation().x, Tab.translation().y, Tab.translation().x + vb.x, Tab.translation().y + vb.y, "brown");
    svg.draw_vector(Tab.translation().x, Tab.translation().y, Tab.translation().x + vb_norm.x, Tab.translation().y + vb_norm.y, "black");

    // Express Va and Vc: 
    turtlelib::Vector2D va = vb; // copy vb
    turtlelib::Vector2D vc = vb; // copy vb
    va = Tab(vb);
    vc = Tcb(vb);

    // Print the vectors:
    std::cout << "va:" << std::endl;
    print_vector(va);
    std::cout << "vb:" << std::endl;
    print_vector(vb);
    std::cout << "vc:" << std::endl;
    print_vector(vc);

    // Plot Va and Vc:
    svg.draw_vector(0.0,0.0,va.x, va.y, "purple");
    svg.draw_vector(Tac.translation().x, Tac.translation().y, vc.x, vc.y, "orange");

    // Close and save the SVG:
    // Build the file: 
    std::string file_contents = svg.build_file();
    // Write to the SVG file
    svg.write_to_file("frames.svg", file_contents);

    return 0;
}
