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

int main() {
    // Establish variables to store the user input:
    double transx1, transy1, transx2, transy2;
    double rot1, rot2; 
    double pax, pay;

    // Get the First Transform:
    std::cerr << "Enter the first transform:\n";
    std::cerr << "Please Enter Tab: Translation x y: ";
    std::cin >> transx1 >> transy1;
    std::cerr << "Rotation (radians): ";
    std::cin >> rot1;

    // Get the Second Transform:
    std::cerr << "\nEnter the second transform:\n";
    std::cerr << "Translation x y: ";
    std::cin >> transx2 >> transy2;
    std::cerr << "Rotation (radians): ";
    std::cin >> rot2;

    // Contruct the Transforms:      
    turtlelib::Transform2D Tab = turtlelib::Transform2D(turtlelib::Vector2D(transx1, transy1), rot1);
    turtlelib::Transform2D Tbc = turtlelib::Transform2D(turtlelib::Vector2D(transx2, transy2), rot2);

    // Calcualte the Resulting Transforms:
    turtlelib::Transform2D Tac = Tab;  // make a copy of Tab
    Tac *= Tbc;                         // now Tac is Tab composed with Tbc


    // Iniitialize the svg file construct:
    turtlelib::Svg svg;

    // Ask the user for a point Pa in {a}:
    std::cerr << "Please Enter Point P_a: Point x y: ";
    std::cin >> pax >> pay;

    return 0;
}
