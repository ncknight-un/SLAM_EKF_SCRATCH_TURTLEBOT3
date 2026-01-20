/// \file
/// \brief Program for angle input, conversion, and normalization
///
/// This program prompts the user to enter an angle with a unit ("deg" or "rad"),
/// normalizes it to the range [-π, π] for radians, converts between degrees and radians,
/// and outputs the result.
///
/// Usage:
///   Enter an angle followed by its unit (deg or rad), e.g., "45 deg" or "1.57 rad".
///   Press CTRL-D to exit.
///
/// Example:
///   Input:  90 deg
///   Output: 90.00 deg is 1.57 rad.
#include "turtlelib/angle.hpp"
#include <iostream>
#include <string>
#include <limits>
#include <iomanip>

int main()
{
  while(true) {
        // Establish variables to store the user input:
    std::string datatype = "";
    auto data = double(0.0);

        // Prompt the user for input:
    std::cout << "Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)" << std::endl;
    std::cin >> data >> datatype;

        // Break loop if user inputs EOF character:
    if (std::cin.eof()) {break;}
        // Convert the datatype and normalize:
    if (datatype == "rad") {
            // Normalize the data:
      data = turtlelib::normalize_angle(data);
      auto conv_data = turtlelib::rad2deg(data);

            // Return the converted data to the user:
      std::cout << std::fixed << std::setprecision(2) << std::format("{} {} is {} deg.", data,
        datatype, conv_data) << std::endl;
    } else if (datatype == "deg") {
            // Convert then normalize:
      auto conv_data = turtlelib::deg2rad(data);
      conv_data = turtlelib::normalize_angle(conv_data);

            // Return the converted data to the user:
      std::cout << std::fixed << std::setprecision(2) << std::format("{} {} is {} rad.",
        turtlelib::rad2deg(conv_data), datatype, conv_data) << std::endl;
    }
        // ################################# Begin_Citation [2] ################################
        // Check to see if an invalid value was inputed for data
    else if (std::cin.fail()) {
            // Break Loop and Notify User of Invalid Input, continue Loop:
      std::cin.clear();       // clear error flags
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid input! ";
    }
        // ################################## End_Citation [2] #################################
        // Return invalid input if datatype isn't rad or deg:
    else {
      std::cout << "Invalid input! ";
    }
  }   // End of while (Program Termination)

  return 0;
}
