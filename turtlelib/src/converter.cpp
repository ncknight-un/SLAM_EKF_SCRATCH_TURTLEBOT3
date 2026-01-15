#include "turtlelib/angle.hpp"
#include <iostream>
#include <cstdio>
#include <cstring>

int main() {
    // Establish variables to store the user input:
    char datatype[4];
    auto data = double(0.0);

    while(true) {
        // Prompt the user for input:
        printf("Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)\n");
        scanf("%lf %s", &data, datatype);

        // Convert the datatype and normalize: 
        if (strcmp(datatype, "rad") == 0){              // Returns 0 if the strings are identical
            // Normalize the data: 
            data = turtlelib::normalize_angle(data);
            auto conv_data = turtlelib::rad2deg(data);

            // Return the converted data to the user:
            printf("%lf %2s is %lf deg.\n", data, datatype, conv_data);
        }
        else if (strcmp(datatype, "deg") == 0){
            // Convert then normalize:
            auto conv_data = turtlelib::deg2rad(data);
            conv_data = turtlelib::normalize_angle(conv_data);

            // Return the converted data to the user:
            printf("%lf %2s is %lf rad.\n", data, datatype, conv_data);
        }
        else {
            // Break Loop and Notify User of Invalid Input, continue Loop: 
            printf("Invalid input: ");
        }
    } // End of while (Program Termination)
        
    return 0;
}

// Make sure that the loop terminates correctly. 
// Fix the use case for invalid data.