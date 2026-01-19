#include "turtlelib/se2d.hpp"

namespace turtlelib {
    std::istream & operator>>(std::istream & is, Twist2D & tw) {
        // First step is to determine the format:
        char first = is.peek();
        std::string unit_type;
        // Peek to see if the first character is '<'
        if (first == '<') {         // Format <omega[<unit>], x, y>
            char ignore; 
            is.get(ignore);             // ignore '<'
            if (!(is >> tw.omega)) {        // try to read w, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
            // Determine if there are units:
            char unit = is.peek();
            if ((unit == 'r') || (unit == 'd')) {          // input has units
                is >> unit_type;                      // parse the unit type
            }
            is.get(ignore);
            if (ignore != ',') {
                is.setstate(std::ios::failbit);
                return is;
            }
            if (!(is >> tw.x)) {        // try to read x, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
            is.get(ignore);
            if (ignore != ',') {
                is.setstate(std::ios::failbit);
                return is;
            }
            if (!(is >> tw.y)) {        // try to read y, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
            is.get(ignore);
            if (ignore != '>') {        // ignore '>'
                is.setstate(std::ios::failbit);
                return is;
            }
        } 
        else if ((first >= '0' && first <= '9') || first == '-') {             // Format -omegax y (alloomeganeg w)
            // Check that w, x and y are valid, otherwise mark them as fail.
            if (!(is >> tw.omega)) {        // try to read w, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
            // Determine if there are units:
            char unit = is.peek();
            if ((unit == 'r') | (unit == 'd')) {          // input has units
                is >> unit_type;                      // parse the unit type
            }
            if (!(is >> tw.x >> tw.y)) {        // try to read x and y, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
        }
        else {
            // Input begins invalid
            is.setstate(std::ios::failbit);
            return is;
        }

        return is;        // return the stream
    } // End of operator>> (Twist)
}