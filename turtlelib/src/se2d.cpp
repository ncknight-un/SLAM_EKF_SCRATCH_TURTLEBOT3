#include "turtlelib/se2d.hpp"

std::istream & operator>>(std::istream & is, Twist2D & tw) {
    // First step is to determine the format:
    char first = is.peek();
    // Peek to see if the first character is '('
    if (first == '<') {         // Format <w [<unit>], x, y>
        char ignore; 
        is.get(ignore);             // ignore '('
        if (!(is >> tw.w)) {        // try to read w, fail if invalid.
            is.setstate(std::ios::failbit);
            return is;
        }
        is.get(ignore);
        if (ignore != ',') {
            is.setstate(std::ios::failbit);
            return is;
        }
        if (!(is >> p.y)) {        // try to read y, fail if invalid.
            is.setstate(std::ios::failbit);
            return is;
        }
        is.get(ignore);
        if (ignore != ')') {
            is.setstate(std::ios::failbit);
            return is;
        }
    } 
    else if ((first >= '0' && first <= '9') || first == '-') {             // Format x y (allow neg x)
        // Check that x and y are valid, otherwise mark them as fail.
        if (!(is >> p.x >> p.y)) {
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