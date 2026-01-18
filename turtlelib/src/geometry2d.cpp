#include "turtlelib/geometry2d.hpp"

namespace turtlelib {
    std::istream & operator>>(std::istream & is, Point2D & p){
        // First step is to determine the format:
        char ignore;
        // Peek to see if the first character is '('
        if (is.peek() == '(') {         // Format (x, y)
            is.get(ignore);             // ignore '('
            if (!(is >> p.x)) {        // try to read x, fail if invalid.
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
        else {                          // Format x y
            // Check that x and y are valid, otherwise mark them as fail.
            if (!(is >> p.x >> p.y)) {
                is.setstate(std::ios::failbit);
                return is;
            }
        }

        return is;        // return the stream
    } // End of operator>> (Point)

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        // Return a 2D vector as the difference from head to tail:
        return Vector2D{head.x - tail.x, head.y - tail.y};
    } // End of operator-

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        // Return a 2DPoint that is moved by a vector:
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    } // End of operator+

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << "[" << v.x << "," << v.y << "]";
        return os;
    } // End of operator<<

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        // First step is to determine the format:
        char ignore;
        // Peek to see if the first character is '['
        if (is.peek() == '[') {         // Format [x, y]
            is.get(ignore);             // ignore '['
            if (!(is >> v.x)) {        // try to read x, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
            is.get(ignore);             // ignore ','
            if (!(is >> v.y)) {        // try to read y, fail if invalid.
                is.setstate(std::ios::failbit);
                return is;
            }
            is.get(ignore);      // ignore ']'
        } 
        else {                       // Format x y
            // Check that x and y are valid, otherwise mark them as fail.
            if (!(is >> v.x >> v.y)) {
                is.setstate(std::ios::failbit);
                return is;
            }
        }
        
        return is;        // return the stream
    } // End of operator>> (Vector)

    Vector2D normalize(Vector2D in) {
        // Throw an exception if it is the zero vector:
        // ################################ Begin_Citation [3] ################################
        if (in.x == 0 && in.y == 0) {throw std::invalid_argument("Zero vector - Error");}
        // ################################ End_Citation [3] ##################################

        // Normalize the vector:
            auto norm = std::sqrt(in.x * in.x + in.y * in.y);
            return Vector2D(in.x/norm, in.y/norm);
    } // End of normalize
}