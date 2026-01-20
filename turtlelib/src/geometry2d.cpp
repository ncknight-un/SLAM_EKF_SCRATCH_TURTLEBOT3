/// \file geometry2d.hpp
/// \brief 2D points and vectors with basic arithmetic and stream operators
/// \namespace turtlelib

#include "turtlelib/geometry2d.hpp"

namespace turtlelib {
/// \brief Reads a Point2D from a stream
///
/// Accepts formats: "(x, y)" or "x y"
///
/// \param is Input stream
/// \param p Point2D object to populate
/// \returns Reference to the input stream
std::istream & operator>>(std::istream & is, Point2D & p)
{
        // First step is to determine the format:
  char first = is.peek();
        // Peek to see if the first character is '('
  if (first == '(') {               // Format (x, y)
    char ignore;
    is.get(ignore);                     // ignore '('
    if (!(is >> p.x)) {                // try to read x, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);
    if (ignore != ',') {
      is.setstate(std::ios::failbit);
      return is;
    }
    if (!(is >> p.y)) {                // try to read y, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);
    if (ignore != ')') {
      is.setstate(std::ios::failbit);
      return is;
    }
  } else if ((first >= '0' && first <= '9') || first == '-') {                 // Format x y (allow neg x)
            // Check that x and y are valid, otherwise mark them as fail.
    if (!(is >> p.x >> p.y)) {
      is.setstate(std::ios::failbit);
      return is;
    }
  } else {
            // Input begins invalid
    is.setstate(std::ios::failbit);
    return is;
  }

  return is;              // return the stream
}     // End of operator>> (Point)

/// \brief Subtracts two Point2D objects to produce a Vector2D
///
/// Computes the vector from tail to head
///
/// \param head Endpoint Point2D
/// \param tail Start Point2D
/// \returns Vector2D representing the displacement from tail to head
Vector2D operator-(const Point2D & head, const Point2D & tail)
{
        // Return a 2D vector as the difference from head to tail:
  return Vector2D{head.x - tail.x, head.y - tail.y};
}     // End of operator-

/// \brief Adds a Vector2D to a Point2D to produce a new Point2D
///
/// Moves the point by the vector displacement
///
/// \param tail Base Point2D
/// \param disp Vector2D displacement
/// \returns New Point2D after translation
Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
        // Return a 2DPoint that is moved by a vector:
  return Point2D{tail.x + disp.x, tail.y + disp.y};
}     // End of operator+

/// \brief Writes a Vector2D to an output stream
///
/// Always prints with 5 decimal places in the format: "[x, y]"
///
/// \param os Output stream
/// \param v Vector2D to print
/// \returns Reference to the output strea
std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
        // I decided to force the printout to always keep a precision of 5 decimal places:
  os << "[" << std::fixed << std::setprecision(5) << v.x << ", " << std::fixed <<
    std::setprecision(5) << v.y << "]";
  return os;
}     // End of operator<<

/// \brief Reads a Vector2D from an input stream
///
/// Accepts formats: "[x, y]" or "x y"
///
/// \param is Input stream
/// \param v Vector2D object to populate
/// \returns Reference to the input stream
std::istream & operator>>(std::istream & is, Vector2D & v)
{
        // First step is to determine the format:
  char first = is.peek();
        // Peek to see if the first character is '['
  if (first == '[') {               // Format [x, y]
    char ignore;
    is.get(ignore);                     // ignore '['
    if (!(is >> v.x)) {                // try to read x, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);                     // ignore ','
    if (ignore != ',') {
      is.setstate(std::ios::failbit);
      return is;
    }
    if (!(is >> v.y)) {                // try to read y, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);              // ignore ']'
    if (ignore != ']') {
      is.setstate(std::ios::failbit);
      return is;
    }
  } else if ((first >= '0' && first <= '9') || first == '-') {                 // Format x y (allow neg x)
            // Check that x and y are valid, otherwise mark them as fail.
    if (!(is >> v.x >> v.y)) {
      is.setstate(std::ios::failbit);
      return is;
    }
  } else {
            // Input begins invalid:
    is.setstate(std::ios::failbit);
    return is;
  }

  return is;              // return the stream
}     // End of operator>> (Vector)

/// \brief Normalizes a Vector2D
///
/// Throws an exception if the vector is zero
///
/// \param in Vector2D to normalize
/// \returns Normalized Vector2D with magnitude 1
/// \throws std::invalid_argument if input vector is zero
Vector2D normalize(Vector2D in)
{
        // Throw an exception if it is the zero vector:
        // ################################ Begin_Citation [3] ################################
  if (in.x == 0 && in.y == 0) {throw std::invalid_argument("Zero vector - Error");}
        // ################################ End_Citation [3] ##################################

        // Normalize the vector:
  auto norm = std::sqrt(in.x * in.x + in.y * in.y);
  return Vector2D(in.x / norm, in.y / norm);
}     // End of normalize
}
