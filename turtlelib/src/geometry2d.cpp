#include "turtlelib/geometry2d.hpp"

namespace turtlelib {
std::istream & operator>>(std::istream & is, Point2D & p) {
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

Vector2D operator-(const Point2D & head, const Point2D & tail) {
        // Return a 2D vector as the difference from head to tail:
  return Vector2D{head.x - tail.x, head.y - tail.y};
}     // End of operator-

Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        // Return a 2DPoint that is moved by a vector:
  return Point2D{tail.x + disp.x, tail.y + disp.y};
}     // End of operator+

std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        // I decided to force the printout to always keep a precision of 5 decimal places:
  os << "[" << std::fixed << std::setprecision(5) << v.x << ", " << std::fixed <<
    std::setprecision(5) << v.y << "]";
  return os;
}     // End of operator<<

std::istream & operator>>(std::istream & is, Vector2D & v) {
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

Vector2D normalize(Vector2D in) {
        // Throw an exception if it is the zero vector:
        // ################################ Begin_Citation [3] ################################
  if (in.x == 0 && in.y == 0) {throw std::invalid_argument("Zero vector - Error");}
        // ################################ End_Citation [3] ##################################

  auto norm = std::sqrt(in.x * in.x + in.y * in.y);
  return Vector2D(in.x / norm, in.y / norm);
}     // End of normalize

Vector2D & Vector2D::operator+=(const Vector2D & v) {
  x += v.x;
  y += v.y;
  return *this;
} // End of operator+= for Vector2D

  Vector2D & Vector2D::operator-=(const Vector2D & v) {
  x -= v.x;
  y -= v.y;
  return *this;
} // End of operator-= for Vector2D

Vector2D operator+(const Vector2D& lhs, const Vector2D& rhs) {
  Vector2D result = lhs;
  result += rhs;
  return result;
} // End of operator+ for Vector2D

Vector2D operator-(const Vector2D& lhs, const Vector2D& rhs) {
  Vector2D result = lhs;
  result -= rhs;
  return result;
} // End of operator- for Vector2D

Vector2D operator*(double scalar, const Vector2D & v) {
  Vector2D result;
  result.x = v.x * scalar;
  result.y = v.y * scalar;
  return result;
} // End of operator* for scalar on lhs

Vector2D operator*(const Vector2D & v, double scalar) {
  Vector2D result;
  result.x = v.x * scalar;
  result.y = v.y * scalar;
  return result;
} // End of operator* for scalar on rhs

Vector2D & Vector2D::operator*=(double scalar) {
  this->x *= scalar;
  this->y *= scalar;
  return *this;
} // End of operator*= for Vector2D

double dot(const Vector2D & v1, const Vector2D & v2){
  return v1.x * v2.x + v1.y * v2.y;
} // End of dot for Vector2D

double magnitude(const Vector2D & v){
  return std::sqrt(v.x * v.x + v.y * v.y);
} // End of magnitude for Vector2D

double angle(const Vector2D & v1, const Vector2D & v2){
  // Dot product divided by magnitudes, then acos of result
  double mag1 = magnitude(v1);
  double mag2 = magnitude(v2);
  if (mag1 == 0.0 || mag2 == 0.0) { throw std::invalid_argument("Zero Magnitude - Error");} // Avoids division by zero

  double dot_product = dot(v1, v2);
  double magnitudes = magnitude(v1) * magnitude(v2);
  double angle = std::acos(dot_product / magnitudes);
  return angle;
} // End of angle for Vector2D

} // End ofnamespace turtlelib
