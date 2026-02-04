#include "turtlelib/se2d.hpp"

namespace turtlelib {
std::istream & operator>>(std::istream & is, Twist2D & tw)
{
        // First step is to determine the format:
  char first = is.peek();
        // Peek to see if the first character is '<'
  if (first == '<') {               // Format <omega[<unit>], x, y>
    is >> std::ws;                  // remove whitespace if it exists
    char ignore;
    is.get(ignore);                     // ignore '<'
    if (!(is >> tw.omega)) {                // try to read w, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is >> std::ws;                  // remove whitespace.
            // Determine if there are units:
    char unit = is.peek();
    if ((unit == 'r') || (unit == 'd')) {                  // input has units
                // Skip everything up to the next space
      is.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
    }
    is >> std::ws;                      // remove whitespace
    if (!(is >> tw.x)) {                // try to read x, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);
    if (ignore != ',') {
      is.setstate(std::ios::failbit);
      return is;
    }
    is >> std::ws;                  // remove whitespace
    if (!(is >> tw.y)) {                // try to read y, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);
    if (ignore != '>') {                // ignore '>'
      is.setstate(std::ios::failbit);
      return is;
    }
  } else if ((first >= '0' && first <= '9') || first == '-') {                 // Format -omega x y (allow neg omega)
            // Check that w, x and y are valid, otherwise mark them as fail.
    if (!(is >> tw.omega)) {                // try to read w, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is >> std::ws;              // remove whitespace
            // Determine if there are units:
    char unit = is.peek();
    if ((unit == 'r') || (unit == 'd')) {                  // input has units
                // Skip everything up to the next space
      is.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
    }
    is >> std::ws;              // remove whitespace
    if (!(is >> tw.x >> tw.y)) {                // try to read x and y, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
  } else {
            // Input begins invalid
    is.setstate(std::ios::failbit);
    return is;
  }

  return is;              // return the stream
}     // End of operator>> (Twist)

// Member Initialization Lists:
// REFER TO: https://en.cppreference.com/w/cpp/language/initializer_list.html
Transform2D::Transform2D()
: trans_{0.0, 0.0}, rot_{0.0} {}
Transform2D::Transform2D(Vector2D trans)
: trans_{trans}, rot_{0.0} {}
Transform2D::Transform2D(double radians)
: trans_{0.0, 0.0}, rot_{radians} {}
Transform2D::Transform2D(Vector2D trans, double radians)
: trans_{trans}, rot_{radians} {}

Point2D Transform2D::operator()(Point2D p) const
{
        // Establish Cos and Sin:
  double c = std::cos(rot_);
  double s = std::sin(rot_);
        // Multiply R_ab by p_b:
  double x_new = (c * p.x) - (s * p.y) + trans_.x;
  double y_new = (s * p.x) + (c * p.y) + trans_.y;
        // Return the new transformed position:
  return Point2D{x_new, y_new};
}     // End of operator() Point

Vector2D Transform2D::operator()(Vector2D v) const
{
        // Establish Cos and Sin:
  double c = std::cos(rot_);
  double s = std::sin(rot_);
        // Rotate the vector only: (Vector not affected by translation)
  double vx_new = (c * v.x) - (s * v.y);
  double vy_new = (s * v.x) + (c * v.y);
        // Return the new transformed position:
  return Vector2D{vx_new, vy_new};
}     // End of operator() Vector

Twist2D Transform2D::operator()(Twist2D v) const
{
  // Establish Cos and Sin:
  double c = std::cos(rot_);
  double s = std::sin(rot_);
  // Multiply Adj_ij by V_j:
  double vw_new = v.omega;
  double vx_new = (trans_.y * v.omega) + (c * v.x) - (s * v.y);
  double vy_new = (-1 * trans_.x * v.omega) + (s * v.x) + (c * v.y);
        // Return the Twist in the new frame:
  return Twist2D{vw_new, vx_new, vy_new};
}     // End of operator() Twist

Transform2D Transform2D::inv() const
{
        // Establish Cos and Sin:
  double c = std::cos(rot_);
  double s = std::sin(rot_);
        // Multiply R_ab inverse by trans_:
  double x_inv = (-1 * c * trans_.x) - (s * trans_.y);
  double y_inv = (s * trans_.x) - (c * trans_.y);
        // Return the Inverse Transform:
  return Transform2D{Vector2D{x_inv, y_inv}, -rot_};
}     // End of inv()

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
        // Establish Cos and Sin for lhs:
  double lhs_c = std::cos(rot_);
  double lhs_s = std::sin(rot_);
        // Establish Translation for rhs:
  double rhs_x = rhs.translation().x;
  double rhs_y = rhs.translation().y;

        // Rotate rhs trans by lhs rotation, then add to lhs translation:
  trans_.x += (lhs_c * rhs_x) - (lhs_s * rhs_y);
  trans_.y += (lhs_s * rhs_x) + (lhs_c * rhs_y);

        // Sum rhs and lhs rotations:
  rot_ += rhs.rotation();

  return *this;
}     //end of operator*=

// RETURN FUNCTIONS:
Vector2D Transform2D::translation() const
{
  return trans_;
}     // End of translation()

double Transform2D::rotation() const
{
  return rot_;
}     // End of rotation()

// TRANSFORM2D Operator>>:
std::istream & operator>>(std::istream & is, Transform2D & tf)
{
        // Create temporary transform to parse:
  Transform2D temptf;
  double temp_rot = 0.0;
  double temp_x = 0.0;
  double temp_y = 0.0;

        // First step is to determine the format:
  char first = is.peek();
        // Peek to see if the first character is '<'
  if (first == '<') {               // Format <omega[<unit>], x, y>
    is >> std::ws;                  // remove whitespace if it exists
    char ignore;
    is.get(ignore);                     // ignore '<'
    if (!(is >> temp_rot)) {                // try to rot, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is >> std::ws;                  // remove whitespace.
            // Determine if there are units:
    char unit = is.peek();
    if ((unit == 'r') || (unit == 'd')) {                  // input has units
                // Skip everything up to the next space
      is.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
    }
    is >> std::ws;                      // remove whitespace
    if (!(is >> temp_x)) {                // try to read x, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);
    if (ignore != ',') {
      is.setstate(std::ios::failbit);
      return is;
    }
    is >> std::ws;                  // remove whitespace
    if (!(is >> temp_y)) {                // try to read y, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is.get(ignore);
    if (ignore != '>') {                // ignore '>'
      is.setstate(std::ios::failbit);
      return is;
    }
  } else if ((first >= '0' && first <= '9') || first == '-') {                 // Format -omega x y (allow neg omega)
            // Check that w, x and y are valid, otherwise mark them as fail.
    if (!(is >> temp_rot)) {                // try to read rotation, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
    is >> std::ws;              // remove whitespace
            // Determine if there are units:
    char unit = is.peek();
    if ((unit == 'r') || (unit == 'd')) {                  // input has units
                // Skip everything up to the next space
      is.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
    }
    is >> std::ws;              // remove whitespace
    if (!(is >> temp_x >> temp_y)) {                // try to read translation x and y, fail if invalid.
      is.setstate(std::ios::failbit);
      return is;
    }
  } else {
            // Input begins invalid
    is.setstate(std::ios::failbit);
    return is;
  }

        // assign set the temp values to the tf:
  temptf = Transform2D(Vector2D{temp_x, temp_y}, temp_rot);
  tf = temptf;

  return is;              // return the stream
}     // End of Transform2D Operator>>

// Transform2D Operator
Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
        // Return the
  return lhs *= rhs;
}     // End of Transform2D Operator*

// Opearator* for Twist2D (scalar on rhs):
Twist2D operator*(const Twist2D& tw, double scalar) {
  Twist2D result;
  // Multiply each component by scalar:
  result.omega = tw.omega * scalar;
  result.x = tw.x * scalar;
  result.y = tw.y * scalar;
  return result;
} // end of operator*

// Opearator* for Twist2D (scalar on lhs):
Twist2D operator*(double scalar, const Twist2D& tw) {
  return tw * scalar;  // utilize the other operator*
} // end of operator*

// Opearator*= for Twist2D:
Twist2D & Twist2D::operator*=(double scalar) {
  // Multiply each component by scalar:
  omega *= scalar;
  x *= scalar;
  y *= scalar;
  return *this;
} // end of operator*=

// Integrate Twist Function:
Transform2D integrate_twist(Twist2D tw) {
  // Initialize Result Transform: (Assumed dt = 1)
  Transform2D result;
  double theta = tw.omega;
  double dx = tw.x;
  double dy = tw.y;

  if (std::abs(theta) < 1e-9) {
    // Case 1: Pure translation - Establish a Transform with only translation
    result = Transform2D(Vector2D{dx, dy});
  } else {
    // Case 2: With rotation
    // Calculate translational displacement due to rotation:
    // ##################################### Begin_Citation [7] ###########################
    double trans_x = (dx * std::sin(theta) + dy * (1 - std::cos(theta))) / theta;
    double trans_y = (dy * std::sin(theta) + dx * (std::cos(theta) - 1)) / theta;
    // ########################################## End_Citation [7] ####################################
    // Build resulting Transform2D
    result = Transform2D(Vector2D{trans_x, trans_y}, theta);
  }
  return result;
} // End of integrate_twist() 

} // end of namespace turtlelib