#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives and other mathematical objects

// Additional include files:
#include "turtlelib/angle.hpp"
#include <sstream>
#include <format>
#include <stdexcept>
#include <cmath>
#include <iomanip>
// Note: <iosfwd> contains forward definitions for iostream objects allowing implementation of
// custom iostream operators without requiring the inclusion of <iostream>, which is a big header file.
#include <iosfwd>

namespace turtlelib
{
    /// \brief a 2-Dimensional Point
struct Point2D
{
        /// \brief the x coordinate
  double x = 0.0;

        /// \brief the y coordinate
  double y = 0.0;
};

    /// \brief Input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   "(x, y)" or "x y"  (Not including the "").
    /// \param is An istream from which to read
    /// \param p [out] The Point2D object that will store the input
    /// \returns A reference to is. An error flag is set on the stream if the input cannot be parsed.
std::istream & operator>>(std::istream & is, Point2D & p);


    /// \brief A 2-Dimensional Vector
struct Vector2D
{
        /// \brief the x coordinate
  double x = 0.0;

        /// \brief the y coordinate
  double y = 0.0;
};

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this operator is not implemented in terms of -=
    /// because subtracting two Point2D yields a Vector2D not a Point2D
Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
Point2D operator+(const Point2D & tail, const Vector2D & disp);


    /// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   "[x, y]" or "x y" (not including the "")
    /// \param is An istream from which to read
    /// \param v [out] - output vector
    /// \returns a reference to the istream, with any error flags set if
    /// a parsing error occurs
std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief Return a unit vector in the direction of v
    /// \param in The vector to normalize
    /// \return The normalized vector.
    /// \throws std::invalid_input if it is the zero vector
Vector2D normalize(Vector2D in);
}

/// \brief A Formatter for 2D points
/// The output is "(x, y)"
/// All floating-point format specifiers are honored and applied to both x and y.
template<class CharT>
struct std::formatter<turtlelib::Point2D, CharT>
{
  std::formatter<float, CharT> float_formatter;       // Member variable for float

        /// \brief Parse the format-specifier, storing the results in *this so that
        ///   they can be used for controlling how MyType is formatted
        /// \param parse_ctx An std::basic_format_parse_Context<CharT>
        ///   This contains .begin() and .end() iterators for all characters after
        ///   the : in the format-spec (including the "}").  If the
        ///   format-spec is empty (e.g., "{}") then .begin() == .end()
        /// \returns std::basic_format_parse_context<CharT>::iterator
        /// The iterator points to the character that is past the end of the last character parsed by the parse function
        /// If re-using a parser via inheritance, do not include this function here.
  constexpr auto parse(auto & parse_ctx)
  {
    return float_formatter.parse(parse_ctx);
  }

        /// \brief Writes a string representation of t to the fmt_ctx range
        /// \param t The type to output.
        ///    (Note: this can also be taken by value instead of const &, if desired).
        /// \param fmt_ctx An std::basic_format_context<LegacyOutputIterator, CharT>
        ///    Contains an iterator to output characters to. There is no guarantee
        ///    of what type LegacyOutputIterator is, so your code should not
        ///    depend it being a particular iterator type
        /// \returns std::basic_format_context<>::iterator. The iterator should
        ///   point to one past the last output character (e.g., where the next
        ///   character from whatever else is being added to the string should be inserted)
  auto format(const turtlelib::Point2D & p, auto & fmt_ctx) const
  {
    return std::format_to(fmt_ctx.out(), "({:.5f}, {:.5f})", p.x, p.y);
  }
};

/// \brief A formatter for Vector2D
/// All double format-spec specifiers apply to each number in the vector
/// The vector is output as [x, y]
template<class CharT>
struct std::formatter<turtlelib::Vector2D, CharT>
{
  std::formatter<float, CharT> float_formatter;       // Member variable for float

        /// \brief Parse the format-specifier, storing the results in *this so that
        ///   they can be used for controlling how MyType is formatted
        /// \param parse_ctx An std::basic_format_parse_Context<CharT>
        ///   This contains .begin() and .end() iterators for all characters after
        ///   the : in the format-spec (including the "}").  If the
        ///   format-spec is empty (e.g., "{}") then .begin() == .end()
        /// \returns std::basic_format_parse_context<CharT>::iterator
        /// The iterator points to the character that is past the end of the last character parsed by the parse function
        /// If re-using a parser via inheritance, do not include this function here.
  constexpr auto parse(auto & parse_ctx)
  {
    return float_formatter.parse(parse_ctx);
  }

        /// \brief Writes a string representation of t to the fmt_ctx range
        /// \param t The type to output.
        ///    (Note: this can also be taken by value instead of const &, if desired).
        /// \param fmt_ctx An std::basic_format_context<LegacyOutputIterator, CharT>
        ///    Contains an iterator to output characters to. There is no guarantee
        ///    of what type LegacyOutputIterator is, so your code should not
        ///    depend it being a particular iterator type
        /// \returns std::basic_format_context<>::iterator. The iterator should
        ///   point to one past the last output character (e.g., where the next
        ///   character from whatever else is being added to the string should be inserted)
  auto format(const turtlelib::Vector2D & v, auto & fmt_ctx) const
  {
    return std::format_to(fmt_ctx.out(), "[{:.5f}, {:.5f}]", v.x, v.y);                 // Returns [x, y]
  }
};
#endif
