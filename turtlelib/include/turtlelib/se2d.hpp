#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include<iosfwd>
#include "turtlelib/geometry2d.hpp"
#include <format>
#include <limits>
#include <sstream>

namespace turtlelib
{

    /// \brief represent a 2-Dimensional twist
    struct Twist2D {
        /// \brief the angular velocity
        double omega = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the linear y velocity
        double y = 0.0;
    };

    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
    /// The "" are not part of the input.
    /// The [<unit>] is optional and can be any string without spaces that starts with an r
    /// (for rad/s) and any string without spaces that starts with a d for deg/s)
    /// If the unit is omitted, assume rad/s.
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D {
        public:
            /// \brief Create an identity transformation
            Transform2D();

            /// \brief create a transformation that is a pure translation
            /// \param trans - the vector by which to translate
            explicit Transform2D(Vector2D trans);

            /// \brief create a pure rotation
            /// \param radians - angle of the rotation, in radians
            explicit Transform2D(double radians);

            /// \brief Create a transformation with a translational and rotational
            /// component
            /// \param trans - the translation
            /// \param radians - the rotation, in radians
            Transform2D(Vector2D trans, double radians);

            /// \brief apply a transformation to a 2D Point
            /// \param p the point to transform
            /// \return a point in the new coordinate system
            Point2D operator()(Point2D p) const;

            /// \brief apply a transformation to a 2D Vector
            /// \param v - the vector to transform
            /// \return a vector in the new coordinate system
            Vector2D operator()(Vector2D v) const;

            /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
            /// \param v - the twist to transform
            /// \return a twist in the new coordinate system
            Twist2D operator()(Twist2D v) const;

            /// \brief invert the transformation
            /// \return the inverse transformation.
            Transform2D inv() const;

            /// \brief compose this transform with another and store the result
            /// in this object
            /// \param rhs - the first transform to apply
            /// \return a reference to the newly transformed operator
            Transform2D & operator*=(const Transform2D & rhs);

            /// \brief the translational component of the transform
            /// \return the x,y translation
            Vector2D translation() const;

            /// \brief get the angular displacement of the transform
            /// \return the angular displacement, in radians
            double rotation() const;

            /// \brief see std::formatter for the Transform2D
            // Primary friend template access:
            // ################################## Begin_Citation [4] #########################
            // The source discusses inheritance of struct for friends. A class can't create a subclass inside itself, so we must use the 
            // primary representation of the formater as a friend that our custom formatter will inherit.
            template<class CharT>
            friend struct std::formatter;
            // ################################## End_Citation [4] ############################
            
        private:
        // Definition of private variables ot hold transofrm translation and rotation:
        Vector2D trans_;
        double rot_;
    };


    /// \brief Read a transformation from stdin
    /// Should be able to read input either as:
    ///  "theta [<unit>] dx dy" (i.e., three numbers separated by whitespace, angle assumed to be radians)
    //   "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  [<unit>] is optional and can be any string without spaces that starts with a d for deg or r for rad
    ///  If [<unit>] is omitted, assume the unit is radians
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

}

// ########################################### Begin_Citation [6] ###########################
/// \brief A formatter for Transform2D
/// Creates a string representation of a Transform2D
/// as "{<angle> [<unit>], <x> <y>}"
/// An R at the beginning of the format-spec makes [<unit>] rad
/// A D  at the beginning of the format-spec makes [<unit>] deg
/// No R or D means no unit is printed but the angle is in radians.
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers that
/// are put into the string
template<class CharT>
struct std::formatter<turtlelib::Transform2D, CharT>
{

};

/// \brief print the Twist2D as "<w [<unit>], x, y>"
/// An R at the beginning of the format-spec makes [<unit>] rad/s
/// A  D at the beginning of the format-spec makes [<unit>] deg/s
/// No R or D means no unit is printed but the w is taken to be in rad/s
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers inserted
/// into the string.
template<class CharT>
struct std::formatter<turtlelib::Twist2D, CharT>
{
    std::formatter<float, CharT> float_formatter; // Member variable for float
    // Parser Unit Identifiers:
    bool isDeg_ = false;

    /// \brief Parse the format-specifier, storing the results in *this so that
    ///   they can be used for controlling how MyType is formatted
    /// \param parse_ctx An std::basic_format_parse_Context<CharT>
    ///   This contains .begin() and .end() iterators for all characters after
    ///   the : in the format-spec (including the "}").  If the
    ///   format-spec is empty (e.g., "{}") then .begin() == .end()
    /// \returns std::basic_format_parse_context<CharT>::iterator
    /// The iterator points to the character that is past the end of the last character parsed by the parse function
    /// If re-using a parser via inheritance, do not include this function here.
    constexpr auto parse(auto & parse_ctx) {
        // Case 3: Custom Formating: Refer to https://nu-msr.github.io/navigation/lectures/cpp/custom_formatters.html
        auto i = parse_ctx.begin();
        // Iterate over parse_ctx and determine if R or D or None:
        while (i != parse_ctx.end() && *i != '}') {
            if (*i == 'D' || *i == 'd') {
                // Set unit for formater:
                isDeg_ = true;
            }
            ++i;
        }
        return i;
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
    auto format(const turtlelib::Twist2D & tw, auto & fmt_ctx) const {
        if (isDeg_) {
            return std::format_to(fmt_ctx.out(), "<{:.5f} deg/s, {:.5f}, {:.5f}>", tw.omega, tw.x, tw.y);
        }
        else {
            return std::format_to(fmt_ctx.out(), "<{:.5f} rad/s, {:.5f}, {:.5f}>", tw.omega, tw.x, tw.y);
        }
    }
    // ########################################## End_Citation [6] #############################
};
#endif
