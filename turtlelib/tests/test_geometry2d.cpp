#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include <sstream>
#include <format>
#include <numbers>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

// POINT2D OPERATOR>>:
TEST_CASE("Point2D instream operator>>", "[operator>>]") {
    turtlelib::Point2D p;

    SECTION("Parses (x, y) format") {
        std::stringstream ss("(1.5, 2.5)");
        ss >> p;
        REQUIRE_THAT(p.x, WithinRel(1.5, EPS));
        REQUIRE_THAT(p.y, WithinRel(2.5, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses x y format") {
        std::stringstream ss("3.0 4.0");
        ss >> p;
        REQUIRE_THAT(p.x, WithinRel(3.0, EPS));
        REQUIRE_THAT(p.y, WithinRel(4.0, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Invalid input sets failbit") {
        std::stringstream ss("abc xyz");
        ss >> p;
        REQUIRE(ss.fail() == true); // reading failed
    }

    SECTION("Partially invalid input sets failbit") {
        std::stringstream ss("(5.0, abc)");
        ss >> p;
        REQUIRE(ss.fail() == true); //reading failed
    }

    SECTION("Fails if formated like a vector") {
        std::stringstream ss("[5.0 5.0]");
        ss >> p;
        REQUIRE(ss.fail() == true);     // reading failed
    }
}

// POINT2D Operator-:
TEST_CASE("Point2D operator-", "[operator-]") {
    turtlelib::Point2D p1;
    turtlelib::Point2D p2;

    SECTION("Correctly Returns Vector between two points") {
        std::stringstream ss1("(1.5, 2.5)");
        ss1 >> p1;
        std::stringstream ss2("(1.0, 2.0)");
        ss2 >> p2;
        // Use operator-:
        turtlelib::Vector2D v = p1 - p2;

        // Verify that the vector was generated successfully:
        REQUIRE_THAT(v.x, WithinRel(0.5, EPS));   // Vector x direction correct
        REQUIRE_THAT(v.y, WithinRel(0.5, EPS));   // Vector y direction correct
    }
}

// VECTOR2D OPERATOR+:
TEST_CASE("VECTOR2D operator+", "[operator+]") {
    turtlelib::Vector2D v;
    turtlelib::Point2D p;

    SECTION("Correctly Returns Point Translated by Vector") {
        std::stringstream ss1("(1.5, 2.5)");
        ss1 >> p;
        std::stringstream ss2("[1.0, 2.0]");
        ss2 >> v;
        // Use operator+:
        turtlelib::Point2D p_res = p + v;

        // Verify that the vector was generated successfully:
        REQUIRE_THAT(p_res.x, WithinRel(2.5, EPS));   // Point x direction correct
        REQUIRE_THAT(p_res.y, WithinRel(4.5, EPS));   // Point y direction correct
    }
}

// VECTOR2D Operator<<:
TEST_CASE("Vector2D ostream", "[operator<<]") {
    turtlelib::Vector2D v;
    std::ostringstream os;

    // Note: I made the decision to have my program keep 5 significant digits at printout:
    SECTION("Correctly outputs vector to outstream") {
        std::stringstream ss("[1.0, 2.0]");
        ss >> v;
        // Use operator>>:
        os << v;

        // Verify that the outstream was generated correctly::
        REQUIRE(os.str() == "[1.00000, 2.00000]");              // Output Correct
    }

    SECTION("Correctly outputs vector precision") {
        std::stringstream ss("[1.53435353, 2.23535353]");
        ss >> v;
        // Use operator>>:
        os << v;

        // Verify that the outstream was generated correctly::
        REQUIRE(os.str() == "[1.53435, 2.23535]");              // Precision Correct
    }
}

// VECTOR2D OPERATOR>>:
TEST_CASE("Vector2D instream operator>>", "[operator>>]") {
    turtlelib::Vector2D v;

    SECTION("Parses [x, y] format") {
        std::stringstream ss("[3.5, 1.5]");
        ss >> v;
        REQUIRE_THAT(v.x, WithinRel(3.5, EPS));
        REQUIRE_THAT(v.y, WithinRel(1.5, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses x y format") {
        std::stringstream ss("5.0 5.0");
        ss >> v;
        REQUIRE_THAT(v.x, WithinRel(5.0, EPS));
        REQUIRE_THAT(v.y, WithinRel(5.0, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Invalid input sets failbit") {
        std::stringstream ss("xyz 33z");
        ss >> v;
        REQUIRE(ss.fail() == true); // reading failed
    }

    SECTION("Partially invalid input sets failbit") {
        std::stringstream ss("5.0 abc");
        ss >> v;
        REQUIRE(ss.fail() == true);     // reading failed
    }

    SECTION("Fails if formated like a point") {
        std::stringstream ss("(5.0 5.0)");
        ss >> v;
        REQUIRE(ss.fail() == true);     // reading failed
    }
}

// VECTOR2D NORMALIZE:
TEST_CASE("VECTOR2D normalize", "[normalize]") {
    turtlelib::Vector2D v;

    SECTION("Correctly Normalizes Non-zero vector") {
        std::stringstream ss("[3.0, 3.0]");
        ss >> v;

        // Use normalize:
        turtlelib::Vector2D v_norm = normalize(v);

        // Verify that the vector was generated successfully:
        REQUIRE_THAT(v_norm.x, WithinRel(0.7071, EPS));   // norm x direction correct
        REQUIRE_THAT(v_norm.y, WithinRel(0.7071, EPS));   // norm y direction correct
        REQUIRE(ss.fail() == false);
    }

    SECTION("Correctly Returns Fail for Zero Vector") {
        std::stringstream ss("[0.0, 0.0]");
        ss >> v;

        // Verify that the vector was failed:
        REQUIRE_THROWS_AS(
            // Use normalize:
            normalize(v),
            std::invalid_argument       // exception invalid_arguement
        );
        REQUIRE(ss.fail() == false);
    }
}

// VECTOR2D Addition:
TEST_CASE("Vector2D Addition Tests", "operator+ and operator+=") {
    turtlelib::Vector2D v1{1.0, 2.0};
    turtlelib::Vector2D v2{3.5, -1.0};

    // Test operator+=
    v1 += v2;                   // V1 is now {4.5, 1.0}
    // Test operator+
    turtlelib::Vector2D result2 = v1 + v2;          // Result2 is now {8.0, 0.0}

    SECTION("Components are added correctly for operator+=") {
        REQUIRE_THAT(v1.x, WithinRel(4.5, EPS));
        REQUIRE_THAT(v1.y, WithinRel(1.0, EPS));
    }

    SECTION("Components are added correctly for operator+") {
        REQUIRE_THAT(result2.x, WithinRel(8.0, EPS));
        REQUIRE_THAT(result2.y, WithinRel(0.0, EPS));
    }
}

// VECTOR2D Subtraction:
TEST_CASE("Vector2D Subtraction Tests", "operator- and operator-=") {
    turtlelib::Vector2D v1{1.0, 2.0};
    turtlelib::Vector2D v2{3.5, -1.0};

    // Test operator-=
    v1 -= v2;               // V1 is now {-2.5, 3.0}
    // Test operator-
    turtlelib::Vector2D result2 = v1 - v2;  // Result2 is now {-6.0, 4.0}

    SECTION("Components are subtracted correctly for operator-=") {
        REQUIRE_THAT(v1.x, WithinRel(-2.5, EPS));
        REQUIRE_THAT(v1.y, WithinRel(3.0, EPS));
    }

    SECTION("Components are subtracted correctly for operator+") {
        REQUIRE_THAT(result2.x, WithinRel(-6.0, EPS));
        REQUIRE_THAT(result2.y, WithinRel(4.0, EPS));
    }
}

// VECTOR2D Multiplication by Scalar:
TEST_CASE("Vector2D Multiplication by Scalar Tests", "operator* and operator*=") {
    turtlelib::Vector2D v1{2.0, -3.0};
    turtlelib::Vector2D v2{2.0, -3.0};
    double scalar = 4.0;
    // Test operator*=
    v1 *= scalar;           // V1 is now {8.0, -12.0}
    // Test operator* scalar on rhs
    turtlelib::Vector2D result1 = v2 * scalar;  // Result1 is now {8.0, -12.0}
    // Test operator* scalar on lhs
    turtlelib::Vector2D result2 = scalar * v2;  // Result2 is now {8.0, -12.0}

    SECTION("Components are multiplied correctly for operator*=") {
        REQUIRE_THAT(v1.x, WithinRel(8.0, EPS));
        REQUIRE_THAT(v1.y, WithinRel(-12.0, EPS));
    }

    SECTION("Components are multiplied correctly for operator+ with scalar on rhs") {
        REQUIRE_THAT(result1.x, WithinRel(8.0, EPS));
        REQUIRE_THAT(result1.y, WithinRel(-12.0, EPS));
    }

    SECTION("Components are multiplied correctly for operator+ with scalar on lhs") {
        REQUIRE_THAT(result2.x, WithinRel(8.0, EPS));
        REQUIRE_THAT(result2.y, WithinRel(-12.0, EPS));
    }
}

// VECTOR2D Dot Product:
TEST_CASE("Vector2D Dot Product Tests", "dot()") {
    turtlelib::Vector2D v1{2.0, 3.0};
    turtlelib::Vector2D v2{-4.0, -4.0};
    // Test dot()
    double result1 = dot(v1, v1);  // Result is now -20.0
    double result2 = dot(v1, v2);  // Result is now 13.0
    
    SECTION("Dot product with itself is computed correctly") {
        REQUIRE_THAT(result1, WithinRel(13.0, EPS));
    }
    SECTION("Dot product is computed correctly between two vectors") {
        REQUIRE_THAT(result2, WithinRel(-20.0, EPS));
    }
}

// VECTOR2D Magnitude:
TEST_CASE("Vector2D Magnitude Tests", "magnitude()") {
    turtlelib::Vector2D v1{3.0, 4.0};
    turtlelib::Vector2D v2{-1.0, -1.0};
    turtlelib::Vector2D v3{0.0, 0.0};
    // Test magnitude()
    double result1 = magnitude(v1);  // Result is now 5.0
    double result2 = magnitude(v2);  // Result is now sqrt(2)
    double result3 = magnitude(v3);  // Result is now 0.0
    
    SECTION("Magnitude of vector with positive components is computed correctly") {
        REQUIRE_THAT(result1, WithinRel(5.0, EPS));
    }
        
    SECTION("Magnitude of vector with negative components is computed correctly") {
        REQUIRE_THAT(result2, WithinRel(sqrt(2.0), EPS));
    }

    SECTION("Magnitude of zero vector is computed correctly") {
        REQUIRE_THAT(result3, WithinRel(0.0, EPS));
    }
}

// VECTOR2D Angles:
TEST_CASE("Vector2D Angle Tests", "angle()") {
    turtlelib::Vector2D v1{2.0, 2.0};
    turtlelib::Vector2D v2{-1.0, -1.0};
    turtlelib::Vector2D v3{3.0, 2.0};
    turtlelib::Vector2D v4{0.0, 0.0};
    // Test angle()
    double result1 = angle(v1, v2);  // Result is now pi
    double result2 = angle(v2, v3);  // Result is now ~2.944
    
    SECTION("Angle of vector with positive components is computed correctly") {
        REQUIRE_THAT(result1, WithinRel(std::numbers::pi, EPS));
    }
        
    SECTION("Angle of vector with negative components is computed correctly") {
        REQUIRE_THAT(result2, WithinRel(2.944, EPS));
    }

    SECTION("Angle of zero magnitude vector is computed correctly") {
        REQUIRE_THROWS_AS(angle(v1, v4), std::invalid_argument);
    }
}

TEST_CASE("Point2D formatting Tests", "turtlelib::Point2D p{x, y}") {
    SECTION("Standard Formating Test") {
        turtlelib::Point2D p{1.5, -2.3};
        std::string s = std::format("{}", p);
        REQUIRE(s == "(1.50000, -2.30000)");
    }

    SECTION("Point2D - Origin") {
        turtlelib::Point2D p{0.0, 0.0};
        REQUIRE(std::format("{}", p) == "(0.00000, 0.00000)");      // Allow formater to remove trailing zeros.
    }
}

TEST_CASE("Vector2D formatting Tests", "turtlelib::Vector2D v{x, y}") {
    SECTION("Standard Formating Test") {
        turtlelib::Vector2D v{1.5, -2.3};
        std::string s = std::format("{}", v);
        REQUIRE(s == "[1.50000, -2.30000]");
    }

    SECTION("Vector2D - Zero Vector") {
        turtlelib::Vector2D v{0.0, 0.0};
        REQUIRE(std::format("{}", v) == "[0.00000, 0.00000]");      // Allow formater to remove trailing zeros.
    }
}
