#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include <sstream>
#include <format>

using Catch::Matchers::WithinRel;
constexpr double EPS =  0.001;

// POINT2D OPERATOR>>:
TEST_CASE("Point2D instream operator>>", "[operator>>]") {
    turtlelib::Point2D p;

    SECTION("Parses (x, y) format") {
        std::stringstream ss("(1.5, 2.5)");
        ss >> p;
        REQUIRE_THAT(p.x, WithinRel(1.5,  EPS));
        REQUIRE_THAT(p.y, WithinRel(2.5,  EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses x y format") {
        std::stringstream ss("3.0 4.0");
        ss >> p;
        REQUIRE_THAT(p.x, WithinRel(3.0,  EPS));
        REQUIRE_THAT(p.y, WithinRel(4.0,  EPS));
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
        REQUIRE_THAT(v.x, WithinRel(0.5,  EPS));  // Vector x direction correct
        REQUIRE_THAT(v.y, WithinRel(0.5,  EPS));  // Vector y direction correct
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
        REQUIRE_THAT(p_res.x, WithinRel(2.5,  EPS));  // Point x direction correct
        REQUIRE_THAT(p_res.y, WithinRel(4.5,  EPS));  // Point y direction correct
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
        REQUIRE_THAT(v.x, WithinRel(3.5,  EPS));
        REQUIRE_THAT(v.y, WithinRel(1.5,  EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses x y format") {
        std::stringstream ss("5.0 5.0");
        ss >> v;
        REQUIRE_THAT(v.x, WithinRel(5.0,  EPS));
        REQUIRE_THAT(v.y, WithinRel(5.0,  EPS));
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
        REQUIRE_THAT(v_norm.x, WithinRel(0.7071,  EPS));  // norm x direction correct
        REQUIRE_THAT(v_norm.y, WithinRel(0.7071,  EPS));  // norm y direction correct
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

TEST_CASE("Point2D formatting Tests", "turtlelib::Point2D p{x, y}") {
    SECTION("Standard Formating Test"){
        turtlelib::Point2D p{1.5, -2.3};
        std::string s = std::format("{}", p);
        REQUIRE(s == "(1.5, -2.3)");
    }

    SECTION("Point2D - Origin") {
        turtlelib::Point2D p{0.0, 0.0};
        REQUIRE(std::format("{}", p) == "(0, 0)");
    }
}

TEST_CASE("Vector2D formatting Tests", "turtlelib::Vector2D v{x, y}") {
    SECTION("Standard Formating Test"){
        turtlelib::Vector2D v{1.5, -2.3};
        std::string s = std::format("{}", v);
        REQUIRE(s == "[1.5, -2.3]");
    }

    SECTION("Vector2D - Zero Vector") {
        turtlelib::Vector2D v{0.0, 0.0};
        REQUIRE(std::format("{}", v) == "[0, 0]");
    }
}