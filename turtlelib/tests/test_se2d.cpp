#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"
#include <sstream>
#include <format>

// Twist2D OPERATOR>>:
TEST_CASE("Twist2D instream operator>>", "[operator>>]") {
    turtlelib::Twist2D tw;

    SECTION("Parses <w [units], x, y> format") {
        std::stringstream ss("< 1.0 rad/s, 1.5, 2.5>");
        ss >> tw;
        REQUIRE_THAT(tw.omega, Catch::Matchers::WithinRel(1.0, 0.001));
        REQUIRE_THAT(tw.x, Catch::Matchers::WithinRel(1.5, 0.001));
        REQUIRE_THAT(tw.y, Catch::Matchers::WithinRel(2.5, 0.001));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses w [units] x y format") {
        std::stringstream ss("2.0 rad/s 3.0 4.0");
        ss >> tw;
        REQUIRE_THAT(tw.omega, Catch::Matchers::WithinRel(2.0, 0.001));
        REQUIRE_THAT(tw.x, Catch::Matchers::WithinRel(3.0, 0.001));
        REQUIRE_THAT(tw.y, Catch::Matchers::WithinRel(4.0, 0.001));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    // SECTION("Invalid input sets failbit") {
    //     std::stringstream ss("abc xyz");
    //     ss >> p;
    //     REQUIRE(ss.fail() == true); // reading failed
    // }

    // SECTION("Partially invalid input sets failbit") {
    //     std::stringstream ss("(5.0, abc)");
    //     ss >> p;
    //     REQUIRE(ss.fail() == true); //reading failed
    // }

    // SECTION("Fails if formated like a vector") {
    //     std::stringstream ss("[5.0 5.0]");
    //     ss >> p;
    //     REQUIRE(ss.fail() == true);     // reading failed
    // }
}