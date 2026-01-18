#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"
#include <sstream>

TEST_CASE("Point2D instream operator>>", "[operator>>]") {
    turtlelib::Point2D p;

    SECTION("Parses (x, y) format") {
        std::stringstream ss("(1.5, 2.5)");
        ss >> p;
        REQUIRE_THAT(p.x, Catch::Matchers::WithinRel(1.5, 0.001));
        REQUIRE_THAT(p.y, Catch::Matchers::WithinRel(2.5, 0.001));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses x y format") {
        std::stringstream ss("3.0 4.0");
        ss >> p;
        REQUIRE_THAT(p.x, Catch::Matchers::WithinRel(3.0, 0.001));
        REQUIRE_THAT(p.y, Catch::Matchers::WithinRel(4.0, 0.001));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Invalid input sets failbit") {
        std::stringstream ss("abc xyz");
        ss >> p;
        REQUIRE(ss.fail() == true);
    }

    SECTION("Partially invalid input sets failbit") {
        std::stringstream ss("(5.0, abc)");
        ss >> p;
        REQUIRE(ss.fail() == true);
    }
}
