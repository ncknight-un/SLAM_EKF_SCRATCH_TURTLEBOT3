#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"
#include <sstream>
#include <format>

// turtlelib::Twist2D OPERATOR>>:
TEST_CASE("turtlelib::Twist2D instream operator>>", "[operator>>]") {      // Nolan, Knight
    turtlelib::Twist2D tw;

    SECTION("Parses <w [units], x, y> format") {
        std::stringstream ss("<1.0 rad/s, 1.5, 2.5>");
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

    SECTION("Invalid input sets failbit") {
        std::stringstream ss("a b c");
        ss >> tw;
        REQUIRE(ss.fail() == true); // reading failed
    }

    SECTION("Partially invalid input sets failbit") {
        std::stringstream ss("5.0, 2.0, abc");
        ss >> tw;
        REQUIRE(ss.fail() == true); //reading failed
    }

    SECTION("Fails if formated like a vector") {
        std::stringstream ss("[5.0, 4.0, 5.0]");
        ss >> tw;
        REQUIRE(ss.fail() == true);     // reading failed
    }
}

// CONSTRUCTORS:
TEST_CASE("turtlelib::Transform2D constructors", "[turtlelib::Transform2D]") {  // Nolan, Knight

    SECTION("Default constructor creates identity transform") {
        // Create identity tf:
        turtlelib::Transform2D tf;
        // Pull out rotation and translation:
        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, Catch::Matchers::WithinRel(0.0, 0.001));
        REQUIRE_THAT(trans.y, Catch::Matchers::WithinRel(0.0, 0.001));
        REQUIRE_THAT(rot, Catch::Matchers::WithinRel(0.0, 0.001));
    }

    SECTION("Pure translation constructor") {
        // Create the pure translation constructor:
        turtlelib::Vector2D v{3.5, -5.5};
        turtlelib::Transform2D tf(v);
        // Pull out rotation and translation:
        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, Catch::Matchers::WithinRel(3.5, 0.001));
        REQUIRE_THAT(trans.y, Catch::Matchers::WithinRel(-5.5, 0.001));
        REQUIRE_THAT(rot, Catch::Matchers::WithinRel(0.0, 0.001));
    }

    SECTION("Pure rotation constructor") {
        // Create pure rotation constructor:
        double radians = 0.555;
        turtlelib::Transform2D tf(radians);
        // Pull out rotation and translation:
        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, Catch::Matchers::WithinRel(0.0, 0.001));
        REQUIRE_THAT(trans.y, Catch::Matchers::WithinRel(0.0, 0.001));
        REQUIRE_THAT(rot, Catch::Matchers::WithinRel(0.555, 0.001));
    }

    SECTION("Translation + rotation constructor") {
        // Create tf with translation and rotation:
        turtlelib::Vector2D v{3.0, -1.0};
        double radians = 0.123;
        turtlelib::Transform2D tf(v, radians);

        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, Catch::Matchers::WithinRel(3.0, 0.001));
        REQUIRE_THAT(trans.y, Catch::Matchers::WithinRel(-1.0, 0.001));
        REQUIRE_THAT(rot, Catch::Matchers::WithinRel(0.123, 0.001));
    }
}

// APPLIED TRANSFORMS:
TEST_CASE("Transforms", "[operator()]")

// INVERSE TRANSFORM:
TEST_CASE("inverse", "[transform]"){        // Nolan Knight
    turtlelib::Transform2D tf;

    //auto trans = tf.translation();
}