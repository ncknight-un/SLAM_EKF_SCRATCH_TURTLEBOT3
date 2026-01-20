#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"
#include <sstream>
#include <format>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

// turtlelib::Twist2D OPERATOR>>:
TEST_CASE("turtlelib::Twist2D instream operator>>", "[operator>>]") {
    turtlelib::Twist2D tw;

    SECTION("Parses <w [units], x, y> format") {
        std::stringstream ss("<1.0 rad/s, 1.5, 2.5>");
        ss >> tw;
        REQUIRE_THAT(tw.omega, WithinRel(1.0, EPS));
        REQUIRE_THAT(tw.x, WithinRel(1.5, EPS));
        REQUIRE_THAT(tw.y, WithinRel(2.5, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses w [units] x y format") {
        std::stringstream ss("2.0 rad/s 3.0 4.0");
        ss >> tw;
        REQUIRE_THAT(tw.omega, WithinRel(2.0, EPS));
        REQUIRE_THAT(tw.x, WithinRel(3.0, EPS));
        REQUIRE_THAT(tw.y, WithinRel(4.0, EPS));
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
TEST_CASE("Transform2D constructors", "[Transform2D]") {
    SECTION("Default constructor creates identity transform") {
        // Create identity tf:
        turtlelib::Transform2D tf;
        // Pull out rotation and translation:
        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(0.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }

    SECTION("Pure translation constructor") {
        // Create the pure translation constructor:
        turtlelib::Vector2D v{3.5, -5.5};
        turtlelib::Transform2D tf(v);
        // Pull out rotation and translation:
        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, WithinRel(3.5, EPS));
        REQUIRE_THAT(trans.y, WithinRel(-5.5, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }

    SECTION("Pure rotation constructor") {
        // Create pure rotation constructor:
        double radians = 0.555;
        turtlelib::Transform2D tf(radians);
        // Pull out rotation and translation:
        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(0.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.555, EPS));
    }

    SECTION("Translation + rotation constructor") {
        // Create tf with translation and rotation:
        turtlelib::Vector2D v{3.0, -1.0};
        double radians = 0.123;
        turtlelib::Transform2D tf(v, radians);

        turtlelib::Vector2D trans = tf.translation();
        double rot = tf.rotation();

        REQUIRE_THAT(trans.x, WithinRel(3.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(-1.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.123, EPS));
    }
}

// APPLIED TRANSFORMS (Translation and Rotation):
TEST_CASE("Applied Transforms", "[operator()]") {
    // Create tf with translation and rotation:
    turtlelib::Vector2D v{1.0, -1.0};
    double radians = 2.5;
    turtlelib::Transform2D tf(v, radians);

    SECTION("Transpose origin") {
        // Create  point:
        turtlelib::Point2D p{0.0, 0.0};
        // Apply Transform to the point:
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(1.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-1.0, EPS));
    }
    SECTION("Transpose a point (Quadrant 1)") {
        // Create  point:
        turtlelib::Point2D p{1.0, 1.0};
        // Apply Transform to the point:
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(-0.3996, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-1.2026, EPS));
    }
    SECTION("Transpose a point (Quadrant 2)") {
        // Create  point:
        turtlelib::Point2D p{-1.0, 1.0};
        // Apply Transform to the point:
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(1.2027, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-2.3996, EPS));
    }
    SECTION("Transpose a point (Quadrant 3)") {
        // Create  point:
        turtlelib::Point2D p{-1.0, -1.0};
        // Apply Transform to the point:
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(2.3996, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-0.7974, EPS));
    }
    SECTION("Transpose a point (Quadrant 4)") {
        // Create  point:
        turtlelib::Point2D p{1.0, -1.0};
        // Apply Transform to the point:
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(0.7974, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(0.3996, EPS));
    }
    SECTION("Transpose a Vector - With Translation") {
        // Create vector:
        turtlelib::Vector2D v{1.0, -1.0};
        // Apply Transform to the vector:
        turtlelib::Vector2D new_v = tf(v);

        REQUIRE_THAT(new_v.x, WithinRel(-0.2026, EPS));
        REQUIRE_THAT(new_v.y, WithinRel(1.3996, EPS));
    }
    SECTION("Transpose a Twist2D - No Omega") {
        // Create vector:
        turtlelib::Twist2D tw{0.0, 1.0, -1.0};
        // Apply Transform to the vector:
        turtlelib::Twist2D new_tw = tf(tw);

        REQUIRE_THAT(new_tw.omega, WithinRel(0.0, EPS));
        REQUIRE_THAT(new_tw.x, WithinRel(-0.2027, EPS));
        REQUIRE_THAT(new_tw.y, WithinRel(1.3996, EPS));
    }
    SECTION("Transpose a Twist2D - Neg Omega") {
        // Create vector:
        turtlelib::Twist2D tw{-3.0, 1.0, -1.0};
        // Apply Transform to the vector:
        turtlelib::Twist2D new_tw = tf(tw);

        REQUIRE_THAT(new_tw.omega, WithinRel(-3, EPS));
        REQUIRE_THAT(new_tw.x, WithinRel(2.7973, EPS));
        REQUIRE_THAT(new_tw.y, WithinRel(4.3996, EPS));
    }
    SECTION("Transpose a Twist2D - Pos Omega") {
        // Create vector:
        turtlelib::Twist2D tw{3.0, 1.0, -1.0};
        // Apply Transform to the vector:
        turtlelib::Twist2D new_tw = tf(tw);

        REQUIRE_THAT(new_tw.omega, WithinRel(3.0, EPS));
        REQUIRE_THAT(new_tw.x, WithinRel(-3.2027, EPS));
        REQUIRE_THAT(new_tw.y, WithinRel(-1.6004, EPS));
    }
}

// ############################################ Begin_Citation [5] ############################
// I used a language model to reproduce the code I wrote above but for the cases where there is only rotation, or only translation.

// ROTATION ONLY (No Translation)
TEST_CASE("Applied Transforms - Rotation Only", "[operator()]") {
    // Create transform with rotation only:
    turtlelib::Vector2D v{0.0, 0.0};  // No translation
    double radians = 2.5;
    turtlelib::Transform2D tf(v, radians);

    SECTION("Rotate origin") {
        turtlelib::Point2D p{0.0, 0.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(0.0, EPS));
    }
    SECTION("Rotate a point (Quadrant 1)") {
        turtlelib::Point2D p{1.0, 1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(-1.3996, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-0.2026, EPS));
    }
    SECTION("Rotate a point (Quadrant 2)") {
        turtlelib::Point2D p{-1.0, 1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(0.2026, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-1.3996, EPS));
    }
    SECTION("Rotate a point (Quadrant 3)") {
        turtlelib::Point2D p{-1.0, -1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(1.3996, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(0.2026, EPS));
    }
    SECTION("Rotate a point (Quadrant 4)") {
        turtlelib::Point2D p{1.0, -1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(-0.2026, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(1.3996, EPS));
    }
    SECTION("Rotate a Vector") {
        turtlelib::Vector2D v{1.0, -1.0};
        turtlelib::Vector2D new_v = tf(v);

        REQUIRE_THAT(new_v.x, WithinRel(-0.2026, EPS));
        REQUIRE_THAT(new_v.y, WithinRel(1.3996, EPS));
    }
}

// TRANSLATION ONLY (No Rotation)
TEST_CASE("Applied Transforms - Translation Only", "[operator()]") {
    // Create transform with translation only:
    turtlelib::Vector2D v{1.0, -1.0};
    double radians = 0.0;  // No rotation
    turtlelib::Transform2D tf(v, radians);

    SECTION("Translate origin") {
        turtlelib::Point2D p{0.0, 0.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(1.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-1.0, EPS));
    }
    SECTION("Translate a point (Quadrant 1)") {
        turtlelib::Point2D p{1.0, 1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(2.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(0.0, EPS));
    }
    SECTION("Translate a point (Quadrant 2)") {
        turtlelib::Point2D p{-1.0, 1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(0.0, EPS));
    }
    SECTION("Translate a point (Quadrant 3)") {
        turtlelib::Point2D p{-1.0, -1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-2.0, EPS));
    }
    SECTION("Translate a point (Quadrant 4)") {
        turtlelib::Point2D p{1.0, -1.0};
        turtlelib::Point2D new_p = tf(p);

        REQUIRE_THAT(new_p.x, WithinRel(2.0, EPS));
        REQUIRE_THAT(new_p.y, WithinRel(-2.0, EPS));
    }
    SECTION("Translate a Vector") {
        turtlelib::Vector2D v{1.0, -1.0};
        turtlelib::Vector2D new_v = tf(v);

        // Vectors are not affected by translation
        REQUIRE_THAT(new_v.x, WithinRel(1.0, EPS));
        REQUIRE_THAT(new_v.y, WithinRel(-1.0, EPS));
    }
}
// ########################################## End_Citation [5] ####################################

// TRANSFORM2D Inverse
TEST_CASE("Transform2D inverse", "[inv()]") {
    SECTION("Inverse - With Translation and Rotation") {
        turtlelib::Transform2D Tab(turtlelib::Vector2D{1.0, 1.0}, 3.0);
        turtlelib::Transform2D Tba = Tab.inv();
        // Extract the inverted translation and rotation:
        turtlelib::Vector2D trans = Tba.translation();
        double rot = Tba.rotation();
        // Verify that the Transform was Inverted:
        REQUIRE_THAT(trans.x, WithinRel(0.849, EPS));
        REQUIRE_THAT(trans.y, WithinRel(1.131, EPS));
        REQUIRE_THAT(rot, WithinRel(-3.0, EPS));
    }
    SECTION("Inverse - No Rotation") {
        turtlelib::Transform2D Tab(turtlelib::Vector2D{1.0, 1.0}, 0.0);
        turtlelib::Transform2D Tba = Tab.inv();
        // Extract the inverted translation and rotation:
        turtlelib::Vector2D trans = Tba.translation();
        double rot = Tba.rotation();
        // Verify that the Transform was Inverted:
        REQUIRE_THAT(trans.x, WithinRel(-1.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(-1.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }
    SECTION("Inverse - No Translation") {
        turtlelib::Transform2D Tab(turtlelib::Vector2D{0.0, 0.0}, 3.0);
        turtlelib::Transform2D Tba = Tab.inv();
        // Extract the inverted translation and rotation:
        turtlelib::Vector2D trans = Tba.translation();
        double rot = Tba.rotation();
        // Verify that the Transform was Inverted:
        REQUIRE_THAT(trans.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(0.0, EPS));
        REQUIRE_THAT(rot, WithinRel(-3.0, EPS));
    }
    SECTION("Inverse - No Translation or rotation") {
        turtlelib::Transform2D Tab(turtlelib::Vector2D{0.0, 0.0}, 0.0);
        turtlelib::Transform2D Tba = Tab.inv();
        // Extract the inverted translation and rotation:
        turtlelib::Vector2D trans = Tba.translation();
        double rot = Tba.rotation();

        // Verify that the Transform was Inverted:
        REQUIRE_THAT(trans.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(0.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }
}

// TRANSFORM2D Multiplication
TEST_CASE("Transform2D Multiplication", "[operator*]") {
    SECTION("Multiplication - Non-Zero TFs") {
        // Define Transfroms rhs and lhs
        turtlelib::Transform2D lhs(turtlelib::Vector2D{1.0, 10.0});
        turtlelib::Transform2D rhs(turtlelib::Vector2D{5.0, 2.0});
        // operator *=:
        lhs *= rhs;

        // Extract the translation and rotation:
        turtlelib::Vector2D trans = lhs.translation();
        double rot = lhs.rotation();

        // Verification of Multiplied TFs:
        REQUIRE_THAT(trans.x, WithinRel(6.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(12.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }
        SECTION("Multiplication - One Zero TF") {
        // Define Transfroms rhs and lhs
        turtlelib::Transform2D lhs(turtlelib::Vector2D{0.0, 0.0});
        turtlelib::Transform2D rhs(turtlelib::Vector2D{0.0, 2.0});
        // operator *=:
        lhs *= rhs;

        // Extract the translation and rotation:
        turtlelib::Vector2D trans = lhs.translation();
        double rot = lhs.rotation();

        // Verification of Multiplied TFs:
        REQUIRE_THAT(trans.x, WithinRel(0.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(2.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }
        SECTION("Multiplication - Negative Valued TFs") {
        // Define Transfroms rhs and lhs
        turtlelib::Transform2D lhs(turtlelib::Vector2D{-1.0, -1.0});
        turtlelib::Transform2D rhs(turtlelib::Vector2D{-4.0, -2.0});
        // operator *=:
        lhs *= rhs;

        // Extract the translation and rotation:
        turtlelib::Vector2D trans = lhs.translation();
        double rot = lhs.rotation();

        // Verification of Multiplied TFs:
        REQUIRE_THAT(trans.x, WithinRel(-5.0, EPS));
        REQUIRE_THAT(trans.y, WithinRel(-3.0, EPS));
        REQUIRE_THAT(rot, WithinRel(0.0, EPS));
    }
}

// TRANSFORM2D Operator>>:
TEST_CASE("turtlelib::Transform2D instream operator>>", "[operator>>]") {
    turtlelib::Transform2D tf;

    SECTION("Parses <omega [units], x, y> format") {
        std::stringstream ss("<1.0 rad/s, 1.5, 2.5>");
        ss >> tf;
        // Verify that the transform was parsed correctly:
        REQUIRE_THAT(tf.rotation(), WithinRel(1.0, EPS));
        REQUIRE_THAT(tf.translation().x, WithinRel(1.5, EPS));
        REQUIRE_THAT(tf.translation().y, WithinRel(2.5, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Parses omega [units] x y format") {
        std::stringstream ss("2.0 rad/s 3.0 4.0");
        ss >> tf;
        // Verify that the transform was parsed correctly:
        REQUIRE_THAT(tf.rotation(), WithinRel(2.0, EPS));
        REQUIRE_THAT(tf.translation().x, WithinRel(3.0, EPS));
        REQUIRE_THAT(tf.translation().y, WithinRel(4.0, EPS));
        REQUIRE(!ss.fail()); // reading succeeded
    }

    SECTION("Invalid input sets failbit") {
        std::stringstream ss("a b c");
        ss >> tf;
        // Verify parsing fails due to poor input
        REQUIRE(ss.fail() == true); // reading failed
    }

    SECTION("Partially invalid input sets failbit") {
        std::stringstream ss("5.0, 2.0, abc");
        ss >> tf;
        // Verify parsing fails due to poor input
        REQUIRE(ss.fail() == true); //reading failed
    }

    SECTION("Fails if formated like a vector") {
        std::stringstream ss("[5.0, 4.0, 5.0]");
        ss >> tf;
        // Verify parsing fails due to poor input
        REQUIRE(ss.fail() == true);     // reading failed
    }
}

// Transform2D GETTER Functions:
TEST_CASE("turtlelib::Transform2D Getter() Functions>>", "[.getter()]") {
    turtlelib::Transform2D tf;

    SECTION("Rotation Getter()") {
        std::stringstream ss("2.0 rad/s 3.0 4.0");
        ss >> tf;
        // Get Rotation Values:
        double rot = tf.rotation();

        REQUIRE_THAT(rot, WithinRel(2.0, EPS));
        REQUIRE(ss.fail() == false); //reading passed
    }

    SECTION("Translation Getter()") {
        std::stringstream ss("2.0 rad/s 3.0 4.0");
        ss >> tf;
        // Get Translation Values:
        double trans_x = tf.translation().x;
        double trans_y = tf.translation().y;

        REQUIRE_THAT(trans_x, WithinRel(3.0, EPS));
        REQUIRE_THAT(trans_y, WithinRel(4.0, EPS));
        REQUIRE(ss.fail() == false);     // reading passed
    }
}

// FORMATING TESTS:
TEST_CASE("Twist2D formatting Tests", "turtlelib::Twist2D p{x, y}") {
    SECTION("Standard Formating Test - Default") {
        turtlelib::Twist2D tw{-3.0, 1.0, -1.0};
        std::string s = std::format("{:R}", tw);
        REQUIRE(s == "<-3.00000 rad/s, 1.00000, -1.00000>");
    }

    SECTION("Standard Formating Test - Origin") {
        turtlelib::Twist2D tw{0.0, 0.0, 0.0};
        std::string s = std::format("{:R}", tw);
        REQUIRE(s == "<0.00000 rad/s, 0.00000, 0.00000>");
    }

    SECTION("Standard Formating Test - Degrees") {
        turtlelib::Twist2D tw{-3.0, 1.0, -1.0};
        std::string s = std::format("{:D}", tw);
        REQUIRE(s == "<-3.00000 deg/s, 1.00000, -1.00000>");
    }

    SECTION("Standard Formating Test - Radians") {
        turtlelib::Twist2D tw{-3.0, 1.0, -1.0};
        std::string s = std::format("{:R}", tw);
        REQUIRE(s == "<-3.00000 rad/s, 1.00000, -1.00000>");
    }

    SECTION("Standard Formating Test - Radians") {
        turtlelib::Twist2D tw{-3.0, 1.0, -1.0};
        std::string s = std::format("{:R}", tw);
        REQUIRE(s == "<-3.00000 rad/s, 1.00000, -1.00000>");
    }
}

// TEST_CASE("Transform2D formatting Tests", "turtlelib::Transform2D v{x, y}") {
//     SECTION("Standard Formating Test"){
//         turtlelib::Vector2D v{1.5, -2.3};
//         std::string s = std::format("{}", v);
//         REQUIRE(s == "[1.5, -2.3]");
//     }

//     SECTION("Vector2D - Zero Vector") {
//         turtlelib::Vector2D v{0.0, 0.0};
//         REQUIRE(std::format("{}", v) == "[0, 0]");
//     }
// }
