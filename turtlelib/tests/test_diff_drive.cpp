#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/diff_drive.hpp"
#include <sstream>
#include <format>
#include <numbers>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

// Forward Path:
TEST_CASE("Forward Path", "[update_fk and compute_ik]") {
    // Create a DiffDrive model with a wheel track of 0.1m and wheel radius of 0.01m.   
    turtlelib::DiffDrive diff_drive(0.1, 0.01);

    SECTION("Forward Kinematics for simple forward path") {
        // Update the wheel positions to move the robot forward 0.2 meters in a straight path.
        diff_drive.update_fk(20.0, 20.0);
        // Check that the robot base transform is updated correctly:
        turtlelib::Transform2D q = diff_drive.get_q();
        REQUIRE_THAT(q.translation().x, WithinRel(0.2, EPS));   // The robot should have moved forward 0.2 meters in the x direction in its frame 
        REQUIRE_THAT(q.translation().y, WithinRel(0.0, EPS));
        REQUIRE_THAT(q.rotation(), WithinRel(0.0, EPS));
    }

    SECTION("Inverse Kinematics for simple forward path") {
        // Compute the required wheel angular velocities for a forward motion of 0.2 meters in x direction:
        turtlelib::Twist2D twist{0.0, 0.2, 0.0};  // Forward motion in x direction only
        turtlelib::wheel_vel vel = diff_drive.compute_ik(twist);

        // Check that the required wheel angular velocities are correct:
        REQUIRE_THAT(vel.v_lw, WithinRel(20, EPS));   // Right wheel should rotate at 20 rad/s
        REQUIRE_THAT(vel.v_rw, WithinRel(20, EPS));   // Left wheel should rotate at 20 rad/s
    }
};

// Pure Rotation Path:
TEST_CASE("Pure Rotation Path", "[update_fk and compute_ik]") {
    // Create a DiffDrive model with a wheel track of 0.1m and wheel radius of 0.01m.   
    turtlelib::DiffDrive diff_drive(0.1, 0.01);

    SECTION("Forward Kinematics for simple rotation path") {
        // Update the wheel positions to only rotate the robot -2 radians in place.
        diff_drive.update_fk(10.0, -10.0);
        // Check that the robot base transform is updated correctly:
        turtlelib::Transform2D q = diff_drive.get_q();
        REQUIRE_THAT(q.translation().x, WithinRel(0.0, EPS));   // The robot should have moved forward 0.0 meters in the x direction in its frame 
        REQUIRE_THAT(q.translation().y, WithinRel(0.0, EPS));
        REQUIRE_THAT(q.rotation(), WithinRel(-2.0, EPS));  // The robot should have rotated -2 radians in place
    }

    SECTION("Inverse Kinematics for simple rotation path") {
        // Compute the required wheel angular velocities for a rotation of 2 radians in place:
        turtlelib::Twist2D twist{2.0, 0.0, 0.0}; 
        turtlelib::wheel_vel vel = diff_drive.compute_ik(twist);

        // Check that the required wheel angular velocities are correct:
        REQUIRE_THAT(vel.v_lw, WithinRel(-10, EPS));   // Right wheel should rotate at -10 rad/s
        REQUIRE_THAT(vel.v_rw, WithinRel(10, EPS));  // Left wheel should rotate at 10 rad/s
    }
};

// Arc Path:
TEST_CASE("Arc Path", "[update_fk and compute_ik]") {
    // Create a DiffDrive model with a wheel track of 0.1m and wheel radius of 0.01m.   
    turtlelib::DiffDrive diff_drive(0.1, 0.01);

    SECTION("Forward Kinematics for simple arc path") {
        // Update the wheel positions to follow an arc path that moves the robot forward 1 meter and rotates it 2.0 rad to the right:
        diff_drive.update_fk(110.0, 90.0);
        // Check that the robot base transform is updated correctly:
        turtlelib::Transform2D q = diff_drive.get_q();
        REQUIRE_THAT(q.translation().x, WithinRel(0.4546, EPS));   // The robot should have moved forward 1.0 meters in the x direction in its frame 
        REQUIRE_THAT(q.translation().y, WithinRel(0.0, EPS));
        REQUIRE_THAT(q.rotation(), WithinRel(2.0, EPS));  // The robot should have rotated -2 radians and moved forward along the path 1 meters
    }

    SECTION("Inverse Kinematics for simple arc path") { 
        // Compute the required wheel angular velocities for an arc path that moves the robot forward 1 meter and rotates it 2.0 rad to the left:
        turtlelib::Twist2D twist{-2.0, 1.0, 0.0}; 
        turtlelib::wheel_vel vel = diff_drive.compute_ik(twist);

        // Check that the required wheel angular velocities are correct:
        REQUIRE_THAT(vel.v_lw, WithinRel(90.0, EPS));   // Right wheel should rotate at 90.0 rad/s
        REQUIRE_THAT(vel.v_rw, WithinRel(110.0, EPS));  // Left wheel should rotate at 110.0 rad/s
    }
};

// An impossible to follow twist is provided: 
TEST_CASE("Impossible Twist", "[compute_ik]") {
    // Create a DiffDrive model with a wheel track of 0.1m and wheel radius of 0.01m.   
    turtlelib::DiffDrive diff_drive(0.1, 0.01);

    // Compute the required wheel angular velocities for a twist that has a non-zero y component (which is impossible for a diff drive):
    turtlelib::Twist2D twist{0.0, 0.0, 1.0}; 

    // Verify that the twist throws a logic error if a non-zero y component is provided since this is an impossible twist for a diff drive:
    REQUIRE_THROWS_AS(
        // compute_ik:
        diff_drive.compute_ik(twist),
        std::logic_error      // exception logic_error
    );
};