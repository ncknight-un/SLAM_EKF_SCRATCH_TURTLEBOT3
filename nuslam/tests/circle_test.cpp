#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "slamlib/circle_reg.hpp"
#include <turtlelib/geometry2d.hpp>
#include <vector>
#include <armadillo>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

TEST_CASE("Verify that the circle fitting algorithm outpouts the appropriate centroid and radius", "[Circle_Reg]") {
    // Create a series of sensor points from the test example1 (Line):
    std::vector<turtlelib::Point2D> points_test1;
    points_test1.push_back(turtlelib::Point2D(1.0, 7.0));
    points_test1.push_back(turtlelib::Point2D(2.0, 6.0));
    points_test1.push_back(turtlelib::Point2D(5.0, 8.0));
    points_test1.push_back(turtlelib::Point2D(7.0, 7.0));
    points_test1.push_back(turtlelib::Point2D(9.0, 5.0));
    points_test1.push_back(turtlelib::Point2D(3.0, 7.0));

    // Create a series of sensor points from the test example2 (Line):
    std::vector<turtlelib::Point2D> points_test2;
    points_test2.push_back(turtlelib::Point2D(-1.0, 0.0));
    points_test2.push_back(turtlelib::Point2D(-0.3, -0.06));
    points_test2.push_back(turtlelib::Point2D(0.3, 0.1));
    points_test2.push_back(turtlelib::Point2D(1.0, 0.0));

    // Create a series of sensor points from the test example2 (Circle): // Simulates what the scanner would see
    std::vector<turtlelib::Point2D> points_test3;
    points_test3.push_back(turtlelib::Point2D(2.0, 2.0));  // P1
    points_test3.push_back(turtlelib::Point2D(2.5, 1.134)); 
    points_test3.push_back(turtlelib::Point2D(3.0, 1.0)); 
    points_test3.push_back(turtlelib::Point2D(3.5, 1.134));
    points_test3.push_back(turtlelib::Point2D(4.0, 2.0));   // P2

    // Create an instance of the CircleReg algorithm:
    slamlib::CircleReg circle_reg1(points_test1);
    slamlib::CircleReg circle_reg2(points_test2);
    slamlib::CircleReg circle_reg3(points_test3);

    // Use the circle fitting algorithm on the two imputs to get the centroids and radii:
    auto [centroid1, radius1, is_circle1] = circle_reg1.fitCircle();
    auto [centroid2, radius2, is_circle2] = circle_reg2.fitCircle();
    auto [centroid3, radius3, is_circle3] = circle_reg3.fitCircle();

    // Verify using Catch2 that the centroids and radii are correct for the two test examples:
    REQUIRE_THAT(centroid1.x, WithinRel(4.61518482, EPS));
    REQUIRE_THAT(centroid1.y, WithinRel(2.807354, EPS));
    REQUIRE_THAT(radius1, WithinRel(4.8275, EPS));
    REQUIRE(!is_circle1); // The first test example should be classified as a circle
    REQUIRE_THAT(centroid2.x, WithinRel(0.4908357, EPS));
    REQUIRE_THAT(centroid2.y, WithinRel(-22.15212, EPS));
    REQUIRE_THAT(radius2, WithinRel(22.17979, EPS)); 
    REQUIRE(!is_circle2); // The second test example should be classified as a circle
    REQUIRE_THAT(centroid3.x, WithinRel(3.0, EPS));
    REQUIRE_THAT(centroid3.y, WithinRel(2.0, EPS));
    REQUIRE_THAT(radius3, WithinRel(1.0, EPS));
    REQUIRE(is_circle3); // The third test example should be classified as a circle
}