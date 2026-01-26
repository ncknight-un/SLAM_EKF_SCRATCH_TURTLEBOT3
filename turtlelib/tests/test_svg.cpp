#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/svg.hpp"
#include <string>
#include <sstream>

using namespace turtlelib;

TEST_CASE("Verified SVG example matches expected output", "[svg]") {
    // Test Write Example from Working SVG Writer:
    turtlelib::Svg svg;

    // Draw a point at (1.0, -1.25) in meters, purple
    svg.draw_point(1.0, -1.25, "purple", "purple");

    // Draw a vector from (0,0) to (1,1), purple
    svg.draw_vector(1.0, 1.0, 0.0, 0.0, "purple");

    // Draw a frame at the origin with x = (1,0), y = (0,1), id = 'a'
    turtlelib::Vector2D x_axis{1.0, 0.0};
    turtlelib::Vector2D y_axis{0.0, 1.0};
    svg.draw_frame(turtlelib::Vector2D{0.0,0.0}, x_axis, y_axis, 'a');

    // Require that the string held by svg is the same as the appropriate output:
   std::string expected_svg = "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
    "<defs>\n"
    "  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n"
    "    <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n"
    "  </marker>\n"
    "</defs>\n"
    "<circle cx=\"504\" cy=\"648\" r=\"5\" stroke=\"purple\" fill=\"purple\" />\n"
    "<line x1=\"408\" y1=\"528\" x2=\"504\" y2=\"432\" stroke=\"purple\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n"
    "<g>\n"
    "<line x1=\"504\" y1=\"528\" x2=\"408\" y2=\"528\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n"
    "<line x1=\"408\" y1=\"432\" x2=\"408\" y2=\"528\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n"
    "<text x=\"398\" y=\"538\">{a}</text>\n"
    "</g>\n"
    "</svg>\n";

    REQUIRE(svg.build_file() == expected_svg);
}


