#include "turtlelib/svg.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace turtlelib {
    Svg::Svg() {
        // Initialize the SVG with the required Headers at the top of the file string:
        commands_.push_back(
            "<svg width=\"8.500000in\" height=\"11.000000in\" "
            "viewBox=\"0 0 816.000000 1056.000000\" "
            "xmlns=\"http://www.w3.org/2000/svg\">\n"
        );

        commands_.push_back(
            "<defs>\n"
            "  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n"
            "    <path transform=\"scale(0.2) translate(6,0)\" "
            "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" "
            "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/>\n"
            "  </marker>\n"
            "</defs>\n"
        );
    }

    double Svg::to_svg_coords_x(const double & x) const {
        // Conversion rate from 1 meter in sim to SVG is 96 pixels. (Origin at 408:528)
        return 408 + x*96;
    }
    double Svg::to_svg_coords_y(const double & y) const {
        // Conversion rate from 1 meter in sim to SVG is 96 pixels. (Origin at 408:528)
        return 528 - y*96;      // y axis is flipped
    }

    void Svg::draw_point(double cx, double cy, std::string stroke, std::string fill) {
        // Convert Point to SVG Coordinates:
        double svg_x = Svg::to_svg_coords_x(cx);
        double svg_y = Svg::to_svg_coords_y(cy);

        // Build the svg command for a point
        std::ostringstream ss;
        ss << "<circle cx=\"" << svg_x
            << "\" cy=\"" << svg_y
            << "\" r=\"3\""
            << " stroke=\"" << stroke << "\""
            << " fill=\"" << fill << "\" />\n";

        commands_.push_back(ss.str());
    }

    void Svg::draw_vector(double x1, double y1, double x2, double y2, std::string color) {
        double svg_x1 = Svg::to_svg_coords_x(x1);
        double svg_y1 = Svg::to_svg_coords_y(y1);
        double svg_x2 = Svg::to_svg_coords_x(x2);
        double svg_y2 = Svg::to_svg_coords_y(y2);

        // Write the svg command for the vector:
        std::ostringstream ss;
        ss << "<line x1=\"" << svg_x1
        << "\" y1=\"" << svg_y1
        << "\" x2=\"" << svg_x2
        << "\" y2=\"" << svg_y2
        << "\" stroke=\"" << color
        << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";

        commands_.push_back(ss.str());
    }

    void Svg::draw_frame(Vector2D v1, Vector2D v2, char frame_id) {
        // Open group tag
        commands_.push_back("<g>\n");

        // X-axis (red) vector: from origin to v1
        draw_vector(v1.x, v1.y, 0.0, 0.0, "red");

        // Y-axis (green) vector: from origin to v2
        draw_vector(v2.x, v2.y, 0.0, 0.0, "green");

        // Text label slightly offset from origin
        std::ostringstream ss;
        ss << "<text x=\"" << Svg::to_svg_coords_x(0.0) - 10    // Shift the title 10 pixels back
        << "\" y=\"" << Svg::to_svg_coords_y(0.0) + 10        // Shift the title 10 pixels down
        << "\">{" << frame_id << "}</text>\n";
        commands_.push_back(ss.str());

        // Close the <g> tag
        commands_.push_back("</g>\n");
    }

    void Svg::write_to_file(const std::string & filename) const {
        std::ofstream file(filename);
        if (!file) {
            throw std::runtime_error("Failed to open SVG file");
        }

        std::ostringstream ss;
        // Extract elements in order added:
        for (const auto & e : commands_) {ss << e;}

        // SVG footer (Needed before write to file!)
        ss << "</svg>\n";

        // Write the output string to the file
        file << ss.str();
    }
} // namespace turtlelib
