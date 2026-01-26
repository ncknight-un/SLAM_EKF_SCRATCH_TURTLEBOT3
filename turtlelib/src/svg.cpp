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
            << "\" r=\"5\""
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

    void Svg::draw_frame(Vector2D origin, Vector2D v1, Vector2D v2, char frame_id) {
        // Normalize v1 and v2 to unit length
        double len_x = std::sqrt(v1.x * v1.x + v1.y * v1.y);
        double len_y = std::sqrt(v2.x * v2.x + v2.y * v2.y);

        turtlelib::Vector2D x_axis{ v1.x / len_x, v1.y / len_x };
        turtlelib::Vector2D y_axis{ v2.x / len_y, v2.y / len_y };

        // Open group tag
        commands_.push_back("<g>\n");

        // X-axis (red) from origin
        draw_vector(origin.x + x_axis.x, origin.y + x_axis.y, origin.x, origin.y, "red");

        // Y-axis (green) from origin
        draw_vector(origin.x + y_axis.x, origin.y + y_axis.y, origin.x, origin.y, "green");

        // Text label slightly offset from origin
        std::ostringstream ss;
        ss << "<text x=\"" << Svg::to_svg_coords_x(origin.x) - 10
        << "\" y=\"" << Svg::to_svg_coords_y(origin.y) + 10
        << "\">{" << frame_id << "}</text>\n";
        commands_.push_back(ss.str());

        // Close the <g> tag
        commands_.push_back("</g>\n");
    }

    std::string Svg::build_file() {
        std::ostringstream ss;
        // Extract elements in order added to the vector:
        for (const auto & e : commands_) {ss << e;}

        // SVG footer (Needed before write to file!)
        ss << "</svg>\n";

        // Return the build concatenated string: 
        return ss.str();
    }

    void Svg::write_to_file(const std::string & filename, std::string file_contents) const {
        std::ofstream file(filename);
        if (!file) {
            throw std::runtime_error("Failed to open SVG file");
        }

        file << file_contents;
    }
} // namespace turtlelib
