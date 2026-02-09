#include "turtlelib/diff_drive.hpp"

namespace turtlelib {
    DiffDrive::DiffDrive(double wheel_track, double wheel_radius){
        // Set the wheel track and wheel radius for the diff drive model:
        wheel_track_ = wheel_track;
        wheel_radius_ = wheel_radius;

        // Initialize wheel positions and robot base transform:
        phi_left_ = 0.0;
        phi_right_ = 0.0;
        q_ = Transform2D(Vector2D{0.0, 0.0}, 0.0);  // Initialize q_ to identity transform
    } // End of DiffDrive Constructor

    // Return Functions:
    double DiffDrive::get_phi_right() const{
        return phi_right_;
    } // End of get_phi_right)

    double DiffDrive::get_phi_left() const{
        return phi_left_;
    } // End of get_phi_left()

    turtlelib::Transform2D DiffDrive::get_q() const{
        return q_;
    } // End of get_q()

    void DiffDrive::set_q(const turtlelib::Transform2D & q) {
        q_ = q;
    } // End of set_q()

    // Kinematics Functions:
    void DiffDrive::update_fk(double phi_left, double phi_right) {
        // Compute the new robot base transform based on the updated wheel positions:
        // Assumptions:
        //
        // - Both Wheels started rotating simultaneously.
        // - Both wheels stopped rotating simultaneously.
        // - The wheels rotated at a constant veloicity.
        // - The wheels achieved their velocity instanateously (i.e. no acceleration or deceleration).
        // - The wheels did not slip.
        // - The wheels rotated the shorted distance possible to get to their final configuration from the starting configuration.
        // Compute change in wheel angles since last update
        double delta_phi_left  = turtlelib::normalize_angle(phi_left  - phi_left_);
        double delta_phi_right = turtlelib::normalize_angle(phi_right - phi_right_);

        // Compute change in orientation
        double delta_theta = (wheel_radius_ / wheel_track_) * (-delta_phi_left + delta_phi_right);
        delta_theta = turtlelib::normalize_angle(delta_theta);

        // ############################### Begin_Citation [12] ############################
        // Spoke with Derick about this issue. My blue odometry bot was sliding sideways when stopping.
        // Apparently I wasn't setting it to turn on the arc correctly.
        // Compute linear displacement along the wheels
        double delta_x = (wheel_radius_ / 2.0) * (delta_phi_right + delta_phi_left);

        double dx_body, dy_body;
        if (std::abs(delta_theta) < 1e-6) {
            // Straight-line approximation for small rotation
            dx_body = delta_x;
            dy_body = 0.0;
        } else {
            // Exact arc integration
            double R = delta_x / delta_theta;         // Radius of curvature
            dx_body = R * sin(delta_theta);
            dy_body = R * (1.0 - cos(delta_theta));
        }
        // ############################### End_Citation [12] ##############################

        // Apply the displacement in the body frame to the current pose:
        Transform2D delta_q(Vector2D{dx_body, dy_body}, delta_theta);
        q_ = q_ * delta_q;  // Apply the change to the current pose

        // Update wheel positions
        phi_left_  = phi_left;
        phi_right_ = phi_right;
    }

    turtlelib::wheel_vel DiffDrive::compute_ik(const Twist2D & twist) {
        // Compute the required wheel angular velocities based on the desired twist for the robot base:
        double v_x = twist.x;  // Linear velocity in x direction
        double v_y = twist.y;  // Linear velocity in y direction (should be zero for a diff drive)
        double omega = twist.omega;  // Angular velocity

        // Throw an error if the twist is not feasable. (ie. v_y is not zero since the robot cannot move sideways):
        if (std::abs(v_y) > 1.0e-12) {
            throw std::logic_error("Can not achieve nonzero body-frame y velocity (no slip assumption violated) - Error");
        }

        // Compute the required wheel linear velocities from the desired twist (Equation 3 & 4 - See doc/Kinematics.pdf):
        double v_right = v_x - (wheel_track_ / 2.0) * omega;
        double v_left = v_x + (wheel_track_ / 2.0) * omega;

        // Convert linear velocities to angular velocities for the wheels (Equation 5 & 6 - See doc/Kinematics.pdf):
        double v_rw = v_right / wheel_radius_;
        double v_lw = v_left / wheel_radius_;
        return turtlelib::wheel_vel{v_lw, v_rw};  
    } // End of compute_ik()

    turtlelib::Twist2D DiffDrive::compute_twist(turtlelib::Vector2D v) const {
        // Compute the current twist of the robot base based on the current wheel velocities:
        double v_left = (v.x - phi_left_) * wheel_radius_;
        double v_right = (v.y - phi_right_) * wheel_radius_;

        // Compute the linear and angular velocity of the robot base from the wheel velocities (Equation 7, 8, & 9 - See doc/Kinematics.pdf):
        double v_x = (v_right + v_left) / 2.0;
        double v_y = 0.0; // We assume no sideways velocity for a diff drive.
        double omega = (-v_left + v_right) / wheel_track_;
        return turtlelib::Twist2D{omega, v_x, v_y};
    } // End of compute_twist()
} // namespace turtlelib