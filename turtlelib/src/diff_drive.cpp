#include "turtlelib/diff_drive.hpp"

namespace turtlelib {
    DiffDrive(double wheel_track, double wheel_radius){
        // Set the wheel track and wheel radius for the diff drive model:
        wheel_track_ = wheel_track;
        wheel_radius_ = wheel_radius;

        // Initialize wheel positions and robot base transform:
        phi_right_ = 0.0;
        phi_left_ = 0.0;
        q_ = Transform2D(Vector2D{0.0, 0.0}, 0.0);  // Initialize q_ to identity transform
    } // End of DiffDrive Constructo

    // Return Functions:
    double phi_right_() const{
        return phi_right_;
    } // End of phi_right_()

    double phi_left_() const{
        return phi_left_;
    } // End of phi_left_()

    turtlelib::Transform2D q_() const{
        return q_;
    } // End of q_()

    // Kinematics Functions:
    void DiffDrive::update_fk(double phi_right, double phi_left){
        // Compute the new robot base transform based on the updated wheel positions:
        // Assumptions:
        //
        // - Both Wheels started rotating simultaneously.
        // - Both wheels stopped rotating simultaneously.
        // - The wheels rotated at a constant veloicity.
        // - The wheels achieved their velocity instanateously (i.e. no acceleration or deceleration).
        // - The wheels did not slip.
        // - The wheels rotated the shorted distance possible to get to their final configuration from the starting configuration.

        // Compute the change in wheel angles since the last update based on the new wheel positions:
        double delta_phi_right = phi_right - phi_right_;    // Change in right wheel angle since last update.
        double delta_phi_left = phi_left - phi_left_;       // Change in left wheel angle since last update.

        // Determine the body's rotation due to the change in wheel angles (Equation 1 - See doc/Kinematics.pdf):
        double delta_theta = (wheel_radius_ / wheel_track_) * (delta_phi_right - delta_phi_left);

        // Determine the change in x position due to the change in wheel angles in the body frame (Equation 2 - See doc/Kinematics.pdf):
        double delta_x = (wheel_radius_ / 2.0) * (delta_phi_right + delta_phi_left);

        // Update the robot base transform q_ based on the change in position and orientation due to the new wheel positions:
        q_.x += delta_x;
        q_.y += 0.0;    // We assume no change in y position since the robot is a differential drive and cannot move sideways.
        q_.theta += delta_theta;
        // Normalize the theta change to get the shortest dist possible:
        q_.theta = turtlelib::normalize_angle(q_.theta);

        // Update the wheel configuration with the new values:
        phi_right_ = phi_right;
        phi_left_ = phi_left;
    } // End of update_fk()

    turtlelib::wheel_vel DiffDrive::compute_ik(const Twist2D & twist) const{
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
        return turtlelib::wheel_vel{v_rw, v_lw}; 
    } // End of compute_ik()
} // namespace turtlelib