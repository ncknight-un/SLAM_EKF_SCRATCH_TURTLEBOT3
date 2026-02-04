#include "turtlelib/diff_drive.hpp"

namespace turtlelib {
    DiffDrive(double wheel_track, double wheel_radius){
        // Set the wheel track and wheel radius for the diff drive model:
        wheel_track_ = wheel_track;
        wheel_radius_ = wheel_radius;
        // Initialize wheel positions and robot base transform:
        phi_right_ = 0.0;
        phi_left_ = 0.0;
        q_ = Transform2D(Vector2D{0.0, 0.0}, 0.0);
    }

    // RETURN FUNCTIONS:
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

    } // End of update_fk()

    Twist2D DiffDrive::compute_ik(const Twist2D & twist) const{

    } // End of compute_ik()

} // namespace turtlelib