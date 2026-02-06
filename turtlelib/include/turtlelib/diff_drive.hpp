#ifndef TURTLELIB_DD_INCLUDE_GUARD_HPP
#define TURTLELIB_DD_INCLUDE_GUARD_HPP
/// \file
/// \brief models the kinematics of a differential drive robot with a given wheel track and wheel radius. 

#include <iosfwd>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/angle.hpp"

namespace turtlelib {
/// \brief a custom struct to hold and return the wheel velocities:
struct wheel_vel {
    /// \brief the right wheel velocity
    double v_rw = 0.0;

    /// \brief the left wheel velocity
    double v_lw = 0.0;
}

/// \brief a diff drive kinematics model for a two wheeled differential drive robot.
class DiffDrive{
    public:
        /// \brief Create a DiffDrive model with wheel_track and wheel_radius
        ///
        /// \param wheel_track - the distance between the wheels
        /// \param wheel_radius - the radius of each wheel
        DiffDrive(double wheel_track, double wheel_radius);

        /// \brief get the position of the right wheel
        /// \return the angular displacement, in radians
        double phi_right_() const;

        /// \brief get the position of the left wheel
        /// \return the angular displacement, in radians
        double phi_left_() const;

        /// \brief get the position of the robot base
        /// \return the current Transform2D of the robot base
        turtlelib::Transform2D q_() const;
 
        /// \brief update the robot base transform based on new wheel positions - Forward Kinematics
        /// \param phi_right - the new right wheel angular displacement, in radians
        /// \param phi_left - the new left wheel angular displacement, in radians
        void update_fk(double phi_right, double phi_left);

        /// \brief Given a desired twist for the robot base, compute the required wheel velocities - Inverse Kinematics
        /// \param twist - the desired twist for the robot base
        /// \return the required wheel angular velocities as a Twist2D where omega is unused, 
        // x is the right wheel angular velocity, and y is the left wheel angular velocity
        Twist2D compute_ik(const Twist2D & twist) const

    private:
        double wheel_track_;  ///< distance between the wheels
        double wheel_radius_; ///< radius of each wheel
        double phi_right_;  ///< right wheel angle
        double phi_left_;   ///< left wheel angle
        turtlelib::Transform2D q_;  ///< current Transfrom of the robot base
}; // End of class DiffDrive
} // namespace turtlelib
