#ifndef SLAMLIB_EKF_HPP_INCLUDE_GUARD
#define SLAMLIB_EKF_HPP_INCLUDE_GUARD
/// \file
/// \brief Extended Kalman Filter implementation. 
/// \author Nolan Knight
/// 
/// This file contains the implementation of the Extended Kalman Filter (EKF) for SLAM.
/// At each timestep (t), EKF filter SLAM takes odometry u_t and sensor measurements z_t, 
// and produces an estimate of the robot's pose and the map of the environment combined_state_.

#include <sstream>
#include <format>
#include <stdexcept>
#include <cmath>
#include <iomanip>
#include <iosfwd>
#include <vector>
#include <armadillo>
#include <turtlelib/angle.hpp>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>

namespace slamlib {
    /// \brief A class for performing SLAM using an Extended Kalman Filter
    class EKF {
    public:
        /// \brief Construct a EKF object with no given initial state (defaults to (0,0,0) with large covariance)
        /// \param num_landmarks The expected number of landmarks in the environment
        /// \param process_noise The process noise covariance for the robot's motion
        /// \param measurement_noise The measurement noise covariance for the robot's sensors
        EKF(int num_landmarks = 3, double process_noise = 0.01, double measurement_noise = 0.01);

        /// \brief Construct a EKF object with the given initial state
        /// \param initial_state The initial state vector (theta, x, y)
        /// \param num_landmarks The expected number of landmarks in the environment
        /// \param process_noise The process noise covariance for the robot's motion
        /// \param measurement_noise The measurement noise covariance for the robot's sensors
        EKF(const arma::colvec& initial_state, int num_landmarks = 3, double process_noise = 0.1, double measurement_noise = 0.01);

        /// \brief Perform a prediction step of the EKF SLAM algorithm
        /// \param control_input The control input vector
        /// \param dt The time step duration
        void predict(const turtlelib::Twist2D& control_input);

        /// \brief Perform an update step of the EKF SLAM algorithm
        /// \param The measurement vector holding the range and bearing to the observed landmark
        /// \param The id of the landmark being measured
        void updateEKF(const arma::colvec& measurement, int landmark_id);

        /// \brief Function to Update the Measuerement Model Matrix H based on which landmark is being observed
        /// \param The id of the landmark being measured
        /// \param The total number of landmarks in the map
        void updateMeasurementModelMatrix(int landmark_id, int total_landmarks);

        /// \brief Function to add a new landmark to the map
        /// \param measurement for the new landmark being observed
        void addLandmark(const arma::colvec& measurement);

        /// \brief Get the current state estimate of the robot (theta, x, y)
        /// \return The current state vector
        turtlelib::Transform2D getState() const;

        /// \brief Get the current combined state estimate of the robot and the map:
        /// \return The current combined state vector (robot state + map state)
        arma::colvec getCombinedState() const;

        /// \brief Get the current covariance estimate
        /// \return The current covariance matrix
        arma::mat getCovariance() const; 

        /// \brief Get the Kalman Gain matrix
        /// \return The current Kalman Gain matrix
        arma::mat getKalmanGain() const;

        /// \brief Get the current estimated landmark positions in the map
        /// \return a 2 xN matrix of the current landmark positions seen.
        std::vector<turtlelib::Point2D> getLandmarkPositions() const;

        /// \brief Return the number of landmarks currently in the map:
        /// \return The number of landmarks currently in the map
        int getNumLandmarks() const;

        /// Feature Association:
        /// \brief Function to associate a measurement to a landmark id to be used in the updateMeasurementModelStep and updateEKF step.
        /// \param The measurement vector holding the range and bearing to the observed landmark
        /// \return The landmark id that is associated with the measurement taken, or -1 for a new landmark id if it is not associated with a previously seen landmark.
        int dataAssociation(const arma::colvec& measurement, double threshold = 0.1);

    private:
        // Combined state vector (robot state + map state):
        arma::colvec combined_state_;

        // The robot is initialized at some robpot state and covariance (3+2N x 3+2N), where N is the number of landmarks in the map:
        // The initial guess covariance is initialized to zero (indicating that we are certain of its position at start)/
        // The initial guess covariance of the landmarks is diagonal with large values (indicating that we are uncertain of their positions at start).
        arma::mat Covariance_; // Combined covariance matrix (robot state + map state)

        // State transition models for the robot's motion: (A and H are recomputed at each step for EKF)
        arma::mat A_; // State transition matrix for the robot's motion, or the jacobian of the motion model with size: (3+2N) x (3+2N)
        arma::mat Q_; // Process noise covariance matrix for the robot's motion with size (3 x 3) as the process noise only affects the robot state, not the landmarks
        arma::mat H_; // Measurement model matrix, recomputed each update based on which landmarks are observed with size (2 x (3 + 2N))
        arma::mat R_; // Measurement noise covariance matrix for the robot's sensors

        // Initialize the Kalman Gain matrix:
        arma::mat K_; // Kalman Gain matrix with size (3+2N x 2) to map the error in the measurement space back to the combined state space.

        // Track which landmarks have been seen/initialized
        std::vector<bool> landmark_initialized_;

        // Number of landmarks expected:
        int n_landmarks_;
    };
}
#endif // SLAMLIB_EKF_HPP_INCLUDE_GUARD