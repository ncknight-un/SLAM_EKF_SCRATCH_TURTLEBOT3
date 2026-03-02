#include "slamlib/ekf.hpp"

namespace slamlib {
    EKF::EKF(int num_landmarks, double process_noise, double measurement_noise) {
        // Initialize the expected number of landmarks
        n_landmarks_ = num_landmarks;

        // Initialize which landmarks have been seen:
        landmark_initialized_ = std::vector<bool>(n_landmarks_, false);

        // Initialize the combined state vector:
        combined_state_ = arma::zeros(3 + 2 * n_landmarks_); 
        combined_state_.subvec(0, 2) = arma::colvec(3, arma::fill::zeros); // Set the robot state to (0,0,0) at start

        // (Equation 19 in tmp/slam_EKF.pdf) Initialize the covariance matrix for the robot state and the obstacles, (3+2N x 3+2N)
        // with large values for uncertainty for obstackes and small values for the robot state (since we are certain of its initial position)
        Covariance_ = arma::eye(3 + 2 * n_landmarks_, 3 + 2 * n_landmarks_); // Initialize full covariance matrix
        // Set landmark uncertainty high:
        Covariance_.submat(3, 3, 3 + 2 * n_landmarks_ - 1, 3 + 2 * n_landmarks_ - 1) = arma::eye(2 * n_landmarks_, 2 * n_landmarks_) * 1e6;
        // Set robot state uncertainty low:
        Covariance_.submat(0, 0, 2, 2) = arma::eye(3, 3) * 1e-6;

        // Initialize the transition and measurement models:
        Q_ = arma::eye(3, 3) * process_noise; // Process Noise Covariance for motion (I assume process noise is the same for x, y, and theta for simplicity)
        R_ = arma::eye(2, 2) * measurement_noise; // Measurement Noise Covariance for lidar sensor
        // Model recomputed each step:
        A_ = arma::eye(3 + 2 * n_landmarks_, 3 + 2 * n_landmarks_); // Will be recomputed each predict step based on the control input
        H_ = arma::zeros(2, 3 + 2 * n_landmarks_); // Will be recomputed each update step based on which landmarks are observed in the function updateMeasurementModel() - Set to 0 at start as non measured landmarks have not been measured.

        // Initialize the Kalman Gain matrix
        K_ = arma::zeros(3 + 2 * n_landmarks_, 2);
    }

    EKF::EKF(const arma::colvec& initial_state, int num_landmarks, double process_noise, double measurement_noise) {
        // Make sure the constructed initial state is the correct size:
        if (initial_state.n_rows != 3) {
            throw std::invalid_argument("Initial state must be a 3x1 vector (theta, x, y)");
        }
        // Initialize the expected number of landmarks
        n_landmarks_ = num_landmarks;

        // Initialize which landmarks have been seen:
        landmark_initialized_ = std::vector<bool>(n_landmarks_, false);

        // Initialize the combined state vector:
        combined_state_ = arma::zeros(3 + 2 * n_landmarks_); 
        combined_state_.subvec(0, 2) = initial_state; // Set the robot state since it is known at start

        // (Equation 19 in tmp/slam_EKF.pdf) Initialize the covariance matrix for the robot state and the obstacles, (3+2N x 3+2N)
        // with large values for uncertainty for obstackes and small values for the robot state (since we are certain of its initial position)
        Covariance_ = arma::eye(3 + 2 * n_landmarks_, 3 + 2 * n_landmarks_); // Initialize full covariance matrix
        // Set landmark uncertainty high:
        Covariance_.submat(3, 3, 3 + 2 * n_landmarks_ - 1, 3 + 2 * n_landmarks_ - 1) = arma::eye(2 * n_landmarks_, 2 * n_landmarks_) * 1e6;
        // Set robot state uncertainty low:
        Covariance_.submat(0, 0, 2, 2) = arma::eye(3, 3) * 1e-6;

        // Initialize the transition and measurement models:
        Q_ = arma::eye(3, 3) * process_noise; // Process Noise Covariance for motion
        R_ = arma::eye(2, 2) * measurement_noise; // Measurement Noise Covariance for lidar sensor
        // Model recomputed each step:
        A_ = arma::eye(3 + 2 * n_landmarks_, 3 + 2 * n_landmarks_); // Will be recomputed each predict step based on the control input
        H_ = arma::zeros(2, 3 + 2 * n_landmarks_); // Will be recomputed each update step based on which landmarks are observed

        // Initialize the Kalman Gain matrix:
        K_ = arma::zeros(3 + 2 * n_landmarks_, 2);
    }

    void EKF::updateMeasurementModelMatrix(int landmark_id) {
        // Function to update H matrix using Equation 18 in tmp/slam_EKF.pdf based on which landmark is being observed:
        // Make sure that H has been re-initialized to zero at the start of the update:
        H_ = arma::zeros(2, 3 + 2 * n_landmarks_);

        // Get dx and dy for the observed landmark:
        auto dx = combined_state_(3 + 2 * landmark_id) - combined_state_(1);         // m_x - robot_x (Equation 16 in tmp/slam_EKF.pdf)
        auto dy = combined_state_(3 + 2 * landmark_id + 1) - combined_state_(2);     // m_y - robot_y (Equation 17 in tmp/slam_EKF.pdf)
        auto d = dx * dx + dy * dy;

        // Set the robot State columns in Equation 18 in tmp/slam_EKF.pdf:
        H_(0, 0) =  0;
        H_(0, 1) = -dx / std::sqrt(d);
        H_(0, 2) = -dy / std::sqrt(d);
        H_(1, 0) = -1;
        H_(1, 1) =  dy / d;
        H_(1, 2) = -dx / d;

        // Set the columns corresponding to the landmark_id:
        H_(0, (3 + 2 * landmark_id)) = dx / std::sqrt(d);
        H_(0, (3 + 2 * landmark_id) + 1) = dy / std::sqrt(d);
        H_(1, (3 + 2 * landmark_id)) = -dy / d;
        H_(1, (3 + 2 * landmark_id) + 1) = dx / d;
    }

    void EKF::predict(const turtlelib::Twist2D& control_input) {
        // Build the State Transition Matrix A based on the control input:
        auto theta = combined_state_(0);
        auto v = control_input.x;
        auto omega = control_input.omega;

        // Update the Estimate using the function g() in Equations 20 in tmp/slam_EKF.pdf:
        arma::colvec state_pred(3);
        if (std::abs(omega) > 1e-6) { // State change has rotation (Equation 7 in tmp/slam_EKF.pdf):
            state_pred(0) = theta + omega;
            state_pred(1) = combined_state_(1) + (v / omega) * (std::sin(state_pred(0)) - std::sin(theta));
            state_pred(2) = combined_state_(2) - (v / omega) * (std::cos(state_pred(0)) - std::cos(theta));
        } else { // State change does not have rotation (linear motion) Equation 5 in tmp/slam_EKF.pdf:
            state_pred(0) = theta;
            state_pred(1) = combined_state_(1) + v * std::cos(theta);
            state_pred(2) = combined_state_(2) + v * std::sin(theta);
        }

        // ################################## Begin_Citation [15] ##################################
        // Source used to check if I was doing my noise propagation correctly, and determined I needed to wrap angle at predict step:
        // Wrap the predicted angle to [-pi, pi]:
        state_pred(0) = turtlelib::normalize_angle(state_pred(0));
        // ################################## End_Citation [15] ################################## 

        // Update the estimated combined state vector for the robot state estimate:
        // Note: The landmark positions do not change during the predict step because we assume a static map.
        combined_state_.subvec(0, 2) = state_pred;

        // Propage the uncertainty using the linearized motion model (Equation 21 in tmp/slam_EKF.pdf):
        // Compute the Jacobian of the motion model (A matrix):
        if (std::abs(omega) > 1e-6) {   // See Equation 10 in tmp/slam_EKF.pdf for the Jacobian of the motion model with rotation
            // Note A_(0,0) does not change.
            A_(1, 0) = (v / omega) * (std::cos(combined_state_(0)) - std::cos(state_pred(0)));         //TODO: Normalize angles...
            A_(2, 0) = (v / omega) * (std::sin(combined_state_(0)) - std::sin(state_pred(0)));
        } else {    // See Equation 9 in tmp/slam_EKF.pdf for the Jacobian of the motion model without rotation
            A_(1, 0) = -v * std::sin(combined_state_(0));
            A_(2, 0) = v * std::cos(combined_state_(0));
        }

        // Build Q_bar - Equation 22 in tmp/slam_EKF.pdf:
        arma::mat Q_bar = arma::zeros(3 + 2 * n_landmarks_, 3 + 2 * n_landmarks_);
        Q_bar.submat(0, 0, 2, 2) = Q_; // Process noise only affects the robot state, not the landmarks

        // Update the covariance matrix - Equation 21 in tmp/slam_EKF.pdf:
        Covariance_ = A_ * Covariance_ * A_.t() + Q_bar;
    }

    void EKF::updateEKF(const arma::colvec& z, int landmark_id) {
        // Extract the range and bearing from the measurement vector:
        auto range = z(0);
        auto bearing = z(1);

        // Check to see if the landmark has been initialized in the combined state vector:
        // We already know the landmark id from the data association step done in ROS2 SLAM Node.
        if (!landmark_initialized_[landmark_id]) {
            // If not initialized, initialize the landmark position in the map state based on the measurement and current robot pose
            auto theta = combined_state_(0);
            auto x = combined_state_(1);
            auto y = combined_state_(2);

            // Get landmarks in Cartesian coordinates in the world frame using Equations 23 and 24 in tmp/slam_EKF.pdf:
            auto m_x = x + range * std::cos(theta + bearing);
            auto m_y = y + range * std::sin(theta + bearing);

            // Initialize the landmark position in the combined state vector:
            combined_state_(3 + 2 * landmark_id) = m_x;
            combined_state_(3 + 2 * landmark_id + 1) = m_y;

            // Mark the new landmark as initialized, so the EKF knows it has been seen:
            landmark_initialized_[landmark_id] = true; 
            return; // No update after init.
        }

        // Update the measurement model matrix H based on which landmark is being observed:
        updateMeasurementModelMatrix(landmark_id);

        // Compute the theoretical measurement (range and bearing), given the current state estimage, using Equations 14 & 25 in tmp/slam_EKF.pdf:
        arma::colvec z_hat(2);
        z_hat(0) = std::sqrt(std::pow(combined_state_(3 + 2 * landmark_id) - combined_state_(1),2)
                                        + std::pow(combined_state_(3 + 2 * landmark_id + 1) - combined_state_(2), 2));
        z_hat(1) = std::atan2(combined_state_(3 + 2 * landmark_id + 1) - combined_state_(2),
                            combined_state_(3 + 2 * landmark_id) - combined_state_(1)) - combined_state_(0);

        // Compute the Kalman Gain from the linearized measurement model (Equation 26 in tmp/slam_EKF.pdf):
        // Note: K will be a (3+2N x 2) to map the error in the measurement space back to the combined state space.
        K_ = Covariance_ * H_.t() * arma::inv(H_ * Covariance_ * H_.t() + R_);

        // Normalize the measurement error (z - z_hat) for the bearing component: 
        auto bearing_wrap = turtlelib::normalize_angle(z(1) - z_hat(1));
        arma::colvec z_norm(2);
        z_norm(0) = z(0) - z_hat(0);
        z_norm(1) = bearing_wrap;

        // Compute the posterior state update (Equation 27 in tmp/slam_EKF.pdf):
        combined_state_ = combined_state_ + K_ * z_norm;

        // Guarantee the robot state_ theta is wrapped to [-pi, pi]:
        combined_state_(0) = turtlelib::normalize_angle(combined_state_(0));
        
        // Compute the updated covariance matrix (Equation 28 in tmp/slam_EKF.pdf):
        Covariance_ = (arma::eye(3 + 2 * n_landmarks_, 3 + 2 * n_landmarks_) - K_ * H_) * Covariance_; 
    }

    turtlelib::Transform2D EKF::getState() const {
        return turtlelib::Transform2D(turtlelib::Vector2D(combined_state_(1), combined_state_(2)), combined_state_(0));
    }

    arma::colvec EKF::getCombinedState() const {
        return combined_state_;
    }

    arma::mat EKF::getCovariance() const {
        return Covariance_;
    }

    arma::mat EKF::getKalmanGain() const {
        return K_;
    }

    std::vector<turtlelib::Point2D> EKF::getLandmarkPositions() const {
        // Only return the lanfmarks/obstacles that have been seen:
        std::vector<turtlelib::Point2D> landmarks;
        for (int i = 0; i < n_landmarks_; i++) {
            if (landmark_initialized_[i])
            landmarks.push_back(turtlelib::Point2D(combined_state_(3 + 2 * i), combined_state_(3 + 2 * i + 1))); // mx, my for each landmark in the map for seen obstacles.
        }
        return landmarks;
    }
} // End of namespace nuslam