#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "slamlib/ekf.hpp"
#include <vector>
#include <armadillo>

using Catch::Matchers::WithinRel;
constexpr double EPS = 0.001;

TEST_CASE("Data Association Test", "[EKF]") {
    // Create an instance of the EKF class with no known landmarks:
    slamlib::EKF ekf(0, 0.01, 0.01);

    // Create a measurement vector for a landmark at (1,1) that the robot is observing for the first time:
    arma::colvec measurement(2);
    measurement.at(0) = std::sqrt(2); // range to the landmark
    measurement.at(1) = std::atan2(1, 1); // bearing to the landmark

    // Call the data association function with this measurement and a threshold:
    int landmark_id_1 = ekf.dataAssociation(measurement, 0.1);

    // Verify that the returned landmark id is -1, indicating that this is a new landmark:
    REQUIRE(landmark_id_1 == -1);

    // Initialize the landmark into the EKF state
    ekf.updateEKF(measurement, 0);

    // Second observation — same landmark with noise, should match landmark 0
    arma::colvec measurement_noisy(2);
    measurement_noisy.at(0) = std::sqrt(2) + 0.05;
    measurement_noisy.at(1) = std::atan2(1, 1) + 0.05;

    int landmark_id_2 = ekf.dataAssociation(measurement_noisy, 0.1);
    REQUIRE(landmark_id_2 == 0);

    ekf.updateEKF(measurement_noisy, landmark_id_2);

    // Third observation — genuinely new landmark in a different direction
    arma::colvec measurement_new2(2);
    measurement_new2.at(0) = std::sqrt(2);
    measurement_new2.at(1) = std::atan2(1, -1); // different direction

    int landmark_id_3 = ekf.dataAssociation(measurement_new2, 0.1);
    REQUIRE(landmark_id_3 == -1);
}