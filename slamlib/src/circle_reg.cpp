#include "slamlib/circle_reg.hpp"

namespace slamlib
{
    // Define the constructor:
CircleReg::CircleReg(std::vector<turtlelib::Point2D> points)
{
        // Store the actual points:
  read_points_ = points;

        // Construct the data matrix Z from the given points:
  Z_ = arma::zeros(points.size(), 4);

        // Calculate the Moment Matrix: (1/n * Z_T * Z)
  M_ = (1.0 / points.size()) * (Z_.t() * Z_);

        // Initialize the constraint matrix H (will set H_(0,0) when z_bar is calculated)
  H_ = arma::zeros(4, 4);
  H_(1, 1) = 1;
  H_(2, 2) = 1;
  H_(3, 0) = 2;
  H_(0, 3) = 2;

        // Initialize the inverse constraint matrix H_T_ (will set H_T_(3,3) when z_bar is calculated)
  H_T_ = arma::zeros(4, 4);
  H_T_(1, 1) = 1;
  H_T_(2, 2) = 1;
  H_T_(3, 0) = 0.5;
  H_T_(0, 3) = 0.5;
}

    // Define ComputeCentroid:
turtlelib::Point2D CircleReg::computeCentroid(std::vector<turtlelib::Point2D> points)
{
        // Compute the centroid of the points in the cluster as the mean of the x and y:
  auto sum_x = 0.0, sum_y = 0.0;
  auto num_points = points.size();
        // Loop through each point in the cluster and sum the x and y values:
  for (const auto & point : points) {
    sum_x += point.x;
    sum_y += point.y;
  }
        // Return the centroid as a Point2D:
  return turtlelib::Point2D(sum_x / num_points, sum_y / num_points);
}

    // Define shiftPoints()
std::vector<turtlelib::Point2D> CircleReg::shiftPoints(turtlelib::Point2D centroid)
{
        // Compute the vector of shifted points to the origin:
  std::vector<turtlelib::Point2D> shifted_points;

        // Centroid x and y:
  auto cx = centroid.x;
  auto cy = centroid.y;

  for(size_t i = 0; i < read_points_.size(); i++) {
    turtlelib::Point2D shifted_point;
    shifted_point.x = read_points_.at(i).x - cx;
    shifted_point.y = read_points_.at(i).y - cy;
            // Add the shifted point to the vector of shifted points:
    shifted_points.push_back(shifted_point);
  }

        // Update the data matrix Z_ with the shifted points:
  for (size_t i = 0; i < shifted_points.size(); i++) {
    auto x_i = shifted_points.at(i).x;
    auto y_i = shifted_points.at(i).y;
    Z_(i, 0) = (x_i * x_i) + (y_i * y_i);
    Z_(i, 1) = x_i;
    Z_(i, 2) = y_i;
    Z_(i, 3) = 1;
  }

        // Update the moment matrix M_ with the shifted points:
  M_ = (1.0 / shifted_points.size()) * (Z_.t() * Z_);

        // Return the shifted points as a vector of Point2D:
  return shifted_points;
}

    // Define ComputeRadius:
double CircleReg::computeZ_bar(std::vector<turtlelib::Point2D> shifted_points)
{
        // Initialize the radius to be caluclated for the cluster:
  auto z = 0.0;

  for(size_t i = 0; i < shifted_points.size(); i++) {
    auto x_i = shifted_points.at(i).x;
    auto y_i = shifted_points.at(i).y;

    z += (x_i * x_i) + (y_i * y_i);
  }

        // Get the mean readius:
  auto cluster_z_bar = z / shifted_points.size();

        // Return the cluster radius:
  return cluster_z_bar;
}

void CircleReg::updateConstraitMatrices(double cluster_z_bar)
{
        // Update H_Matrix:
  H_(0, 0) = 8 * cluster_z_bar;
  H_T_(3, 3) = -2.0 * cluster_z_bar;
}

    // Compute the Circle Parameters a and b for the equation of the circle:
std::pair<turtlelib::Point2D, double> CircleReg::computeCircleParams(turtlelib::Point2D centroid)
{
        // #################################### Begin_Citation [16] ######################################
        // Initialize the circle parameters:
  auto a = 0.0;
  auto b = 0.0;
  auto R = 0.0;       // Radius of the circle from algorithm.

        // Initialize the SVD Function Matrices:
  arma::mat U;          // Left singular vectors (m x k)
  arma::mat V;          // Right singular vectors (n x k)
  arma::vec s;          // Singular values (k x 1)

        // Z_ = U * diagmat(s) * V.t() is the SVD of Z_:
  arma::svd(U, s, V, Z_, "std");

        // Initialize the matrix Y and A to solve for the circle parameters:
  arma::mat Y;          // Product of the V * eigen(s) * V_T
  arma::mat A;          // The solution to the SVD problem that will give us the circle parameters a and b.
  arma::mat Q;
  arma::vec A_star;         // The solution to the generalized eigenvalue problem where it is the smallest positive eighenvalue of Q

        // If sigma4 is less than 10-12, we let A be the fourth column of V.
  if(s(3) < 1e-12) {
    A = V.col(3);
  } else {
            // Otherwise, we solve the generalized eigenvalue problem to find A:
    Y = V * arma::diagmat(s) * V.t();
    Q = Y * H_T_ * Y;

            // Find the smallest positive eigenvalue of Q and its corresponding eigenvector:
    arma::vec eigval;
    arma::mat eigvec;
    arma::eig_sym(eigval, eigvec, Q);

            // eigval is already sorted in ascending order, so just find the first positive:
    for (size_t i = 0; i < eigval.n_elem; i++) {
      if (eigval(i) > 0) {
        A_star = eigvec.col(i);              // Set the smallest positive eigenvalue.
        break;
      }
    }
            // Compute A if A_star is not empty:
    if(A_star.n_elem == 0) {
                // Use A_star as 0 if there are no positive eigenvalues to avoid issues with arma::solve:
      A_star = arma::zeros(4);
    } else {
      A = arma::solve(Y, A_star);
    }
  }

        // Now that we have a, we can caluclate for the equaction of the circle:
  a = -A.at(1) / (2 * A.at(0));
  b = -A.at(2) / (2 * A.at(0));
  R = std::sqrt((A.at(1) * A.at(1) + A.at(2) * A.at(2) - 4 * A.at(0) * A.at(3)) /
      (4 * A.at(0) * A.at(0)));

        // Reshift a and b, since we shifted the coordinate system to the origin before fitting:
  auto cx = centroid.x;
  auto cy = centroid.y;
  a = a + cx;
  b = b + cy;

        // Return the circle parameters as a point for easy extraction
  return {turtlelib::Point2D(a, b), R};
        // #################################### End_Citation [16] ######################################
}

bool CircleReg::isCircle()
{
        // Classification to determine if the points in a cluster are classified as a circle or not:
  turtlelib::Point2D P1 = read_points_.front();
  turtlelib::Point2D P2 = read_points_.back();

        // Loop through each inner point and calculate the angle from P1 to P and from P to P2, and sum the angles, to calculate mean and std of the angles:
  std::vector<double> angles;
  for(size_t i = 1; i < read_points_.size() - 1; i++) {
    turtlelib::Point2D P = read_points_.at(i);
            // Get the vectors from P to P1 and from P2 to P:
    turtlelib::Vector2D v1 = P1 - P;
    turtlelib::Vector2D v2 = P2 - P;

            // Get the smallest angle between the two vectors:
    auto angle_diff = turtlelib::angle(v1, v2);
    angles.push_back(angle_diff);
  }
        // Calculate mean and standard deviation of the angles:
  double mean_angle = std::accumulate(angles.begin(), angles.end(), 0.0) / angles.size();
  double variance_angle = 0.0;
  for(const auto & angle : angles) {
    variance_angle += std::pow(angle - mean_angle, 2);
  }
        // Calculate the std of the angles:
  variance_angle /= angles.size();
  double std_angle = std::sqrt(variance_angle);

        // Classification logic from the paper:
  if((mean_angle >= turtlelib::deg2rad(90) && mean_angle <= turtlelib::deg2rad(135)) &&
    std_angle <= 0.15)
  {
    return true;
  } else {
    return false;
  }
}

std::tuple<turtlelib::Point2D, double, bool> CircleReg::fitCircle()
{
        // Compute the centroid of the points in the cluster:
  auto centroid = computeCentroid(read_points_);

        // Shift the points so that the centroid is at the origin:
  auto shifted_points = shiftPoints(centroid);

        // Compute the radius of the circle that best fits the given points:
  auto cluster_z_bar = computeZ_bar(shifted_points);

        // Update the constraint matrices based on the calculated radius:
  updateConstraitMatrices(cluster_z_bar);

        // Compute the circle parameters a and b that best fit the given points:
  auto [circle_center, circle_radius] = computeCircleParams(centroid);

        // Determine if the fitted circle is a circle or not based on the classification model:
  bool is_circle = this->isCircle();

        // Return the circle parameters a and b as a Point2D for easy extraction:
  return {circle_center, circle_radius, is_circle};
}
} // End of namespace slamlib
