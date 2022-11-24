#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  n_x_ = 5;
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // The maximum acceleration of a car is about 6 m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // Not initialized at first
  is_initialized_ = false;

  // Include two process noise
  n_aug_ = n_x_ + 2;

  // Predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Lidar noise covariance matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
      0, std_laspy_*std_laspy_;  

  // Radar noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0, std_radrd_*std_radrd_;

  // Calculate weights for signma points
  lambda_ = 3 - n_aug_;
  double weight = 0.5/(lambda_ + n_aug_);
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(weight);
  weights_[0] = lambda_/(lambda_ + n_aug_);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  // Initial measurement
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      // Lidar measurement
      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];
      x_ << px, py, 0, 0, 0;
      P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
          0, std_laspy_ * std_laspy_, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;      
    }else{
      // Radar measurement
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      x_ << px, py, rho_dot, rho, rho_dot;
      P_ << std_radr_*std_radr_, 0, 0, 0, 0,
              0, std_radr_ * std_radr_, 0, 0, 0,
              0, 0, std_radrd_ * std_radrd_, 0, 0,
              0, 0, 0, std_radphi_ * std_radphi_, 0,
              0, 0, 0, 0, std_radphi_ * std_radphi_ * 1e-2;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // Elapsed time since the last measurement
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  // Update the timestamp
  time_us_ = meas_package.timestamp_;

  // Predic the state and error covariance through sigma points
  Prediction(dt);

  // Correct the state and error covariance using sensor measurements and sigma points
  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    // Process Lidar
    UpdateLidar(meas_package);
  }
  else{
    // Process Radar
    UpdateRadar(meas_package);   
  }    

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Form augmented vector/matrix
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) << std_a_*std_a_, 0,
                                      0, std_yawdd_*std_yawdd_;

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  MatrixXd L = P_aug.llt().matrixL(); // create square root matrix
  for (int i = 0; i < n_aug_; ++i){
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // Predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_next, py_next;

    // avoid division by zero
    if (fabs(yawd) > 1e-3){
        px_next = px + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_next = py + v/yawd * (-cos(yaw + yawd*delta_t) + cos(yaw));
    }else{
        px_next = px + v * cos(yaw) * delta_t;
        py_next = py + v * sin(yaw) * delta_t;;
    }

    double v_next = v;
    double yaw_next = yaw + yawd * delta_t;
    double yawd_next = yawd;

    // Add noise
    px_next = px_next + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_next = py_next + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_next = v_next + nu_a * delta_t;
    yaw_next = yaw_next + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_next = yawd_next + nu_yawdd * delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0,i) = px_next;
    Xsig_pred_(1,i) = py_next;
    Xsig_pred_(2,i) = v_next;
    Xsig_pred_(3,i) = yaw_next;
    Xsig_pred_(4,i) = yawd_next;
  }


  // Predicted state
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted error covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Normalize the yaw angle within -pi to +pi
    while (x_diff(3) > M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Measurements from lidar
  int n_meas = 2;
  VectorXd z_meas(n_meas);
  double px = meas_package.raw_measurements_[0];
  double py = meas_package.raw_measurements_[1];
  z_meas << px, py;

  // Transform sigma points into measurement space
  MatrixXd Zsig_pred = MatrixXd(n_meas, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    // Extract each parameter for better readability
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);

    // Measurement equation
    Zsig_pred(0,i) = px;
    Zsig_pred(1,i) = py;
  }  

   // Predicted measurement
  VectorXd z_pred(n_meas);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    z_pred = z_pred + weights_(i) * Zsig_pred.col(i);
  }

  // Measurement covariance matrix
  MatrixXd S = MatrixXd(n_meas, n_meas);
  S.fill(0.0);

  // Cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_meas);
  Tc.fill(0.0); 

  // Predicted error covariance matrix
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // Angle normalization
    while (x_diff(3) > M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;    

    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  S = S + R_lidar_;

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // Innovation
  VectorXd innovation = z_meas - z_pred;

  // Corrected state
  x_ = x_ + K * innovation;

  // Corrected error covariance matrix
  P_ = P_ - K * S * K.transpose();

  // Normalized Innovation Squared (NIS)
  // NIS follows the Chi squared distribution
  // For 2-dimensional measurement space, 95% of the NIS value should be below 5.991
  const double threshold = 5.991;
  double NIS_lidar_ = innovation.transpose() * S.inverse() * innovation;
  bool check_NIS;
  if (NIS_lidar_ < threshold){
    // If the filter is consistent, 95% of the cases should be here 
    check_NIS = true;
    std::cout << "Lidar: normalized innovation squared = " << NIS_lidar_ << ", which is within 95% line of " << threshold << std::endl;
  }else{
    check_NIS = false;
    std::cout << "Lidar: normalized innovation squared = " << NIS_lidar_ << ", which NOT within 95% line of " << threshold << std::endl;
  }
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // Measurements from radar
  int n_meas = 3;
  VectorXd z_meas(n_meas);
  double rho = meas_package.raw_measurements_[0];
  double phi = meas_package.raw_measurements_[1];
  double rho_dot = meas_package.raw_measurements_[2];
  z_meas << rho, phi, rho_dot; 

  // Transform sigma points into measurement space
  MatrixXd Zsig_pred = MatrixXd(n_meas, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    // Extract each parameter for better readability
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // Measurement equation
    Zsig_pred(0,i) = sqrt(px*px + py*py);  // r
    Zsig_pred(1,i) = atan2(py, px);  // phi
    Zsig_pred(2,i) = (px*v1 + py*v2) / sqrt(px*px + py*py);  // r_dot
  }

  // Predicted measurement
  VectorXd z_pred(n_meas);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    z_pred = z_pred + weights_(i) * Zsig_pred.col(i);
  }

  // Measurement covariance matrix
  MatrixXd S = MatrixXd(n_meas, n_meas);
  S.fill(0.0);

  // Cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_meas);
  Tc.fill(0.0);

  // Predicted error covariance matrix
  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;

    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    while (z_diff(1) > M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  S = S + R_radar_;

  // Kalman gain K
  MatrixXd K = Tc * S.inverse();

  // Innovation
  VectorXd innovation = z_meas - z_pred;
  while (innovation(1) > M_PI) innovation(1)-=2.*M_PI;
  while (innovation(1) < -M_PI) innovation(1)+=2.*M_PI;

  // Corrected state
  x_ = x_ + K * innovation;

  // Corrected error covariance matrix
  P_ = P_ - K * S * K.transpose();

  // Normalized Innovation Squared (NIS)
  // NIS follows the Chi squared distribution
  // For 2-dimensional measurement space, 95% of the NIS value should be below 7.815
  const double threshold = 7.815;
  double R_radar_ = innovation.transpose() * S.inverse() * innovation;
  bool check_NIS;
  if (R_radar_ < threshold){
    // If the filter is consistent, 95% of the cases should be here 
    check_NIS = true;
    std::cout << "Radar: normalized innovation squared = " << R_radar_ << ", which is within 95% line of " << threshold << std::endl;
  }else{
    check_NIS = false;
    std::cout << "Radar: normalized innovation squared = " << R_radar_ << ", which NOT within 95% line of " << threshold << std::endl;
  }
}