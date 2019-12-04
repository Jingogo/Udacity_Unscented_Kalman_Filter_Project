#include <iostream>
#include <iomanip>
#include "Eigen/Dense"
#include "UKF.h"
#include "tools.h"

UKF::UKF()
{
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  dim_measurement_radar_ = 3;
  state_mean_ = Eigen::VectorXd(n_x_);
  state_cov_ = Eigen::MatrixXd(n_x_, n_x_);
  pred_state_sig_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
  pred_measurement_sig_ = Eigen::MatrixXd(dim_measurement_radar_, 2 * n_aug_ + 1);
  previous_timestamp_ = 0;
  sensor_type_ = MeasurementPackage::LASER;
  dt_ = 0;
  std_acceleration = 30;
  std_yaw_acceleration = 3;
  std_px_laser_ = 0.15;
  std_py_laser_ = 0.15;
  std_rho_radar_ = 0.3;
  std_phi_radar_ = 0.03;
  std_phirate_radar_ = 0.3;
  lambda_ = 3 - n_x_;
  initWeights();
  process_noise_cov_ = Eigen::MatrixXd(2, 2);
  process_noise_cov_ << pow(std_acceleration, 2), 0,
      0, pow(std_yaw_acceleration, 2);
  laser_measurement_matrix_ = Eigen::MatrixXd(2, 5);
  laser_measurement_matrix_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;
  laser_measurement_noise_cov_ = Eigen::MatrixXd(2, 2);
  laser_measurement_noise_cov_ << pow(std_px_laser_, 2), 0,
      0, pow(std_py_laser_, 2);
  radar_measurement_noise_cov_ = Eigen::MatrixXd(3, 3);
  radar_measurement_noise_cov_ << pow(std_rho_radar_, 2), 0, 0,
      0, pow(std_phi_radar_, 2), 0,
      0, 0, pow(std_phirate_radar_, 2);
}

UKF::~UKF() {}

void UKF::initWeights()
{
  weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  weights_.tail(2 * n_aug_).setConstant(1.0 / (2 * (lambda_ + n_aug_)));
}

void UKF::processMeasurement(const MeasurementPackage &meas_package)
{
  readMeasurementPackage(meas_package);
  if (!isInitialized())
  {
    initUKF();
  }
  else
  {
    predictState();
    updateState();
  }
}

void UKF::readMeasurementPackage(const MeasurementPackage &measurement_pack)
{
  measurement_ = measurement_pack.raw_measurements_;
  sensor_type_ = measurement_pack.sensor_type_;
  long long new_timestamp = measurement_pack.timestamp_;
  dt_ = (new_timestamp - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = new_timestamp;
}

void UKF::initUKF()
{
  initStateMean();
  initStateCov();
  is_initialized_ = true;
}

void UKF::initStateMean()
{
  double px, py;
  if (isRadar())
  {
    const double rho = measurement_[0];
    const double theta = measurement_[1];
    px = rho * cos(theta);
    py = rho * sin(theta);
  }
  else if (isLaser())
  {
    px = measurement_[0];
    py = measurement_[1];
  }
  state_mean_ << px, py, 0, 0, 0;
}

void UKF::initStateCov()
{
  state_cov_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;
}

void UKF::predictState()
{
  Eigen::MatrixXd state_sig = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
  state_sig = generateAugmentedStateSig();
  updatePredStateSig(state_sig);
  updatePredStateMean();
  updatePredStateCov();
}

Eigen::MatrixXd UKF::generateAugmentedStateSig()
{
  Eigen::MatrixXd state_sig = Eigen::MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  Eigen::VectorXd aug_state_mean = Eigen::VectorXd::Zero(n_aug_);
  Eigen::MatrixXd aug_state_cov = Eigen::MatrixXd::Zero(n_aug_, n_aug_);

  aug_state_mean.head(n_x_) = state_mean_;
  aug_state_cov.block(0, 0, n_x_, n_x_) = state_cov_;
  aug_state_cov.block(n_x_, n_x_, 2, 2) = process_noise_cov_;
  Eigen::MatrixXd A = aug_state_cov.llt().matrixL();
  A = sqrt(lambda_ + n_aug_) * A;
  state_sig.col(0) = aug_state_mean;
  state_sig.block(0, 1, n_aug_, n_aug_) = A.colwise() + aug_state_mean;
  state_sig.block(0, n_aug_ + 1, n_aug_, n_aug_) = (-A).colwise() + aug_state_mean;

  return state_sig;
}

void UKF::updatePredStateSig(const Eigen::MatrixXd &state_sig)
{
  double eps = 1e-5;
  Eigen::VectorXd px = state_sig.row(0);
  Eigen::VectorXd py = state_sig.row(1);
  Eigen::VectorXd v = state_sig.row(2);
  Eigen::VectorXd yaw = state_sig.row(3);
  Eigen::VectorXd yaw_rate = state_sig.row(4);
  Eigen::VectorXd acceleration_noise = state_sig.row(5);
  Eigen::VectorXd yaw_acceleration_noise = state_sig.row(6);

  pred_state_sig_.row(2) = v + dt_ * acceleration_noise;
  pred_state_sig_.row(3) = yaw + dt_ * yaw_rate + 0.5 * dt_ * dt_ * yaw_acceleration_noise;
  pred_state_sig_.row(4) = yaw_rate + dt_ * yaw_acceleration_noise;
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    if (abs(yaw_rate(i)) < eps)
    {
      pred_state_sig_(0, i) = px(i) + v(i) * cos(yaw(i)) * dt_;
      pred_state_sig_(1, i) = py(i) + v(i) * sin(yaw(i)) * dt_;
    }
    else
    {
      pred_state_sig_(0, i) = px(i) + (v(i) / yaw_rate(i)) * (sin(yaw(i) + dt_ * yaw_rate(i)) - sin(yaw(i)));
      pred_state_sig_(1, i) = py(i) + (v(i) / yaw_rate(i)) * (cos(yaw(i)) - cos(yaw(i) + dt_ * yaw_rate(i)));
    }
    pred_state_sig_(0, i) = pred_state_sig_(0, i) + 0.5 * dt_ * dt_ * cos(yaw(i)) * acceleration_noise(i);
    pred_state_sig_(1, i) = pred_state_sig_(1, i) + 0.5 * dt_ * dt_ * sin(yaw(i)) * acceleration_noise(i);
  }
}

void UKF::updatePredStateMean()
{
  state_mean_ = pred_state_sig_ * weights_;
}

void UKF::updatePredStateCov()
{
  // cout << "##During updatePredStateCov" << endl;
  Eigen::ArrayXXd pred_state_error = pred_state_sig_.colwise() - state_mean_;
  Eigen::MatrixXd weighted_pred_state_error = pred_state_error.rowwise() * weights_.transpose().array();
  state_cov_ = weighted_pred_state_error * pred_state_error.matrix().transpose();
}

void UKF::updateState()
{
  if (isRadar())
  {
    updateStateByRadar();
  }
  else if (isLaser())
  {
    updateStateByLaser();
  }
}

void UKF::updateStateByRadar()
{
  predictRadarMeasurement();
  updateKalmanGainRadar();
  Eigen::VectorXd measurement_error = measurement_ - pred_measurement_mean_;
  tools::normalizeRadarMeasurement(measurement_error);
  state_mean_ = state_mean_ + kalman_gain_ * measurement_error;
  state_cov_ = state_cov_ - kalman_gain_ * pred_measurement_cov_ * kalman_gain_.transpose();
}

void UKF::updateStateByLaser()
{
  predictLaserMeasurement();
  updateKalmanGainLaser();
  Eigen::VectorXd measurement_error = measurement_ - pred_measurement_mean_;
  state_mean_ = state_mean_ + kalman_gain_ * measurement_error;
  state_cov_ = state_cov_ - kalman_gain_ * pred_measurement_cov_ * kalman_gain_.transpose();
}

void UKF::predictRadarMeasurement()
{
  updatePredMeasurementSig();
  updatePredRadarMeasurementMean();
  updatePredRadarMeasurementCov();
}

void UKF::updatePredMeasurementSig()
{
  double eps = 1e-5;
  Eigen::ArrayXd px = pred_state_sig_.row(0);
  Eigen::ArrayXd py = pred_state_sig_.row(1);
  Eigen::ArrayXd v = pred_state_sig_.row(2);
  Eigen::ArrayXd yaw = pred_state_sig_.row(3);
  Eigen::ArrayXd yaw_rate = pred_state_sig_.row(4);
  for (int i = 0; i < (2 * n_aug_ + 1); i++)
  {
    if (abs(px(i)) < eps)
    {
      tools::addEpsIfZero(py(i));
    }
    pred_measurement_sig_(1, i) = atan2(py(i), px(i));
  }

  Eigen::ArrayXd rho = (px * px + py * py).sqrt();
  pred_measurement_sig_.row(0) = rho;
  pred_measurement_sig_.row(2) = (px * v * yaw.cos() + py * v * yaw.sin()) / rho;
}

void UKF::updatePredRadarMeasurementMean()
{
  pred_measurement_mean_ = pred_measurement_sig_ * weights_;
}

void UKF::updatePredRadarMeasurementCov()
{
  Eigen::ArrayXXd error = pred_measurement_sig_.colwise() - pred_measurement_mean_;
  Eigen::MatrixXd weighted_error = error.rowwise() * weights_.transpose().array();
  pred_measurement_cov_ = weighted_error * error.matrix().transpose() + radar_measurement_noise_cov_;
}

void UKF::updateKalmanGainRadar()
{
  Eigen::ArrayXXd pred_state_error = pred_state_sig_.colwise() - state_mean_;
  Eigen::ArrayXXd pred_measurement_error = pred_measurement_sig_.colwise() - pred_measurement_mean_;
  Eigen::MatrixXd pred_weighted_state_error = pred_state_error.rowwise() * weights_.transpose().array();
  Eigen::MatrixXd pred_state_measurement_cov = pred_weighted_state_error * pred_measurement_error.matrix().transpose();

  kalman_gain_ = pred_state_measurement_cov * pred_measurement_cov_.inverse();
}

void UKF::predictLaserMeasurement()
{
  updatePredLaserMeasurementMean();
  updatePredLaserMeasurementCov();
}

void UKF::updatePredLaserMeasurementMean()
{
  pred_measurement_mean_ = laser_measurement_matrix_ * state_mean_;
}

void UKF::updatePredLaserMeasurementCov()
{
  pred_measurement_cov_ = laser_measurement_matrix_ * state_cov_ * laser_measurement_matrix_.transpose() + laser_measurement_noise_cov_;
}

void UKF::updateKalmanGainLaser()
{
  kalman_gain_ = state_cov_ * laser_measurement_matrix_.transpose() * pred_measurement_cov_.inverse();
}

Eigen::VectorXd UKF::getStateMean()
{
  return state_mean_;
}

