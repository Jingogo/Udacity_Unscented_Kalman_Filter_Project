#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "EKF.h"
#include "tools.h"

EKF::EKF(Eigen::MatrixXd R_laser, Eigen::MatrixXd R_radar, Eigen::VectorXd noise):
         laser_measurement_noise_cov_(R_laser),
         radar_measurement_noise_cov_(R_radar),
         noise_(noise)
{
  state_mean_ = Eigen::VectorXd(4);
  state_cov_ = Eigen::MatrixXd(4, 4);
  state_transition_matrix_ = Eigen::MatrixXd(4, 4);
  process_noise_cov_ = Eigen::MatrixXd(4, 4);
  laser_measurement_matrix_ = Eigen::MatrixXd(2, 4);
  laser_measurement_matrix_ << 1.0, 0, 0, 0,
      0, 1.0, 0, 0;
  radar_measurement_matrix_ = Eigen::MatrixXd(3, 4);
};

EKF::~EKF() {}

void EKF::processMeasurement(const MeasurementPackage &measurement_pack)
{
  readMeasurementPackage(measurement_pack);
  if (!isInitialized())
  {
    initEKF();
  }
  else
  {
    predictState();
    updateState();
  }
}

void EKF::readMeasurementPackage(const MeasurementPackage &measurement_pack)
{
  measurement_ = measurement_pack.raw_measurements_;
  sensor_type_ = measurement_pack.sensor_type_;
  long long new_timestamp = measurement_pack.timestamp_;
  dt_ = (new_timestamp - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = new_timestamp;
}

void EKF::initEKF()
{
  initStateMean();
  initStateCov();
  is_initialized_ = true;
}

void EKF::initStateMean()
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
  state_mean_ << px, py, 0, 0;
}

void EKF::initStateCov()
{
  state_cov_ << 1., 0, 0, 0,
      0, 1., 0, 0,
      0, 0, 1000., 0,
      0, 0, 0, 1000.;
}

void EKF::predictState()
{
  updateStateTransitionMatrix();
  updateProcessNoiseCov();
  state_mean_ = state_transition_matrix_ * state_mean_;
  state_cov_ = state_transition_matrix_ * state_cov_ * state_transition_matrix_.transpose() + process_noise_cov_;
}

void EKF::updateStateTransitionMatrix()
{
  state_transition_matrix_ << 1, 0, dt_, 0,
      0, 1, 0, dt_,
      0, 0, 1, 0,
      0, 0, 0, 1;
}

void EKF::updateProcessNoiseCov()
{
  const double dt_2 = dt_ * dt_;
  const double dt_3 = dt_2 * dt_;
  const double dt_4 = dt_3 * dt_;
  double noise_ax = noise_(0);
  double noise_ay = noise_(1);
  process_noise_cov_ << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
      0, dt_4 * noise_ay / 4, 0, dt_3 * noise_ay / 2,
      dt_3 * noise_ax / 2, 0, dt_2 * noise_ax, 0,
      0, dt_3 * noise_ay / 2, 0, dt_2 * noise_ay;
}

void EKF::updateState()
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

void EKF::updateStateByRadar()
{
  updateRadarMeasurementMatrix();
  Eigen::MatrixXd Ht = radar_measurement_matrix_.transpose();
  Eigen::MatrixXd S = (radar_measurement_matrix_ * state_cov_) * Ht + radar_measurement_noise_cov_;
  Eigen::MatrixXd kalman_gain = state_cov_ * Ht * S.inverse();
  Eigen::VectorXd pred_measurement = PredMeasurement();
  Eigen::VectorXd measurement_diff = measurement_ - pred_measurement;
  tools::normalizeRadarMeasurement(measurement_diff);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  state_mean_ = state_mean_ + kalman_gain * measurement_diff;
  state_cov_ = (I - kalman_gain * radar_measurement_matrix_) * state_cov_;
}

void EKF::updateStateByLaser()
{
  Eigen::MatrixXd Ht = laser_measurement_matrix_.transpose();
  Eigen::MatrixXd S = (laser_measurement_matrix_ * state_cov_) * Ht + laser_measurement_noise_cov_;
  Eigen::MatrixXd kalman_gain = state_cov_ * Ht * S.inverse();
  Eigen::VectorXd pred_measurement = laser_measurement_matrix_ * state_mean_;
  Eigen::VectorXd measurement_diff = measurement_ - pred_measurement;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  state_mean_ = state_mean_ + kalman_gain * measurement_diff;
  state_cov_ = (I - kalman_gain * laser_measurement_matrix_) * state_cov_;
}

void EKF::updateRadarMeasurementMatrix()
{
  radar_measurement_matrix_ = calculateJacobian();
}

Eigen::MatrixXd EKF::calculateJacobian()
{
  double px = state_mean_[0];
  double py = state_mean_[1];
  double vx = state_mean_[2];
  double vy = state_mean_[3];
  double z = std::pow(px, 2) + std::pow(py, 2);
  tools::addEpsIfZero(z);
  double a = (vx * py - vy * px) / std::pow(z, 1.5);
  Eigen::MatrixXd jacobian(3,4); 
  jacobian << px / sqrt(z), py / sqrt(z), 0, 0,
      -py / z, px / z, 0, 0,
      py * a, -px * a, px / sqrt(z), py / sqrt(z);

  return jacobian;
}

Eigen::VectorXd EKF::PredMeasurement()
{
  double px = state_mean_(0);
  double py = state_mean_(1);
  double vx = state_mean_(2);
  double vy = state_mean_(3);
  double z = std::pow(px, 2) + std::pow(py, 2);
  tools::addEpsIfZero(z);
  double rho = std::sqrt(z);
  double theta = std::atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;

  Eigen::VectorXd pred_measurement(3);
  pred_measurement << rho, theta, rho_dot;

  return pred_measurement;
}

Eigen::VectorXd EKF::getStateMean()
{
  return state_mean_;
}
