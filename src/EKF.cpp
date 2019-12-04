#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "EKF.h"
#include "tools.h"

EKF::EKF(Eigen::VectorXd processNoise, Eigen::MatrixXd RadarMeasurementNoiseCov, Eigen::MatrixXd laserMeasurementNoiseCov):
         processNoise_(processNoise),
         radarMeasurementNoiseCov_(RadarMeasurementNoiseCov),
         laserMeasurementNoiseCov_(laserMeasurementNoiseCov)
{
  laserMeasurementMatrix_ = Eigen::MatrixXd(2, 4);
  laserMeasurementMatrix_ << 1.0, 0, 0, 0,
      0, 1.0, 0, 0;
};

EKF::~EKF() {}

void EKF::processMeasurement(const MeasurementPackage &measurementPack)
{
  readMeasurementPackage(measurementPack);
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

void EKF::readMeasurementPackage(const MeasurementPackage &measurementPack)
{
  measurement_ = measurementPack.rawMeasurements_;
  sensorType_ = measurementPack.sensorType_;
  long long newTimestamp = measurementPack.timestamp_;
  dt_ = (newTimestamp - previousTimestamp_) / 1000000.0;
  previousTimestamp_ = newTimestamp;
}

void EKF::initEKF()
{
  initStateMean();
  initStateCov();
  initializationFlag = true;
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
  stateMean_ = Eigen::VectorXd(4);
  stateMean_ << px, py, 0, 0;
}

void EKF::initStateCov()
{
  stateCov_ = Eigen::MatrixXd(4, 4);
  stateCov_ << 1., 0, 0, 0,
      0, 1., 0, 0,
      0, 0, 1000., 0,
      0, 0, 0, 1000.;
}

void EKF::predictState()
{
  updateStateTransitionMatrix();
  updateProcessNoiseCov();
  stateMean_ = stateTransitionMatrix_ * stateMean_;
  stateCov_ = stateTransitionMatrix_ * stateCov_ * stateTransitionMatrix_.transpose() + processNoiseCov_;
}

void EKF::updateStateTransitionMatrix()
{
  stateTransitionMatrix_ = Eigen::MatrixXd(4, 4);
  stateTransitionMatrix_ << 1, 0, dt_, 0,
      0, 1, 0, dt_,
      0, 0, 1, 0,
      0, 0, 0, 1;
}

void EKF::updateProcessNoiseCov()
{
  const double dt_2 = dt_ * dt_;
  const double dt_3 = dt_2 * dt_;
  const double dt_4 = dt_3 * dt_;
  double noise_ax = processNoise_(0);
  double noise_ay = processNoise_(1);
  processNoiseCov_ = Eigen::MatrixXd(4, 4);
  processNoiseCov_ << dt_4 * noise_ax / 4, 0, dt_3 * noise_ax / 2, 0,
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
  Eigen::MatrixXd Ht = radarMeasurementMatrix_.transpose();
  Eigen::MatrixXd S = (radarMeasurementMatrix_ * stateCov_) * Ht + radarMeasurementNoiseCov_;
  Eigen::MatrixXd kalman_gain = stateCov_ * Ht * S.inverse();
  Eigen::VectorXd pred_measurement = predictMeasurement();
  Eigen::VectorXd measurement_diff = measurement_ - pred_measurement;
  tools::normalizeRadarMeasurement(measurement_diff);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  stateMean_ = stateMean_ + kalman_gain * measurement_diff;
  stateCov_ = (I - kalman_gain * radarMeasurementMatrix_) * stateCov_;
}

void EKF::updateStateByLaser()
{
  Eigen::MatrixXd Ht = laserMeasurementMatrix_.transpose();
  Eigen::MatrixXd S = (laserMeasurementMatrix_ * stateCov_) * Ht + laserMeasurementNoiseCov_;
  Eigen::MatrixXd kalman_gain = stateCov_ * Ht * S.inverse();
  Eigen::VectorXd pred_measurement = laserMeasurementMatrix_ * stateMean_;
  Eigen::VectorXd measurement_diff = measurement_ - pred_measurement;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4, 4);
  stateMean_ = stateMean_ + kalman_gain * measurement_diff;
  stateCov_ = (I - kalman_gain * laserMeasurementMatrix_) * stateCov_;
}

void EKF::updateRadarMeasurementMatrix()
{
  radarMeasurementMatrix_ = calculateJacobian();
}

Eigen::MatrixXd EKF::calculateJacobian()
{
  double px = stateMean_[0];
  double py = stateMean_[1];
  double vx = stateMean_[2];
  double vy = stateMean_[3];
  double z = std::pow(px, 2) + std::pow(py, 2);
  tools::addEpsIfZero(z);
  double a = (vx * py - vy * px) / std::pow(z, 1.5);
  Eigen::MatrixXd jacobian(3,4); 
  jacobian << px / sqrt(z), py / sqrt(z), 0, 0,
      -py / z, px / z, 0, 0,
      py * a, -px * a, px / sqrt(z), py / sqrt(z);

  return jacobian;
}

Eigen::VectorXd EKF::predictMeasurement()
{
  double px = stateMean_(0);
  double py = stateMean_(1);
  double vx = stateMean_(2);
  double vy = stateMean_(3);
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
  return stateMean_;
}
