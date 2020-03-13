#include <iostream>
#include "Eigen/Dense"
#include "UKF.h"
#include "tools.h"

UKF::UKF(Eigen::MatrixXd processNoiseCov, Eigen::MatrixXd radarMeasurementNoiseCov, Eigen::MatrixXd laserMeasurementNoiseCov):
         processNoiseCov_(processNoiseCov),
         radarMeasurementNoiseCov_(radarMeasurementNoiseCov),
         laserMeasurementNoiseCov_(laserMeasurementNoiseCov)
{
  initWeights();
  laserMeasurementMatrix_ = Eigen::MatrixXd(2, 5);
  laserMeasurementMatrix_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0;
}

UKF::~UKF() {}

void UKF::initWeights()
{
  weights_ = Eigen::VectorXd(2 * augmentedStateDim_ + 1);
  weights_(0) = lambda_ / (lambda_ + augmentedStateDim_);
  weights_.tail(2 * augmentedStateDim_).setConstant(1.0 / (2 * (lambda_ + augmentedStateDim_)));
}

void UKF::processMeasurement(const MeasurementPackage &measurementPackage)
{
  readMeasurementPackage(measurementPackage);
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

void UKF::readMeasurementPackage(const MeasurementPackage &measurementPackage)
{
  measurement_ = measurementPackage.rawMeasurements_;
  sensorType_ = measurementPackage.sensorType_;
  long long newTimestamp = measurementPackage.timestamp_;
  dt_ = (newTimestamp - previousTimestamp_) / 1000000.0;
  previousTimestamp_ = newTimestamp;
}

void UKF::initUKF()
{
  initStateMean();
  initStateCov();
  initializationFlag = true;
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

  stateMean_ = Eigen::VectorXd(stateDim_);  
  stateMean_ << px, py, 0, 0, 0;
}

void UKF::initStateCov()
{
  stateCov_ = Eigen::MatrixXd(stateDim_, stateDim_);
  stateCov_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;
}

void UKF::predictState()
{
  Eigen::MatrixXd stateSig = Eigen::MatrixXd(augmentedStateDim_, 2 * augmentedStateDim_ + 1);
  stateSig = generateAugmentedStateSig();
  updatePredStateSig(stateSig);
  updatePredStateMean();
  updatePredStateCov();
}

Eigen::MatrixXd UKF::generateAugmentedStateSig()
{
  Eigen::MatrixXd stateSig = Eigen::MatrixXd::Zero(augmentedStateDim_, 2 * augmentedStateDim_ + 1);
  Eigen::VectorXd augmentedStateMean = Eigen::VectorXd::Zero(augmentedStateDim_);
  Eigen::MatrixXd augmentedStateCov = Eigen::MatrixXd::Zero(augmentedStateDim_, augmentedStateDim_);

  augmentedStateMean.head(stateDim_) = stateMean_;
  augmentedStateCov.block(0, 0, stateDim_, stateDim_) = stateCov_;
  augmentedStateCov.block(stateDim_, stateDim_, 2, 2) = processNoiseCov_;
  Eigen::MatrixXd A = augmentedStateCov.llt().matrixL();
  A = sqrt(lambda_ + augmentedStateDim_) * A;
  stateSig.col(0) = augmentedStateMean;
  stateSig.block(0, 1, augmentedStateDim_, augmentedStateDim_) = A.colwise() + augmentedStateMean;
  stateSig.block(0, augmentedStateDim_ + 1, augmentedStateDim_, augmentedStateDim_) = (-A).colwise() + augmentedStateMean;

  return stateSig;
}

void UKF::updatePredStateSig(const Eigen::MatrixXd &stateSig)
{
  double eps = 1e-5;
  Eigen::VectorXd px = stateSig.row(0);
  Eigen::VectorXd py = stateSig.row(1);
  Eigen::VectorXd v = stateSig.row(2);
  Eigen::VectorXd yaw = stateSig.row(3);
  Eigen::VectorXd yawRate = stateSig.row(4);
  Eigen::VectorXd accelerationNoise = stateSig.row(5);
  Eigen::VectorXd yawAccelerationNoise = stateSig.row(6);

  predictedStateSig_ = Eigen::MatrixXd(stateDim_, 2*augmentedStateDim_ + 1);
  predictedStateSig_.row(2) = v + dt_ * accelerationNoise;
  predictedStateSig_.row(3) = yaw + dt_ * yawRate + 0.5 * dt_ * dt_ * yawAccelerationNoise;
  predictedStateSig_.row(4) = yawRate + dt_ * yawAccelerationNoise;
  for (int i = 0; i < 2 * augmentedStateDim_ + 1; i++)
  {
    if (abs(yawRate(i)) < eps)
    {
      predictedStateSig_(0, i) = px(i) + v(i) * cos(yaw(i)) * dt_;
      predictedStateSig_(1, i) = py(i) + v(i) * sin(yaw(i)) * dt_;
    }
    else
    {
      predictedStateSig_(0, i) = px(i) + (v(i) / yawRate(i)) * (sin(yaw(i) + dt_ * yawRate(i)) - sin(yaw(i)));
      predictedStateSig_(1, i) = py(i) + (v(i) / yawRate(i)) * (cos(yaw(i)) - cos(yaw(i) + dt_ * yawRate(i)));
    }
    predictedStateSig_(0, i) = predictedStateSig_(0, i) + 0.5 * dt_ * dt_ * cos(yaw(i)) * accelerationNoise(i);
    predictedStateSig_(1, i) = predictedStateSig_(1, i) + 0.5 * dt_ * dt_ * sin(yaw(i)) * accelerationNoise(i);
  }
}

void UKF::updatePredStateMean()
{
  stateMean_ = predictedStateSig_ * weights_;
}

void UKF::updatePredStateCov()
{
  Eigen::ArrayXXd predictedStateError = predictedStateSig_.colwise() - stateMean_;
  Eigen::MatrixXd weightedPredictdStateError = predictedStateError.rowwise() * weights_.transpose().array();
  stateCov_ = weightedPredictdStateError * predictedStateError.matrix().transpose();
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
  Eigen::VectorXd measurementError = measurement_ - predictedMeasurementMean_;
  tools::normalizeRadarMeasurement(measurementError);
  stateMean_ = stateMean_ + kalmanGain_ * measurementError;
  stateCov_ = stateCov_ - kalmanGain_ * predictedMeasurementCov_ * kalmanGain_.transpose();
}

void UKF::updateStateByLaser()
{
  predictLaserMeasurement();
  updateKalmanGainLaser();
  Eigen::VectorXd measurementError = measurement_ - predictedMeasurementMean_;
  stateMean_ = stateMean_ + kalmanGain_ * measurementError;
  stateCov_ = stateCov_ - kalmanGain_ * predictedMeasurementCov_ * kalmanGain_.transpose();
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
  Eigen::ArrayXd px = predictedStateSig_.row(0);
  Eigen::ArrayXd py = predictedStateSig_.row(1);
  Eigen::ArrayXd v = predictedStateSig_.row(2);
  Eigen::ArrayXd yaw = predictedStateSig_.row(3);
  Eigen::ArrayXd yawRate = predictedStateSig_.row(4);

  predictedMeasurementSig_ = Eigen::MatrixXd(radarMeasurementDim_, 2*augmentedStateDim_ + 1);
  for (int i = 0; i < (2 * augmentedStateDim_ + 1); i++)
  {
    if (abs(px(i)) < eps)
    {
      tools::addEpsIfZero(py(i));
    }
    predictedMeasurementSig_(1, i) = atan2(py(i), px(i));
  }

  Eigen::ArrayXd rho = (px * px + py * py).sqrt();
  predictedMeasurementSig_.row(0) = rho;
  predictedMeasurementSig_.row(2) = (px * v * yaw.cos() + py * v * yaw.sin()) / rho;
}

void UKF::updatePredRadarMeasurementMean()
{
  predictedMeasurementMean_ = predictedMeasurementSig_ * weights_;
}

void UKF::updatePredRadarMeasurementCov()
{
  Eigen::ArrayXXd error = predictedMeasurementSig_.colwise() - predictedMeasurementMean_;
  Eigen::MatrixXd weighted_error = error.rowwise() * weights_.transpose().array();   
  predictedMeasurementCov_ = Eigen::MatrixXd(radarMeasurementDim_,radarMeasurementDim_);
  predictedMeasurementCov_ = weighted_error * error.matrix().transpose() + radarMeasurementNoiseCov_;
}

void UKF::updateKalmanGainRadar()
{
  Eigen::ArrayXXd predictedStateError = predictedStateSig_.colwise() - stateMean_;
  Eigen::ArrayXXd pred_measurement_error = predictedMeasurementSig_.colwise() - predictedMeasurementMean_;
  Eigen::MatrixXd pred_weighted_state_error = predictedStateError.rowwise() * weights_.transpose().array();
  Eigen::MatrixXd pred_state_measurement_cov = pred_weighted_state_error * pred_measurement_error.matrix().transpose();

  kalmanGain_ = Eigen::MatrixXd(5,3);
  kalmanGain_ = pred_state_measurement_cov * predictedMeasurementCov_.inverse();
}

void UKF::predictLaserMeasurement()
{
  updatePredLaserMeasurementMean();
  updatePredLaserMeasurementCov();
}

void UKF::updatePredLaserMeasurementMean()
{
  predictedMeasurementMean_ = laserMeasurementMatrix_ * stateMean_;
}

void UKF::updatePredLaserMeasurementCov()
{
  predictedMeasurementCov_ = laserMeasurementMatrix_ * stateCov_ * laserMeasurementMatrix_.transpose() + laserMeasurementNoiseCov_;
}

void UKF::updateKalmanGainLaser()
{
  kalmanGain_ = stateCov_ * laserMeasurementMatrix_.transpose() * predictedMeasurementCov_.inverse();
}

Eigen::VectorXd UKF::getStateMean()
{
  return stateMean_;
}

