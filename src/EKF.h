#ifndef EKF_H_
#define EKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "measurementPackage.h"
#include "tools.h"

class EKF{
public:
  EKF(Eigen::VectorXd processNoise, Eigen::MatrixXd RadarMeasNoiseCov, Eigen::MatrixXd laserMeasNoiseCov);
  virtual ~EKF();
  bool isInitialized(){return initializationFlag;};
  void processMeasurement(const MeasurementPackage &measurementPack);
  void readMeasurementPackage(const MeasurementPackage &measurementPack);
  void initEKF();
  void initStateMean();
  void initStateCov();
  void predictState();
  void updateStateTransitionMatrix();
  void updateProcessNoiseCov();
  void updateState();
  void updateStateByRadar();
  void updateStateByLaser();
  void updateRadarMeasurementMatrix();
  Eigen::MatrixXd calculateJacobian();
  Eigen::VectorXd predictMeasurement();
  bool isRadar(){return sensorType_ == MeasurementPackage::RADAR;};
  bool isLaser(){return sensorType_ == MeasurementPackage::LASER;};

  Eigen::VectorXd getStateMean();


 private:
  bool initializationFlag = false;
  long long previousTimestamp_ = 0;
  double dt_ = 0.0;
  MeasurementPackage::SensorType sensorType_ = MeasurementPackage::RADAR;

  Eigen::VectorXd measurement_;
  Eigen::VectorXd stateMean_;
  Eigen::MatrixXd stateCov_;
  Eigen::MatrixXd stateTransitionMatrix_;
  Eigen::MatrixXd processNoise_;
  Eigen::MatrixXd processNoiseCov_;
  Eigen::MatrixXd radarMeasurementNoiseCov_;
  Eigen::MatrixXd laserMeasurementNoiseCov_;
  Eigen::MatrixXd laserMeasurementMatrix_;
  Eigen::MatrixXd radarMeasurementMatrix_;
};

#endif // EKF_H_
