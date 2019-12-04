#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurementPackage.h"

class UKF {
public:
  UKF(Eigen::MatrixXd processNoiseCov, Eigen::MatrixXd radarMeasurementNoiseCov, Eigen::MatrixXd laserMeasurementNoiseCov);
  virtual ~UKF();
  void initWeights();
  const bool isInitialized() const {return initializationFlag;};
  void processMeasurement(const MeasurementPackage &measPackage);
  void readMeasurementPackage(const MeasurementPackage &measPackage);
  void initUKF();
  void initStateMean();
  void initStateCov();

  void predictState();
  Eigen::MatrixXd generateAugmentedStateSig();
  void updatePredStateSig(const Eigen::MatrixXd &stateSig);
  void updatePredStateMean();
  void updatePredStateCov();
  
  void updateState();
  void updateStateByLaser();
  void predictLaserMeasurement();
  void updatePredLaserMeasurementMean();
  void updatePredLaserMeasurementCov();
  void updateKalmanGainLaser();
  
  void updateStateByRadar();
  void predictRadarMeasurement();
  void updatePredMeasurementSig();
  void updatePredRadarMeasurementMean();
  void updatePredRadarMeasurementCov();
  void updateKalmanGainRadar();
  
  Eigen::VectorXd getStateMean();
  bool isRadar(){return sensorType_ == MeasurementPackage::RADAR;};
  bool isLaser(){return sensorType_ == MeasurementPackage::LASER;};

private:
  bool initializationFlag = false;
  long long previousTimestamp_ = 0;
  double dt_ = 0.0;
  MeasurementPackage::SensorType sensorType_ = MeasurementPackage::RADAR;

  int stateDim_ = 5;
  int augmentedStateDim_ = 7;
  int radarMeasurementDim_ = 3;

  double lambda_;
  Eigen::VectorXd weights_;

  Eigen::VectorXd measurement_;
  Eigen::VectorXd stateMean_;
  Eigen::MatrixXd stateCov_;
  Eigen::MatrixXd predictedStateSig_;
  Eigen::MatrixXd predictedMeasurementSig_;
  Eigen::VectorXd predictedMeasurementMean_;
  Eigen::MatrixXd predictedMeasurementCov_;
  Eigen::MatrixXd laserMeasurementMatrix_;

  Eigen::MatrixXd processNoiseCov_;
  Eigen::MatrixXd radarMeasurementNoiseCov_;
  Eigen::MatrixXd laserMeasurementNoiseCov_;
  Eigen::MatrixXd kalmanGain_;
};

#endif  // UKF_H