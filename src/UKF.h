#ifndef UKF_H
#define UKF_H

#include <iomanip>
#include "Eigen/Dense"
#include "measurementPackage.h"

class UKF {
public:
  UKF(Eigen::MatrixXd processNoiseCov, Eigen::MatrixXd radarMeasurementNoiseCov, Eigen::MatrixXd laserMeasurementNoiseCov);
  virtual ~UKF();
  void initWeights();
  const bool isInitialized() const {return initializationFlag;};
  void processMeasurement(const MeasurementPackage &measurementPackage);
  void readMeasurementPackage(const MeasurementPackage &measurementPackage);
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

  const int stateDim_ = 5;
  const int augmentedStateDim_ = 7;
  const int radarMeasurementDim_ = 3;
  const int laserMeasurementDim_ = 2;
  const double lambda_ = 3 - stateDim_;;
  Eigen::VectorXd weights_;

  Eigen::VectorXd measurement_;
  Eigen::VectorXd stateMean_;
  Eigen::MatrixXd stateCov_;
  Eigen::MatrixXd predictedStateSig_;
  Eigen::MatrixXd predictedMeasurementSig_;
  Eigen::VectorXd predictedMeasurementMean_;
  Eigen::MatrixXd predictedMeasurementCov_;
  Eigen::MatrixXd laserMeasurementMatrix_;
  Eigen::MatrixXd kalmanGain_;

  Eigen::MatrixXd processNoiseCov_;
  Eigen::MatrixXd radarMeasurementNoiseCov_;
  Eigen::MatrixXd laserMeasurementNoiseCov_;
};

#endif  // UKF_H