#ifndef EKF_H_
#define EKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tools.h"

class EKF{
public:
  EKF(Eigen::MatrixXd R_laser, Eigen::MatrixXd R_radar, Eigen::VectorXd noise);
  virtual ~EKF();
  bool isInitialized(){return is_initialized_;};
  void processMeasurement(const MeasurementPackage &measurement_pack);
  void readMeasurementPackage(const MeasurementPackage &measurement_pack);
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
  Eigen::VectorXd PredMeasurement();
  bool isRadar(){return sensor_type_ == MeasurementPackage::RADAR;};
  bool isLaser(){return sensor_type_ == MeasurementPackage::LASER;};

  Eigen::VectorXd getStateMean();


 private:
  bool is_initialized_ = false;
  long long previous_timestamp_ = 0;
  Eigen::VectorXd measurement_;
  double dt_ = 0.0;
  MeasurementPackage::SensorType sensor_type_ = MeasurementPackage::RADAR;

  Eigen::VectorXd state_mean_;
  Eigen::MatrixXd state_cov_;
  Eigen::MatrixXd state_transition_matrix_;
  Eigen::MatrixXd process_noise_cov_;
  Eigen::MatrixXd laser_measurement_noise_cov_;
  Eigen::MatrixXd radar_measurement_noise_cov_;
  Eigen::MatrixXd laser_measurement_matrix_;
  Eigen::MatrixXd radar_measurement_matrix_;
  Eigen::VectorXd noise_;
};

#endif // EKF_H_
