#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
public:
  UKF();
  virtual ~UKF();
  void initWeights();
  const bool isInitialized() const {return is_initialized_;};
  void processMeasurement(const MeasurementPackage &meas_package);
  void readMeasurementPackage(const MeasurementPackage &meas_package);
  void initUKF();
  void initStateMean();
  void initStateCov();

  void predictState();
  Eigen::MatrixXd generateAugmentedStateSig();
  void updatePredStateSig(const Eigen::MatrixXd &state_sig);
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
  bool isRadar(){return sensor_type_ == MeasurementPackage::RADAR;};
  bool isLaser(){return sensor_type_ == MeasurementPackage::LASER;};

private:
  bool is_initialized_;
  int n_x_;
  int n_aug_;
  int dim_measurement_radar_;
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd state_mean_;
  Eigen::MatrixXd state_cov_;
  Eigen::MatrixXd pred_state_sig_;
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd pred_measurement_sig_;
  Eigen::VectorXd pred_measurement_mean_;
  Eigen::MatrixXd pred_measurement_cov_;
  Eigen::MatrixXd laser_measurement_matrix_;
  // time when the state is true, in us
  long long previous_timestamp_;
  double dt_;
  MeasurementPackage::SensorType sensor_type_;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_acceleration;
  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yaw_acceleration;
  // Laser measurement noise standard deviation position1 in m
  double std_px_laser_;
  // Laser measurement noise standard deviation position2 in m
  double std_py_laser_;
  // Radar measurement noise standard deviation radius in m
  double std_rho_radar_;
  // Radar measurement noise standard deviation angle in rad
  double std_phi_radar_;
  // Radar measurement noise standard deviation radius change in m/s
  double std_phirate_radar_;
  // Sigma point spreading parameter
  double lambda_;
  // Weights of sigma points
  Eigen::VectorXd weights_;
  Eigen::MatrixXd process_noise_cov_;
  Eigen::MatrixXd laser_measurement_noise_cov_;
  Eigen::MatrixXd radar_measurement_noise_cov_;
  Eigen::MatrixXd kalman_gain_;
};

#endif  // UKF_H