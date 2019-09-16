#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
public:
  UKF();
  virtual ~UKF();
  void initWeights();
  const bool isInitialized() const;
  void processMeasurement(const MeasurementPackage &meas_package);
  void readMeasurementPackage(const MeasurementPackage &meas_package);
  void initializeUKF();
  void initStateMean();
  void initStateCov();

  void predictState();
  Eigen::MatrixXd generateAugmentedStateSig();
  void updatePredStateSig(const Eigen::MatrixXd &state_sig);
  void updatePredStateMean();
  void updatePredStateCov();

  void updateLaser();
  void predictMeasurementLaser();
  void updatePredMeasurementMeanLaser();
  void updatePredMeasurementCovLaser();
  void updateKalmanGainLaser();
  void updateStateLaser();
  
  void updateRadar();
  void predictMeasurementRadar();
  void updatePredMeasurementSig();
  void updatePredMeasurementMeanRadar();
  void updatePredMeasurementCovRadar();
  void updateKalmanGainRadar();
  void updateStateRadar();
  
  Eigen::VectorXd getStateMean();
  Eigen::VectorXd getWeights();
  Eigen::MatrixXd getStateCov();
  Eigen::MatrixXd getPredStateSig();
  Eigen::MatrixXd getPredMeasurementSig();
  Eigen::VectorXd getPredMeasurementMean();
  Eigen::MatrixXd getPredMeasurementCov();
  Eigen::MatrixXd getKalmanGain();
  
  void setWeights(const Eigen::VectorXd weights);
  void setLambda(const double &lambda);
  void setPredStateSig(const Eigen::MatrixXd &pred_state_sig);
  void setStateMean(const Eigen::VectorXd state_mean);
  void setMeasurement(const Eigen::VectorXd measurement);
  void setSensorType(const MeasurementPackage::SensorType & sensor_type);
  void setStateCov(const Eigen::MatrixXd &state_cov);
  void setPredMeasurementCov(const Eigen::MatrixXd &pred_measurement_cov);

  void print();

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
  Eigen::MatrixXd measurement_matrix_laser_;
  // time when the state is true, in us
  long long update_time_us_;
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
  Eigen::MatrixXd measurement_noise_cov_laser_;
  Eigen::MatrixXd measurement_noise_cov_radar_;
  Eigen::MatrixXd kalman_gain_;
};

#endif  // UKF_H