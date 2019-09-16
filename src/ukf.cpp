#include "ukf.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>
#include <iomanip>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::ArrayXXd;
using Eigen::ArrayXd;

using namespace std;

UKF::UKF()
{
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  dim_measurement_radar_ = 3;
  state_mean_ = VectorXd(n_x_);
  state_cov_ = MatrixXd(n_x_, n_x_);
  pred_state_sig_ = MatrixXd(n_x_, 2*n_aug_+1);
  pred_measurement_sig_ = MatrixXd(dim_measurement_radar_, 2*n_aug_+1);
  update_time_us_ = 0;
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
  process_noise_cov_ = MatrixXd(2,2);
  process_noise_cov_ << pow(std_acceleration, 2), 0,
                        0, pow(std_yaw_acceleration, 2);
  measurement_matrix_laser_ = MatrixXd(2, 5);
  measurement_matrix_laser_ << 1, 0, 0, 0, 0,
                               0, 1, 0, 0, 0;
  measurement_noise_cov_laser_ = MatrixXd(2, 2);
  measurement_noise_cov_laser_ << pow(std_px_laser_, 2), 0,
                                  0, pow(std_py_laser_, 2);
  measurement_noise_cov_radar_ = MatrixXd(3, 3);     
  measurement_noise_cov_radar_  << pow(std_rho_radar_, 2), 0, 0,
                                  0, pow(std_phi_radar_, 2), 0,
                                  0, 0, pow(std_phirate_radar_, 2);                          
}

UKF::~UKF() {}

void UKF::initWeights()
{
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  weights_.tail(2*n_aug_).setConstant( 1.0/(2*(lambda_ + n_aug_)));
}

void UKF::processMeasurement(const MeasurementPackage &meas_package)
{
  readMeasurementPackage(meas_package);
  cout << fixed << setprecision(3) << endl;
  if (isInitialized()==false)
  {
    initializeUKF();
  }  
  else
  {
    cout << "#Before predictState " << endl;
    VectorXd expected_state_mean(5);
    expected_state_mean << 0.312243,  0.58034,    0,    0,  0;
    double error = tools::compareExpectedToActual(expected_state_mean, state_mean_);
    cout << "state_mean: actual expecetd diff: " << error << endl;
    MatrixXd expected_state_cov(5,5);
    expected_state_cov << 
      1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;
    error = tools::compareExpectedToActual(expected_state_cov, state_cov_);
    cout << "state_cov: actual expecetd diff: " << error << endl;

    predictState();

    cout << "#After predictState()" << endl;
    expected_state_mean << 0.312, 0.580, 0, 0, 0;
    error = tools::compareExpectedToActual(expected_state_mean, state_mean_);
    cout << "state mean: actual expecetd diff: " << error << endl;
    expected_state_cov << 
      1.004, 0.000, 0.106,-0.000, 0.000,
      0.000, 1.000, 0.000, 0.000, 0.000, 
      0.106, 0.000, 3.250, 0.000, 0.000,
     -0.000, 0.000, 0.000, 1.004, 0.106, 
      0.000, 0.000, 0.000, 0.106, 3.250;
    error = tools::compareExpectedToActual(expected_state_cov, state_cov_);
    cout << "state_cov: actual expecetd diff: " << error << endl; 
  
    if (sensor_type_ == MeasurementPackage::RADAR)
    {
      updateRadar();
    }
    else
    {
      updateLaser();
    }
  }
}

void UKF::readMeasurementPackage(const MeasurementPackage &meas_package)
{
  measurement_ = meas_package.raw_measurements_;
  sensor_type_ = meas_package.sensor_type_;
  long long new_timestamp = meas_package.timestamp_;
  dt_ = (new_timestamp - update_time_us_) / 1000000.0;
  update_time_us_ = new_timestamp;
}

void UKF::initializeUKF()
{
    initStateMean();
    initStateCov();  
    is_initialized_ = true;
}

void UKF::initStateMean()
{
  state_mean_ << 0, 0, 0, 0, 0;
  if (sensor_type_ == MeasurementPackage::LASER)
  {
    state_mean_[0] = measurement_[0];
    state_mean_[1] = measurement_[1];
  }
  else if (sensor_type_ == MeasurementPackage::RADAR)
  {
    const double rho = measurement_[0];
    const double phi = measurement_[1];
    state_mean_[0] = rho * cos(phi);
    state_mean_[1] = rho * sin(phi);
  }
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
  cout << "#During predictState()" << endl;
  MatrixXd state_sig = MatrixXd(n_aug_, 2*n_aug_ + 1);
  state_sig = generateAugmentedStateSig();

  MatrixXd expected_state_sig(7,15);
  expected_state_sig << 
    0.312,   2.548,   0.312,   0.312,   0.312,   0.312,   0.312,   0.312,  -1.924,
    0.312,   0.312,   0.312,   0.312,   0.312,   0.312,   
    0.580,   0.580,   2.816,   0.580,   0.580,   0.580,   0.580,   0.580,   0.580,
    -1.656,   0.580,   0.580,   0.580,   0.580,   0.580,
    0.000,   0.000,   0.000,   2.236,   0.000,   0.000,   0.000,   0.000,   0.000,
    0.000,  -2.236,   0.000,   0.000,   0.000,   0.000,
    0.000,   0.000,   0.000,   0.000,   2.236,   0.000,   0.000,   0.000,   0.000,
    0.000,   0.000,  -2.236,   0.000,   0.000,   0.000,
    0.000,   0.000,   0.000,   0.000,   0.000,   2.236,   0.000,   0.000,   0.000,
    0.000,   0.000,   0.000,  -2.236,   0.000,   0.000,
    0.000,   0.000,   0.000,   0.000,   0.000,   0.000,  67.082,   0.000,   0.000, 
    0.000,   0.000,   0.000,   0.000, -67.082,   0.000,
    0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,  67.082,   0.000,
    0.000,   0.000,   0.000,   0.000,   0.000, -67.082;
  double error = tools::compareExpectedToActual(expected_state_sig, state_sig);
  cout << "##After generateAugmentedStateSig()" << endl;
  cout << "state sig: actual expecetd diff: " << error << endl;

  updatePredStateSig(state_sig);

  MatrixXd expected_pred_state_sig(5,15);
  expected_pred_state_sig << 
    0.312,  2.548,  0.312,  0.424,  0.312,  0.312,  0.396,  0.312, -1.924,  0.312,
    0.200,  0.312,  0.312,  0.228,  0.312,
    0.580,  0.580,  2.816,  0.580,  0.580,  0.580,  0.580,  0.580,  0.580, -1.656,
    0.580,  0.580,  0.580,  0.580,  0.580,
    0.000,  0.000,  0.000,  2.236,  0.000,  0.000,  3.354,  0.000,  0.000,  0.000,
   -2.236,  0.000,  0.000, -3.354,  0.000,
    0.000,  0.000,  0.000,  0.000,  2.236,  0.112,  0.000,  0.084,  0.000,  0.000,
    0.000, -2.236, -0.112,  0.000, -0.084,
    0.000,  0.000,  0.000,  0.000,  0.000,  2.236,  0.000,  3.354,  0.000,  0.000,
    0.000,  0.000, -2.236,  0.000, -3.354;
  error = tools::compareExpectedToActual(expected_pred_state_sig, pred_state_sig_);
  cout << "##After updatePredStateSig()" << endl;
  cout << "pred state sig: actual expecetd diff: " << error << endl;

  updatePredStateMean();

  updatePredStateCov();
}

MatrixXd UKF::generateAugmentedStateSig()
{ 
  MatrixXd state_sig = MatrixXd::Zero(n_aug_, 2*n_aug_ + 1);
  VectorXd aug_state_mean = VectorXd::Zero(n_aug_);
  MatrixXd aug_state_cov = MatrixXd::Zero(n_aug_, n_aug_);

  aug_state_mean.head(n_x_) = state_mean_;
  aug_state_cov.block(0, 0, n_x_, n_x_) = state_cov_;
  aug_state_cov.block(n_x_, n_x_, 2, 2) = process_noise_cov_;
  MatrixXd A = aug_state_cov.llt().matrixL();
  A = sqrt(lambda_ + n_aug_)*A;
  state_sig.col(0) = aug_state_mean;
  state_sig.block(0, 1, n_aug_, n_aug_) =  A.colwise() + aug_state_mean;
  state_sig.block(0, n_aug_+1, n_aug_, n_aug_) = (-A).colwise() + aug_state_mean;

  return state_sig;
}

void UKF::updatePredStateSig(const MatrixXd &state_sig)
{
  // should be processed by colomn, so we can deal with situation when the yaw rate is zero
  double eps = 1e-5;
  VectorXd px = state_sig.row(0);
  VectorXd py = state_sig.row(1);
  VectorXd v = state_sig.row(2);
  VectorXd yaw = state_sig.row(3);
  VectorXd yaw_rate = state_sig.row(4);
  VectorXd acceleration_noise = state_sig.row(5);
  VectorXd yaw_acceleration_noise = state_sig.row(6);

  pred_state_sig_.row(2) = v + dt_*acceleration_noise;
  pred_state_sig_.row(3) = yaw + dt_*yaw_rate + 0.5*dt_*dt_*yaw_acceleration_noise;
  pred_state_sig_.row(4) = yaw_rate + dt_*yaw_acceleration_noise;
  for(int i = 0; i < 2*n_aug_ + 1; i++)
  {
    if(abs(yaw_rate(i)) < eps)
    {
      pred_state_sig_(0,i) = px(i) + v(i)*cos(yaw(i))*dt_;
      pred_state_sig_(1,i) = py(i) + v(i)*sin(yaw(i))*dt_;
    }
    else
    {
      pred_state_sig_(0,i) = px(i) + (v(i)/yaw_rate(i))*(sin(yaw(i)+dt_*yaw_rate(i)) - sin(yaw(i)));
      pred_state_sig_(1,i) = py(i) + (v(i)/yaw_rate(i))*(cos(yaw(i)) - cos(yaw(i)+dt_*yaw_rate(i))); 
    }  
    pred_state_sig_(0,i) = pred_state_sig_(0,i) + 0.5*dt_*dt_*cos(yaw(i))*acceleration_noise(i);
    pred_state_sig_(1,i) = pred_state_sig_(1,i) + 0.5*dt_*dt_*sin(yaw(i))*acceleration_noise(i);
  } 
}

void UKF::updatePredStateMean()
{
  state_mean_ = pred_state_sig_*weights_;
}

void UKF::updatePredStateCov()
{
  cout << "##During updatePredStateCov" << endl;
  ArrayXXd pred_state_error = pred_state_sig_.colwise() - state_mean_;
  MatrixXd weighted_pred_state_error = pred_state_error.rowwise()*weights_.transpose().array();
  state_cov_ = weighted_pred_state_error*pred_state_error.matrix().transpose();
  
  MatrixXd expected_pred_state_error(5, 15);
  expected_pred_state_error << 
    0.000, 2.236, 0.000, 0.112, 0.000, 0.000, 0.084, 0.000,-2.236, 0.000,
   -0.112, 0.000, 0.000,-0.084, 0.000,
    0.000, 0.000, 2.236, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,-2.236, 
    0.000, 0.000, 0.000, 0.000, 0.000,
    0.000, 0.000, 0.000, 2.236, 0.000, 0.000, 3.354, 0.000, 0.000, 0.000, 
   -2.236, 0.000, 0.000,-3.354, 0.000,
   -0.000,-0.000,-0.000,-0.000, 2.236, 0.112,-0.000, 0.084,-0.000,-0.000, 
   -0.000,-2.236,-0.112,-0.000,-0.084,
    0.000, 0.000, 0.000, 0.000, 0.000, 2.236, 0.000, 3.354, 0.000, 0.000,
    0.000, 0.000,-2.236, 0.000,-3.354;
  double error = tools::compareExpectedToActual(expected_pred_state_error, pred_state_error);
  cout << "pred_state_error: actual expecetd diff: " << error << endl;
}

void UKF::updateLaser() 
{
  predictMeasurementLaser();
  updateKalmanGainLaser();
  updateStateLaser();
}

void UKF::predictMeasurementLaser()
{
  updatePredMeasurementMeanLaser();
  updatePredMeasurementCovLaser();
}

void UKF::updatePredMeasurementMeanLaser()
{
  pred_measurement_mean_ = measurement_matrix_laser_*state_mean_;
}

void UKF::updatePredMeasurementCovLaser()
{
  pred_measurement_cov_ = measurement_matrix_laser_ * state_cov_ * measurement_matrix_laser_.transpose() 
                        + measurement_noise_cov_laser_;
}

void UKF::updateKalmanGainLaser() 
{
  kalman_gain_ = state_cov_*measurement_matrix_laser_.transpose()*pred_measurement_cov_.inverse();
}

void UKF::updateStateLaser() 
{
  state_mean_ = state_mean_ + kalman_gain_*(measurement_ - pred_measurement_mean_);
  state_cov_ =  state_cov_ - kalman_gain_*pred_measurement_cov_*kalman_gain_.transpose();
}

void UKF::updateRadar() 
{
  cout << "#During updateRadar " << endl;

  predictMeasurementRadar();

  cout << "##After predictMeasurementRadar" << endl;
  VectorXd expected_pred_measurement_mean(3);
  expected_pred_measurement_mean << 1.312, 0.965,  0.125;
  double error = tools::compareExpectedToActual(expected_pred_measurement_mean,pred_measurement_mean_);
  cout << "pred_measurement_mean: actual expecetd diff: " << error << endl;
  MatrixXd expected_pred_measurement_cov(3,3);
  expected_pred_measurement_cov << 
    0.807, -0.026, -0.058,
   -0.026,  1.003, -0.050,
   -0.058, -0.050,  0.810;
  error = tools::compareExpectedToActual(expected_pred_measurement_cov, pred_measurement_cov_);
  cout << "pred_measurement_cov: actual expecetd diff: " << error << endl;  

  updateKalmanGainRadar();
  cout << "##After updateKalmanGainRadar" << endl;
  MatrixXd expected_kalman_gain(5,3);
  expected_kalman_gain << 
    0.153, -0.585,  0.035,
    0.344,  0.647,  0.065, 
    0.195, -0.043,  1.868,
    0.000,  0.000,  0.000,
    0.000,  0.000,  0.000;
  error = tools::compareExpectedToActual(expected_kalman_gain, kalman_gain_);
  cout << "kalman_gain_: actual expecetd diff: " << error << endl;

  updateStateRadar();

  cout << "##After updateStateRadar " << endl;
  VectorXd expected_state_mean(5);
  expected_state_mean << 0.675, 0.522, 8.868, 0.000, 0.000;
  error = tools::compareExpectedToActual(expected_state_mean, state_mean_);
  cout << "state_mean: actual expecetd diff: " << error << endl;
  MatrixXd expected_state_cov(5,5);
  expected_state_cov << 
    0.635, 0.333,-0.037,-0.000,-0.000,
    0.333, 0.500,-0.023,-0.000,-0.000, 
   -0.037,-0.023, 0.423,-0.000,-0.000,
   -0.000,-0.000,-0.000, 1.004, 0.106, 
   -0.000,-0.000,-0.000, 0.106, 3.250;
  error = tools::compareExpectedToActual(expected_state_cov, state_cov_);
  cout << "state_cov: actual expecetd diff: " << error << endl;
}

void UKF::predictMeasurementRadar()
{
  cout << "##During predictMeasurementRadar" << endl;

  updatePredMeasurementSig();

  cout << "###After updatePredMeasurementSig" << endl;
  MatrixXd expected_pred_measurement_sig(3,15);
  expected_pred_measurement_sig << 
    0.659, 2.614, 2.834, 0.719, 0.659, 0.659, 0.703, 0.659, 2.009, 1.685, 
    0.614, 0.659, 0.659, 0.624, 0.659,
    1.077, 0.224, 1.460, 0.940, 1.077, 1.077, 0.972, 1.077, 2.849,-1.384,
    1.238, 1.077, 1.077, 1.196, 1.077,
    0.000, 0.000, 0.000, 1.319, 0.000, 0.000, 1.891, 0.000, 0.000, 0.000,
   -0.730,-0.000, 0.000,-1.228, 0.000;
  double error = tools::compareExpectedToActual(expected_pred_measurement_sig, pred_measurement_sig_);
  cout << "pred_measurement_sig: actual expecetd diff: " << error << endl;

  updatePredMeasurementMeanRadar();
  updatePredMeasurementCovRadar();
}

void UKF::updatePredMeasurementSig()
{
  double eps = 1e-5;
  ArrayXd px = pred_state_sig_.row(0);
  ArrayXd py = pred_state_sig_.row(1);
  ArrayXd v = pred_state_sig_.row(2);
  ArrayXd yaw = pred_state_sig_.row(3);
  ArrayXd yaw_rate = pred_state_sig_.row(4);
  for(int i = 0; i < (2*n_aug_ + 1); i++)
  {
    if (abs(px(i))<eps )
    {
      tools::addEpsIfZero(py(i));
    }
    pred_measurement_sig_(1,i) = atan2(py(i), px(i));
  }

  ArrayXd rho = (px*px + py*py).sqrt();
  pred_measurement_sig_.row(0) = rho;
  pred_measurement_sig_.row(2) = (px*v*yaw.cos() + py*v*yaw.sin())/rho; 
}

void UKF::updatePredMeasurementMeanRadar()
{
  pred_measurement_mean_ = pred_measurement_sig_*weights_;
}

void UKF::updatePredMeasurementCovRadar()
{
  ArrayXXd error = pred_measurement_sig_.colwise() - pred_measurement_mean_;
  MatrixXd weighted_error = error.rowwise()*weights_.transpose().array();
  pred_measurement_cov_ = weighted_error*error.matrix().transpose() + measurement_noise_cov_radar_;
}

void UKF::updateKalmanGainRadar() 
{
  ArrayXXd pred_state_error = pred_state_sig_.colwise() - state_mean_;
  ArrayXXd pred_measurement_error = pred_measurement_sig_.colwise() - pred_measurement_mean_;
  MatrixXd pred_weighted_state_error = pred_state_error.rowwise()*weights_.transpose().array();
  MatrixXd pred_state_measurement_cov = pred_weighted_state_error*pred_measurement_error.matrix().transpose();
                                        
  kalman_gain_ = pred_state_measurement_cov*pred_measurement_cov_.inverse();
}

void UKF::updateStateRadar() 
{
  VectorXd measurement_error = measurement_ - pred_measurement_mean_;
  double phi_error = measurement_error(1);
  phi_error = tools::normalizeTheta(phi_error);
  measurement_error(1) = phi_error;

  state_mean_ = state_mean_ + kalman_gain_*measurement_error ;
  state_cov_ =  state_cov_ - kalman_gain_*pred_measurement_cov_*kalman_gain_.transpose();
}

VectorXd UKF::getStateMean()
{
  return state_mean_;
}

VectorXd UKF::getWeights()
{
  return weights_;
}
MatrixXd UKF::getStateCov()
{
  return state_cov_;
}

MatrixXd UKF::getPredStateSig()
{
  return pred_state_sig_;
}

MatrixXd UKF::getPredMeasurementSig()
{
  return pred_measurement_sig_;
}

VectorXd UKF::getPredMeasurementMean()
{
  return pred_measurement_mean_;
}

MatrixXd UKF::getPredMeasurementCov()
{
  return pred_measurement_cov_;
}

MatrixXd UKF::getKalmanGain()
{
  return kalman_gain_;
}

void UKF::setWeights(const Eigen::VectorXd weights)
{
  weights_ = weights;
}

void UKF::setLambda(const double &lambda)
{
  lambda_ = lambda;
}

void UKF::setPredStateSig(const Eigen::MatrixXd &pred_state_sig)
{
  pred_state_sig_ = pred_state_sig;
}

void UKF::setStateMean(const Eigen::VectorXd state_mean)
{
  state_mean_ = state_mean;
}

void UKF::setMeasurement(const Eigen::VectorXd measurement)
{
  measurement_ = measurement;
}

void UKF::setSensorType(const MeasurementPackage::SensorType & sensor_type)
{
  sensor_type_ = sensor_type;
}

void UKF::setStateCov(const Eigen::MatrixXd &state_cov)
{
  state_cov_ = state_cov;
}

void UKF::setPredMeasurementCov(const Eigen::MatrixXd &pred_measurement_cov)
{
  pred_measurement_cov_ = pred_measurement_cov;
}

void UKF::print()
{
  cout << "Measurement: " << measurement_.transpose()<< endl;
  cout << "State mean: " << state_mean_.transpose()<< endl;
  VectorXd estimate = tools::stateToEstimate(state_mean_);
  cout << "Estimation: " << estimate.transpose() << "\n" << endl;
}

const bool UKF::isInitialized() const 
{
  return is_initialized_;
}