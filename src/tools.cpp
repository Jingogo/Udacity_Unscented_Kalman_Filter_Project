#include "tools.h"
#include <iostream>
#include <cmath>
#include <stdlib.h>

using std::vector;

namespace tools
{
Eigen::VectorXd calculateRMSE(const vector<Eigen::VectorXd> &estimations,
                                     const vector<Eigen::VectorXd> &ground_truth)
{
  if (estimations.empty() or ground_truth.empty())
  {
    throw "Empty estimations or groud truth.";
  }
  auto estimations_begin = estimations.begin();
  auto ground_truth_begin = ground_truth.begin();
  
  Eigen::VectorXd RMSE(4);
  RMSE << 0,0,0,0;
  for (; estimations_begin != estimations.cend(); estimations_begin++, ground_truth_begin++)
  {
    Eigen::ArrayXd diff = *estimations_begin - *ground_truth_begin;
    Eigen::VectorXd MSE = diff * diff;
    RMSE += MSE;
  }

  const auto estimations_size = estimations.size();
  RMSE = RMSE/estimations_size;
  RMSE = RMSE.array().sqrt();
  return RMSE;
}

void addEpsIfZero(double &div)
{
  if (abs(div) < 0.00001)
  {
    div += 0.00001 ;
  }
}

void normalizeRadarMeasurement(Eigen::VectorXd &measurement)
{
  double theta = measurement[1];
  if (theta > M_PI){
    theta -= 2*M_PI;
  }
  if (theta < -M_PI){
    theta += 2*M_PI;
  }
  measurement[1] = theta;
}

MeasurementPackage readMeasurement(std::istringstream &iss)
{
  std::string sensor_type;
  iss >> sensor_type;
  MeasurementPackage meas_package;

  if (sensor_type.compare("L") == 0){
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = Eigen::VectorXd(2);
    float px, py;
    iss >> px >> py;
    meas_package.raw_measurements_ << px, py;
  }
  else if (sensor_type.compare("R") == 0)
  {
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = Eigen::VectorXd(3);
    float ro, theta, ro_dot;
    iss >> ro >> theta >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;
  }

  long long timestamp;
  iss >> timestamp;
  meas_package.timestamp_ = timestamp;

  return meas_package;
}

Eigen::VectorXd readGroundTruth(std::istringstream &iss)
{
  float x_gt, y_gt, vx_gt, vy_gt;
  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  Eigen::VectorXd gt_values(4);
  gt_values << x_gt, y_gt, vx_gt, vy_gt;
  return gt_values;
}

Eigen::VectorXd stateToEstimate(const Eigen::VectorXd &state_mean)
{
  double p_x = state_mean(0);
  double p_y = state_mean(1);
  double v = state_mean(2);
  double yaw = state_mean(3);

  double v1 = cos(yaw) * v;
  double v2 = sin(yaw) * v;

  Eigen::VectorXd estimate(4); 
  estimate(0) = p_x;
  estimate(1) = p_y;
  estimate(2) = v1;
  estimate(3) = v2;

  return estimate;
}

double compareExpectedToActual(const Eigen::VectorXd &expected, const Eigen::VectorXd &actual)
{
  Eigen::VectorXd diff = expected - actual;
  return diff.squaredNorm();
}
double compareExpectedToActual(const Eigen::MatrixXd &expected, const Eigen::MatrixXd &actual)
{
  Eigen::MatrixXd diff = expected - actual;
  return diff.squaredNorm();
}
} // namespace tools
