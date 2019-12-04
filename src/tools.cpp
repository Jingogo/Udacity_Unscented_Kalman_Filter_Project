#include "tools.h"
#include <iostream>
#include <cmath>
#include <stdlib.h>

using std::vector;

namespace tools
{
Eigen::VectorXd calculateRMSE(const vector<Eigen::VectorXd> &estimations,
                                     const vector<Eigen::VectorXd> &groundTruths)
{
  if (estimations.empty() or groundTruths.empty())
  {
    throw "Empty estimations or groud truth.";
  }
  auto estimationsBegin = estimations.begin();
  auto groundTruthsBegin = groundTruths.begin();
  
  Eigen::VectorXd RMSE(4);
  RMSE << 0,0,0,0;
  for (; estimationsBegin != estimations.cend(); estimationsBegin++, groundTruthsBegin++)
  {
    Eigen::ArrayXd diff = *estimationsBegin - *groundTruthsBegin;
    Eigen::VectorXd MSE = diff * diff;
    RMSE += MSE;
  }

  const auto estimationsSize = estimations.size();
  RMSE = RMSE/estimationsSize;
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
  std::string sensorType_;
  iss >> sensorType_;
  MeasurementPackage measPackage;

  if (sensorType_.compare("L") == 0){
    measPackage.sensorType_ = MeasurementPackage::LASER;
    measPackage.rawMeasurements_ = Eigen::VectorXd(2);
    float px, py;
    iss >> px >> py;
    measPackage.rawMeasurements_ << px, py;
  }
  else if (sensorType_.compare("R") == 0)
  {
    measPackage.sensorType_ = MeasurementPackage::RADAR;
    measPackage.rawMeasurements_ = Eigen::VectorXd(3);
    float ro, theta, ro_dot;
    iss >> ro >> theta >> ro_dot;
    measPackage.rawMeasurements_ << ro, theta, ro_dot;
  }

  long long timestamp;
  iss >> timestamp;
  measPackage.timestamp_ = timestamp;

  return measPackage;
}

Eigen::VectorXd readGroundTruth(std::istringstream &iss)
{
  float x_gt, y_gt, vx_gt, vy_gt;
  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  Eigen::VectorXd groundTruth(4);
  groundTruth << x_gt, y_gt, vx_gt, vy_gt;
  return groundTruth;
}

Eigen::VectorXd stateToEstimate(const Eigen::VectorXd &stateMean)
{
  double p_x = stateMean(0);
  double p_y = stateMean(1);
  double v = stateMean(2);
  double yaw = stateMean(3);

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
