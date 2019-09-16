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

double normalizeTheta(double theta)
{
  if (theta > M_PI)
  {
    theta -= 2*M_PI;
  }

  if (theta < -M_PI) 
  {
    theta += 2*M_PI;
  }

  return theta;
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
