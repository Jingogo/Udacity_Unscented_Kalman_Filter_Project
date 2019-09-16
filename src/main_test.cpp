#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "ukf.h"
#include "tools.h"
#include <fstream>
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    std::cout << "The programs need two arguments" << std::endl;
    return -1;
  }

  std::ifstream input_file(argv[1]);
  if (!input_file.is_open())
  {
    std::cout << "Could not open file" << argv[1] << std::endl;
  }

  UKF ukf;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  std::string line;
  const int iteration_num = 10000;
  int cnt = 0;
  while (std::getline(input_file, line))
  {
    cnt++;
    if (cnt > iteration_num)
    {
      break;
    }
    std::istringstream iss(line);
    std::string sensor_type;
    iss >> sensor_type;
    MeasurementPackage meas_package;

    if (sensor_type.compare("L") == 0)
    {
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
    }
    else if (sensor_type.compare("R") == 0)
    {
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float theta;
      float ro_dot;
      iss >> ro;
      iss >> theta;
      iss >> ro_dot;
      //theta = tools::normalizeTheta(theta);
      meas_package.raw_measurements_ << ro, theta, ro_dot;
    }

    long long timestamp;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;

    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    VectorXd gt_values(4);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = vx_gt;
    gt_values(3) = vy_gt;
    ground_truth.push_back(gt_values);

    // Call ProcessMeasurement(meas_package) for Kalman filter

    ukf.processMeasurement(meas_package);

    // Push the current estimated x,y positon from the Kalman filter's
    //   state vector

    VectorXd estimate(4);
    VectorXd state_mean = ukf.getStateMean();

    estimate = tools::stateToEstimate(state_mean);
    estimations.push_back(estimate);

    VectorXd RMSE = tools::calculateRMSE(estimations, ground_truth);
    std::cout << "Estimation: x,y,vx,vy\n"
              << estimate << std::endl;
    std::cout << "ground_truth: x,y,vx,vy\n"
              << gt_values << std::endl;
    std::cout << "RMSE: x,y,vx,vy\n"
              << RMSE << std::endl;
  }
}