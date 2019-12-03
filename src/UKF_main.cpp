#include <iostream>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include "json.hpp"
#include "UKF.h"
#include "tools.h"

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
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  std::string line;

  while (std::getline(input_file, line))
  {
    std::istringstream iss(line);
    
    MeasurementPackage meas_package = tools::readMeasurement(iss); 
    ukf.processMeasurement(meas_package);

    Eigen::VectorXd gt_values = tools::readGroundTruth(iss);
    ground_truth.push_back(gt_values);
    Eigen::VectorXd state_mean = ukf.getStateMean();
    Eigen::VectorXd estimate = tools::stateToEstimate(state_mean);
    estimations.push_back(estimate);

    Eigen::VectorXd RMSE = tools::calculateRMSE(estimations, ground_truth);
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    std::cout << "Estimation: x,y,vx,vy\n" << estimate << std::endl;
    std::cout << "ground_truth: x,y,vx,vy\n" << gt_values << std::endl;
    std::cout << "RMSE: x,y,vx,vy\n" << RMSE << std::endl;
  }
}