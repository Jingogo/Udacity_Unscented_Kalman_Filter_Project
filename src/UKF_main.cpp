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

  std::ifstream inputFile(argv[1]);
  if (!inputFile.is_open())
  {
    std::cout << "Could not open file" << argv[1] << std::endl;
  }

  double std_acceleration = 30;
  double std_yaw_acceleration = 3;
  double std_px_laser_ = 0.15;
  double std_py_laser_ = 0.15;
  double std_rho_radar_ = 0.3;
  double std_phi_radar_ = 0.03;
  double std_phirate_radar_ = 0.3;

  Eigen::MatrixXd processNoiseCov(2,2);
  Eigen::MatrixXd RadarMeasurementNoiseCov(3, 3);
  Eigen::MatrixXd laserMeasurementNoise(2, 2);
  processNoiseCov << pow(std_acceleration, 2), 0,
      0, pow(std_yaw_acceleration, 2);
  RadarMeasurementNoiseCov << pow(std_rho_radar_, 2), 0, 0,
      0, pow(std_phi_radar_, 2), 0,
      0, 0, pow(std_phirate_radar_, 2);
  laserMeasurementNoise << pow(std_px_laser_, 2), 0,
      0, pow(std_py_laser_, 2);

  UKF ukf(processNoiseCov, RadarMeasurementNoiseCov,laserMeasurementNoise);
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> groundTruths;
  std::string line;

  while (std::getline(inputFile, line))
  {
    std::istringstream iss(line);
    
    MeasurementPackage measPackage = tools::readMeasurement(iss); 
    ukf.processMeasurement(measPackage);

    Eigen::VectorXd groundTruth = tools::readGroundTruth(iss);
    groundTruths.push_back(groundTruth);
    Eigen::VectorXd stateMean = ukf.getStateMean();
    Eigen::VectorXd estimate = tools::stateToEstimate(stateMean);
    estimations.push_back(estimate);

    Eigen::VectorXd RMSE = tools::calculateRMSE(estimations, groundTruths);
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    std::cout << "Estimation: x,y,vx,vy\n" << estimate << std::endl;
    std::cout << "groundTruths: x,y,vx,vy\n" << groundTruth << std::endl;
    std::cout << "RMSE: x,y,vx,vy\n" << RMSE << std::endl;
  }
}