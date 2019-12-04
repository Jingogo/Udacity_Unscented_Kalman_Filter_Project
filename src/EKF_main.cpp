
#include <iostream>     
#include <fstream>
#include <iomanip> 
#include <math.h>
#include <uWS/uWS.h>
#include "json.hpp"
#include "EKF.h"
#include "tools.h"

int main(int argc, char *argv[]){
  if (argc != 2){
    std::cout << "The programs need two arguments" << std::endl;
    return -1;
  }

  std::ifstream inputFile(argv[1]);
  if (!inputFile.is_open()){
    std::cout << "Could not open data file" << argv[1] << std::endl;
  }

  Eigen::VectorXd processNoise(2);
  Eigen::MatrixXd RadarMeasNoiseCov(3, 3);
  Eigen::MatrixXd laserMeasNoise(2, 2);
  processNoise << 9.0, 9.0;
  laserMeasNoise << 0.0225, 0,
      0, 0.0225;
  RadarMeasNoiseCov << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  EKF ekf(processNoise, RadarMeasNoiseCov, laserMeasNoise);
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> groundTruths;
  std::string line;

  while (std::getline(inputFile, line)){  
    std::istringstream iss(line);

    MeasurementPackage measurementPackage = tools::readMeasurement(iss);
    ekf.processMeasurement(measurementPackage);

    Eigen::VectorXd groundTruth = tools::readGroundTruth(iss);
    groundTruths.push_back(groundTruth);
    Eigen::VectorXd estimate = ekf.getStateMean();
    estimations.push_back(estimate);

    Eigen::VectorXd RMSE = tools::calculateRMSE(estimations, groundTruths);
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    std::cout << "Estimation: x,y,vx,vy\n" << estimate << std::endl;
    std::cout << "Ground_truth: x,y,vx,vy\n" << groundTruth << std::endl;
    std::cout << "RMSE: x,y,vx,vy\n" << RMSE << std::endl;
  }
}


