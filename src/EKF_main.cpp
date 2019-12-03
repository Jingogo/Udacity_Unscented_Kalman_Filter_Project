
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

  std::ifstream input_file(argv[1]);
  if (!input_file.is_open()){
    std::cout << "Could not open data file" << argv[1] << std::endl;
  }

  Eigen::MatrixXd R_laser(2, 2);
  Eigen::MatrixXd R_radar(3, 3);
  Eigen::VectorXd noise(2);
  //measurement covariance matrix - laser
  R_laser << 0.0225, 0,
             0, 0.0225;
  //measurement covariance matrix - radar
  R_radar << 0.09, 0, 0,
             0, 0.0009, 0,
             0, 0, 0.09;
 
  noise << 9.0, 9.0;
                      
  EKF ekf(R_laser, R_radar,  noise);
  std::vector<Eigen::VectorXd> estimations;
  std::vector<Eigen::VectorXd> ground_truth;
  std::string line;

  while (std::getline(input_file, line)){  
    std::istringstream iss(line);

    MeasurementPackage meas_package = tools::readMeasurement(iss);
    ekf.processMeasurement(meas_package);

    Eigen::VectorXd gt_values = tools::readGroundTruth(iss);
    ground_truth.push_back(gt_values);
    Eigen::VectorXd estimate = ekf.getStateMean();
    estimations.push_back(estimate);

    Eigen::VectorXd RMSE = tools::calculateRMSE(estimations, ground_truth);
    std::cout << std::fixed;
    std::cout << std::setprecision(3);
    std::cout << "Estimation: x,y,vx,vy\n" << estimate << std::endl;
    std::cout << "ground_truth: x,y,vx,vy\n" << gt_values << std::endl;
    std::cout << "RMSE: x,y,vx,vy\n" << RMSE << std::endl;
  }
}


