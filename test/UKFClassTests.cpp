// #include "gtest/gtest.h"
// #include "tools.h"
// #include "ukf.h"
// #include<iostream>
// using Eigen::VectorXd;
// using Eigen::MatrixXd;
// using namespace std;

// TEST(TestUKFClass, UKFClassLaserTest)
// {
//   vector<VectorXd> estimations;
//   vector<VectorXd> ground_truth;

//   MeasurementPackage meas_package;
//   meas_package.sensor_type_ = MeasurementPackage::LASER;
//   meas_package.raw_measurements_ = VectorXd(2);
//   meas_package.raw_measurements_ << 0.3122427,	0.5803398;
//   meas_package.timestamp_ = 1477010443000000;

//   VectorXd gt(4);
//   gt << 0.6, 0.6, 5.19994, 0;
//   ground_truth.push_back(gt);

//   UKF ukf;
//   ukf.processMeasurement(meas_package);
//   VectorXd state_mean = ukf.getStateMean();
//   VectorXd estimate = tools::stateToEstimate(state_mean);
//   estimations.push_back(estimate);

//   // cout << "After initilization" << endl;
//   // ukf.print();
//   // cout << "ground truth\n" << gt << endl;

//   meas_package.raw_measurements_ << 1.173848,	4.810729e-01;
//   meas_package.timestamp_ = 1477010443100000;	
//   gt << 1.119984,	0.6002246, 5.199429, 5.389957e-03;
//   ground_truth.push_back(gt);


//   ukf.processMeasurement(meas_package);
//   state_mean = ukf.getStateMean();
//   estimate = tools::stateToEstimate(state_mean);
//   estimations.push_back(estimate);

//   // cout << "First update" << endl;
//   // ukf.print();
//   // cout << "Ground truth\n" << gt << endl;
// }


// TEST(TestUKFClass, UKFClassRadarTest)
// {
//   vector<VectorXd> estimations;
//   vector<VectorXd> ground_truth;

//   MeasurementPackage meas_package_laser;
//   meas_package_laser.sensor_type_ = MeasurementPackage::LASER;
//   meas_package_laser.raw_measurements_ = VectorXd(2);
//   meas_package_laser.raw_measurements_ << 0.3122427,	0.5803398;
//   meas_package_laser.timestamp_ = 1477010443000000;

//   VectorXd gt(4);
//   gt << 0.6, 0.6, 5.19994, 0;
//   ground_truth.push_back(gt);

//   UKF ukf;
//   ukf.processMeasurement(meas_package_laser);
//   VectorXd state_mean = ukf.getStateMean();
//   VectorXd estimate = tools::stateToEstimate(state_mean);
//   estimations.push_back(estimate);

//   // cout << "After initilization" << endl;
//   // ukf.print();
//   // cout << "ground truth\n" << gt << endl;
  
//   MeasurementPackage meas_package_radar;
//   meas_package_radar.raw_measurements_ = VectorXd(3);
//   meas_package_radar.raw_measurements_ << 1.014892,	0.5543292,	4.892807;
//   meas_package_radar.sensor_type_ = MeasurementPackage::RADAR;
//   meas_package_radar.timestamp_ = 1477010443050000;	
//   gt << 0.859997,  0.600045,   5.19975, 0.00179686;
//   ground_truth.push_back(gt);

//   ukf.processMeasurement(meas_package_radar);
//   state_mean = ukf.getStateMean();
//   estimate = tools::stateToEstimate(state_mean);
//   estimations.push_back(estimate);

//   cout << "After one iteration" << endl;
//   ukf.print();
//   cout << "Ground truth\n" << gt.transpose() << endl;
// }