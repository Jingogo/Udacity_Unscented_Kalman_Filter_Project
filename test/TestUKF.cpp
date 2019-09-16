// #include "gtest/gtest.h"
// #include "tools.h"
// #include "ukf.h"
// using Eigen::VectorXd;
// using Eigen::MatrixXd;
// using std::vector;

// TEST(CalculateRMSE, TestRMSECalulation)
// {
//     VectorXd estimate(4);
//     estimate << 2, 3, 1, 5;

//     vector<VectorXd> estimations;
//     estimations.push_back(estimate);

//     VectorXd gt_values(4);
//     gt_values << 5, 4, 1, 2;
//     vector<VectorXd> ground_truth;
//     ground_truth.push_back(gt_values);

//     VectorXd expected_RMSE(4);
//     expected_RMSE << 3, 1, 0, 3;
//     VectorXd actual_RMSE = tools::calculateRMSE(estimations, ground_truth);

//     bool is_equal = expected_RMSE.isApprox(actual_RMSE);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFInitWeights, TestInitWeights)
// {
//     UKF ukf;
//     VectorXd expected_weights = VectorXd(15);
//     expected_weights << -2./5, 0.1, 0.1, 0.1, 0.1,
//                          0.1, 0.1, 0.1, 0.1, 0.1,
//                          0.1, 0.1, 0.1, 0.1, 0.1;
    
//     VectorXd actual_weights = ukf.getWeights();
//     bool is_equal = expected_weights.isApprox(actual_weights);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFInitialization, TestInitializationLaser)
// {
//     UKF ukf;
//     VectorXd measurement = VectorXd(2);
//     measurement << 3.0, 4.0;
//     ukf.setMeasurement(measurement);
//     ukf.initialization();
   
//     VectorXd expected_state_mean = VectorXd(5);
//     MatrixXd expected_state_cov = MatrixXd(5,5);
//     expected_state_mean << 3.0, 4.0, 0, 0, 0;
//     expected_state_cov << 1, 0, 0, 0, 0,
//         0, 1, 0, 0, 0,
//         0, 0, 1, 0, 0,
//         0, 0, 0, 1, 0,
//         0, 0, 0, 0, 1;
//     VectorXd actual_state_mean = ukf.getStateMean();
//     MatrixXd actual_state_cov = ukf.getStateCov();
    
//     bool is_equal = expected_state_mean.isApprox(actual_state_mean);
//     EXPECT_EQ(true, is_equal );
//     is_equal = expected_state_cov.isApprox(actual_state_cov);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFInitialization, TestInitializationRadar)
// {
//     UKF ukf;
//     VectorXd measurement = VectorXd(3);
//     measurement << 5, M_PI/3, 4;
//     ukf.setMeasurement(measurement);
//     MeasurementPackage::SensorType sensor_type = MeasurementPackage::RADAR;
//     ukf.setSensorType(sensor_type);
//     ukf.initialization();
   
//     VectorXd expected_state_mean = VectorXd(5);
//     MatrixXd expected_state_cov = MatrixXd(5,5);
//     expected_state_mean << 2.5, 2.5*std::sqrt(3), 0, 0, 0;
//     expected_state_cov <<
//         1, 0, 0, 0, 0,
//         0, 1, 0, 0, 0,
//         0, 0, 1, 0, 0,
//         0, 0, 0, 1, 0,
//         0, 0, 0, 0, 1;
//     VectorXd actual_state_mean = ukf.getStateMean();
//     MatrixXd actual_state_cov = ukf.getStateCov();

//     bool is_equal = expected_state_mean.isApprox(actual_state_mean);
//     EXPECT_EQ(true, is_equal );
//     is_equal = expected_state_cov.isApprox(actual_state_cov);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFGenerateAugmentedStateSig, TestGenerateAugmentedStateSig)
// {
//     UKF ukf;
//     VectorXd measurement(2);
//     measurement << 3.0, 4.0;
//     ukf.setMeasurement(measurement);
//     ukf.initialization();
    
//     MatrixXd expected_sig(7, 15);
//     expected_sig << 
//         3,   5, 3, 3, 3, 3, 3,  3,   1, 3, 3,  3, 3,  3,  3,
//         4,   4, 6, 4, 4, 4, 4,  4,   4, 2, 4,  4, 4,  4,  4,
//         0,   0, 0, 2, 0, 0, 0,  0,   0, 0,-2,  0, 0,  0,  0,
//         0,   0, 0, 0, 2, 0, 0,  0,   0, 0, 0, -2, 0,  0,  0,
//         0,   0, 0, 0, 0, 2, 0,  0,   0, 0, 0,  0,-2,  0,  0,
//         0,   0, 0, 0, 0, 0, 60, 0,   0, 0, 0,  0, 0, -60, 0,
//         0,   0, 0, 0, 0, 0, 0, 60,   0, 0, 0,  0, 0,  0, -60;
//     ukf.setLambda(-1);
//     MatrixXd actual_sig = ukf.generateAugmentedStateSig();

//     bool is_equal = expected_sig.isApprox(actual_sig);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFUpdatePredStateSigTests, TestUpdatePredStateSig)
// {
//     MatrixXd state_sig(7, 15);
//     state_sig << 3,   5, 3, 3, 3, 3, 3,  3,   1, 3, 3,  3, 3,  3,  3,
//         4,   4, 6, 4, 4, 4, 4,  4,   4, 2, 4,  4, 4,  4,  4,
//         0,   0, 0, 2, 0, 0, 0,  0,   0, 0,-2,  0, 0,  0,  0,
//         0,   0, 0, 0, M_PI/6, 0, 0,  0,   0, 0, 0, -M_PI/3, 0,  0,  0,
//         0,   0, 0, 0, 0, M_PI/6, 0,  0,   0, 0, 0,  0, M_PI/3,  0,  0,
//         0,   0, 0, 0, 0, 0, 60, 0,   0, 0, 0,  0, 0, -60, 0,
//         0,   0, 0, 0, 0, 0, 0, 60,   0, 0, 0,  0, 0,  0, -60;

//     MatrixXd expected_pred_state_sig(5, 15);
    
//     UKF ukf;
//     ukf.updatePredStateSig(state_sig);
//     MatrixXd actual_pred_state_sig = ukf.getPredStateSig();
// }

// TEST(UKFUpdatePredStateMean, TestUpdatePredStateMean)
// {
//     MatrixXd pred_state_sig(5,15);
//     pred_state_sig <<         
//         3,   5, 3, 4, 3, 3, 3,  3,   1, 3, 3,  3, 3,  3,  3,
//         4,   4, 6, 4, 4, 4, 4,  4,   4, 2, 4,  4, 4,  4,  4,
//         0,   0, 0, 2, 5, 0, 0,  0,   0, 0,-2,  0, 0,  0,  0,
//         0,   0, 0, 0, 2, 0, 0,  0,   0, 0, 0, -2, 0,  0,  0,
//         0,   0, 0, 0, 0, 2, 0,  0,   0, 0, 0,  0,-2,  0,  0;

//     UKF ukf;    
//     ukf.setPredStateSig(pred_state_sig);
//     ukf.updatePredStateMean();
//     VectorXd actual_pred_state_mean = ukf.getStateMean();

//     VectorXd expected_pred_state_mean(5);
//     expected_pred_state_mean << 3.1, 4, 0.5, 0, 0;

//     bool is_equal = expected_pred_state_mean.isApprox(actual_pred_state_mean);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFUpdatePredStateCov, TestUpdatePredStateCov)
// {
//     MatrixXd pred_state_sig(3, 2);
//     pred_state_sig << 
//         3, 4,  
//         4, 5,   
//         1, 3;
//     VectorXd state_mean(3);
//     state_mean <<  2.0, 4.0, 0;
//     VectorXd weights(2);
//     weights << 0.2, 0.1;

//     UKF ukf;
//     ukf.setPredStateSig(pred_state_sig);
//     ukf.setStateMean(state_mean);
//     ukf.setWeights(weights);
//     ukf.updatePredStateCov(); 

//     MatrixXd actual_pred_state_cov = ukf.getStateCov();
    
//     MatrixXd expecetd_pred_state_cov(3,3);
//     expecetd_pred_state_cov << 
//         0.6, 0.2, 0.8,
//         0.2, 0.1, 0.3,
//         0.8, 0.3, 1.1;
    
//     bool is_equal = expecetd_pred_state_cov.isApprox(actual_pred_state_cov);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFUpdatePredMeasurementMeanLaser, TestUpdatePredMeasurementMeanLaser)
// {
//     VectorXd state_mean(5);
//     state_mean << 3, 4, 0, 3, 5;
//     UKF ukf;
//     ukf.setStateMean(state_mean);
//     ukf.updatePredMeasurementMeanLaser();
//     VectorXd actual_pred_measurement_mean = ukf.getPredMeasurementMean();

//     VectorXd expected_pred_measurement_mean(2);
//     expected_pred_measurement_mean << 3, 4;

//     bool is_equal = expected_pred_measurement_mean.isApprox(actual_pred_measurement_mean);
//     EXPECT_EQ(true, is_equal);
// }

// TEST(UKFUpdatePredMeasurementCovLaser, TestUpdatePredMeasurementCovLaser)
// {
//     MatrixXd state_cov(5,5);
//     state_cov <<
//         1, 2, 0, 0, 0,
//         2, 1, 0, 4, 0,
//         0, 0, 1, 0, 0,
//         0, 4, 0, 1, 0,
//         0, 0, 0, 0, 1;
//     UKF ukf;
//     ukf.setStateCov(state_cov);
//     ukf.updatePredMeasurementCovLaser();
//     MatrixXd actual_pred_measurement_cov = ukf.getPredMeasurementCov();

//     MatrixXd expected_pred_measurement_cov(2,2);
//     expected_pred_measurement_cov << 
//         1.0225, 2,
//         2, 1.0225;

//     bool is_equal = expected_pred_measurement_cov.isApprox(actual_pred_measurement_cov);
//     EXPECT_EQ(true, is_equal);    
// }

// TEST(UKFUpdateKalmanGainLaser, TestUpdateKalmanGainLaser)
// {
//     MatrixXd state_cov(5,5);
//     state_cov <<
//         1, 2, 0, 0, 0,
//         2, 1, 0, 0, 0,
//         0, 0, 1, 0, 0,
//         0, 0, 0, 1, 0,
//         0, 0, 0, 0, 1;
//     MatrixXd pred_measurement_cov(2,2);
//     pred_measurement_cov <<
//         1/3., 2/3.,
//         2/3., 1/3.;
//     UKF ukf;
//     ukf.setStateCov(state_cov);
//     ukf.setPredMeasurementCov(pred_measurement_cov);
//     ukf.updateKalmanGainLaser();
//     MatrixXd actual_kalman_gain = ukf.getKalmanGain();
   
//     MatrixXd expected_kalman_gain(5,2);
//     expected_kalman_gain << 
//         3, 0,
//         0, 3,
//         0, 0,
//         0, 0,
//         0, 0;
//     bool is_equal = expected_kalman_gain.isApprox(actual_kalman_gain);    
//     EXPECT_EQ(true, is_equal);    
// }

// TEST(UKFUpdatePredMeasurementSig, TestUpdatePredMeasurementSig)
// {
//     MatrixXd pred_state_sig(5,15);
//     pred_state_sig <<         
//         3,   5, 3, 0, 3, 3, 3,  3,   1, 3, 3,  3, 3,  3,  3,
//         4,   4, 6, 0, 4, 4, 4,  4,   4, 2, 4,  4, 4,  4,  4,
//         0,   0, 0, 0, 1, 0, 0,  0,   0, 0,-2,  0, 0,  0,  0,
//         0,   0, 0, 0, M_PI/2, 0, 0,  0,   0, 0, 0, M_PI, 0,  0,  0,
//         0,   0, 0, 0, 0, 2, 0,  0,   0, 0, 0,  0,-2,  0,  0;

//     UKF ukf;
//     ukf.setPredStateSig(pred_state_sig);
//     ukf.updatePredMeasurementSig();
//     MatrixXd actual_pred_measurement_sig = ukf.getPredMeasurementSig();
//     std::cout << actual_pred_measurement_sig.col(4)<<std::endl;
    
//     VectorXd expected_pred_measurement_sig(3);
//     expected_pred_measurement_sig << 5, 0.927295, 0.8;
 
//     bool is_equal = (expected_pred_measurement_sig- actual_pred_measurement_sig.col(4)).norm() < 1e-5;
//     EXPECT_EQ(true, is_equal);   
// }

