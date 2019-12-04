#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include "measurementPackage.h"

namespace tools
{
  Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &ground_truth);
  void addEpsIfZero(double &div);
  void normalizeRadarMeasurement(Eigen::VectorXd &measurement);
  MeasurementPackage readMeasurement(std::istringstream &iss);
  Eigen::VectorXd readGroundTruth(std::istringstream &iss);
  Eigen::VectorXd stateToEstimate(const Eigen::VectorXd &stateMean);
  double compareExpectedToActual(const Eigen::VectorXd &expected, const Eigen::VectorXd &actual);
  double compareExpectedToActual(const Eigen::MatrixXd &expected, const Eigen::MatrixXd &actual);
} // namespace tools

#endif // TOOLS_H_
