#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensorType_;

  long long timestamp_;

  Eigen::VectorXd rawMeasurements_;
};

#endif // MEASUREMENT_PACKAGE_H_