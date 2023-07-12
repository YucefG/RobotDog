/*!
 * @file
 * @brief Orientation Estimation Algorithms
 * 
 * orientation: quaternion
 * rBody: transformation matrix( vBody = Rbody * vWorld)
 * omegaBody: angular vel in body frame
 * omegaWorld: ... in world frame
 * rpy: roll pitch yaw
 */

#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "StateEstimatorContainer.h"
#include "Math/orientation_tools.h"

/*!
 * Estimator for the IMU.  The VectorNav provides an orientation already and
 * we just retrive the infomation
 */
class VectorNavOrientationEstimator : public GenericEstimator {
 public:
  virtual void run();
  virtual void setup() {}
};

/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */

class CheaterOrientationEstimator : public GenericEstimator {
 public:
  virtual void run();
  virtual void setup() {}
};


// Not needed, but will be for Position/Velocity estimator
// /*!
//  * T265 estimator for orientation, just transforms back to base frame. 
//  *     NOTE: using lowState.cheat! TODO: fuse with IMU data?
//  */

// class T265OrientationEstimator : public GenericEstimator {
//  public:
//   virtual void run();
//   virtual void setup() {}
// };

#endif