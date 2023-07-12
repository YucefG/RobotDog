/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */


#ifndef PROEJCT_STATE_ESTIMATOR_CONTAINER_H
#define PROJECT_STATE_ESTIMATOR_CONTAINER_H

#pragma once

#include "CurrentState.h"
#include "LegController.h"
#include <sensor_msgs/Imu.h>
#include "unitree_legged_msgs/IMU.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#include "body.h"

using namespace unitree_model;

/*!
 * "Cheater" state sent to robot from simulation
 */
struct CheaterState{
  Quat<double> orientation;
  Vec3<double> position;
  Vec3<double> omegaBody;
  Vec3<double> vBody;
  Vec3<double> acceleration;
};

/*!
 * Result of state estimation
 */

struct StateEstimate {
    Vec4<double> contactEstimate;
    Vec3<double> position;
    Vec3<double> vBody;
    Quat<double> orientation;
    Vec3<double> omegaBody;
    RotMat<double> rBody;
    Vec3<double> rpy;

    Vec3<double> omegaWorld;
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;

   // ros::NodeHandle n;

   // ros::Publisher state_pub;
    
  //  void setROSmsg(){
        /*!
        state_pub = n.advertise<laikago_msgs::HighState>("/laikago_gazebo/highState/state",1);
        highState.forwardPosition.x = position(0);
        highState.forwardPosition.y = position(1);
        highState.forwardPosition.z = position(2);
        highState.forwardSpeed = vBody(0);
        highState.sideSpeed = vBody(1);
        highState.updownSpeed = vBody(2);

        for (int i = 0; i < 3; i++){
            highState.imu.rpy[i] = rpy(i);
            //lowstate.imu.rpy[i] = rpy(i); // rpy, orientation, accel already stored in lowState
            highState.imu.acceleration[i] = aBody(i);
            //lowstate.imu.acceleration[i] = aBody(i);
        }

        for (int i = 0; i < 4; i++){
            highState.imu.quaternion[i] = orientation[i];
            //lowstate.imu.quaternion[i] = orientation[i];
        }
        state_pub.publish(highState);

    }
    */
};


/*!
 * input for state estimation
 */ 
struct StateEstimatorData {
    StateEstimate* result;
    unitree_legged_msgs::IMU* imuData;
    LegControllerData* legControllerData;
    Vec4<double>* contactPhase;
};

/*!
 * All Estimators inherit from this class
 */ 
class GenericEstimator{
  public:
    virtual void run() = 0;
    virtual void setup() = 0;

    void setData(StateEstimatorData data) {_stateEstimatorData = data;};

    virtual ~GenericEstimator() = default;
    StateEstimatorData _stateEstimatorData;
};

/*!
 * Main State Estimator class
 * Contains all GenericEstimator
 */ 
class StateEstimatorContainer {
  public:
    // Constructor
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateEstimatorContainer(unitree_legged_msgs::IMU* _imu_data,
                            LegControllerData* _legControllerData,
                            StateEstimate* stateEstimate){
        _data.imuData = _imu_data;
        _data.legControllerData = _legControllerData;
        _data.result = stateEstimate;
        _phase = Vec4<double>::Zero();
        _data.contactPhase = &_phase;
    }
    // deconstructor
    ~StateEstimatorContainer() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
    }
    // run estimator
    void run(){
        for (auto estimator : _estimators){
            estimator->run();
        }
    }

    // get result
    const StateEstimate&  getResult() {return *_data.result;}

    // set result
    void setStateEstimate(StateEstimate& newEstimate) { *_data.result = newEstimate; }

    // set contact phase
    void setContactPhase(Vec4<double>& phase) { *_data.contactPhase = phase; }

    // add estimator of given type
    template <typename EstimatorToAdd>
    void addEstimator(){
        std::cout << "add estimator" << std::endl;
        auto* estimator = new EstimatorToAdd();
        estimator->setData(_data);
        estimator->setup();
        _estimators.push_back(estimator);
    }

    // remove estimator of given type
    template <typename EstimatorToRemove>
    void removeEstimator() {
        int nRemoved = 0;
        _estimators.erase(
            std::remove_if(_estimators.begin(), _estimators.end(),
                           [&nRemoved](GenericEstimator* e){
                               if (dynamic_cast<EstimatorToRemove*>(e)){
                                   delete e;
                                   nRemoved++;
                                   return true;
                                } else {
                                    return false;
                                }
                           }),
            _estimators.end());
    }

    // remove all estimators
    void removeAllEstimators() {
        for (auto estimator : _estimators) {
            delete estimator;
        }
        _estimators.clear();
    }
    //StateEstimatorData _data;
  private:
    std::vector<GenericEstimator*> _estimators;
    Vec4<double> _phase;
    StateEstimatorData _data;
};


#endif