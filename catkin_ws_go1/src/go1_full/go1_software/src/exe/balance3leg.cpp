/************************************************************************

Test vanilla CPG. The "setup" is standard and can mostly be copied between
different files. You MUST try any control algorithms with the simulated
sensors in Gazebo BEFORE trying on the hardware.

************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <fstream>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>
#include "body.h"
#include "../include/CurrentState.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/ControlFSMData.h"
//##include "../../CPG/HopfPolar.h"
#include "../../include/Math/MathUtilities.h"

using namespace std;
using namespace unitree_model;

//global variable
Vec3<double> target_ee;


void stand3legs()
{
   // double pos[12] = {-0.2, -0.3, -2.0, -0.3, 0.67, -1.3, -0.3, 0.6, -1.3,
   //                   +0.3, 0.6, -1.3};

    //front left  feet in air 
    double pos1[12] = {0.2, 0.67, -1.3, -0.3, 0.67, -1.3, -0.1, 0.6, -1.5,
                    +0.1, 0.6, -1.5};

    //front right feet in air 
//    double pos1[12] = {0.3, 0.67, -1.3, 0.0, 0.67, -1.3, -0.1, 0.6, -1.5,
 //                     +0.1, 0.6, -1.5};

    // 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos1, 5000);

    //front left feet in air 
    double pos2[12] = {0.4, 0.67, -1.3, 0.1, 0.3, -2.0, -0.1, 0.6, -1.5,
                      +0.1, 0.6, -1.5};

    //front right feet in air 
    //double pos2[12] = {0.2, -0.3, -2.0, -0.3, 0.67, -1.3, -0.1, 0.6, -1.5,
    //                  +0.1, 0.6, -1.5};

    // 0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos2, 5000);
}

void endEffectorPositionsCallback(const geometry_msgs::PointConstPtr& msg) {
  // Update the end effector class attribute with the new end-effector positions.
  target_ee[0] = msg->x;
  target_ee[1] = msg->y;
  target_ee[2] = msg->z;
//  std::cerr << "ROS EE POSITION RECEIVED" <<std::endl;  
//  cout << target_ee << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go1_test_3leg");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    CurrentState listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one thread
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; // for rviz visualization
    ros::ServiceClient set_model_state_serv;

    //xxx peut etre trop lent
    ros::Rate loop_rate(1000);

    //suscribe to end effector position topic
    ros::Subscriber sub = n.subscribe("end_effector_positions", 1, endEffectorPositionsCallback);


    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    // add simulation reset, so we don't have to keep relaunching ROS if robot falls over
    set_model_state_serv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    motion_reset();
    usleep(1000000);
    gazebo_msgs::SetModelState model_state;
    model_state.request.model_state.model_name = robot_name + "_gazebo"; //"laikago_gazebo";
    model_state.request.model_state.pose.position.z = 0.5;
    // model_state.request.model_state.pose.position.x = -5;
    std::cout << "set model" << std::endl;
    std::cout << set_model_state_serv.call(model_state) << std::endl;
    usleep(1000000);

    // select Quadruped
    std::cout << "set quadruped" << std::endl;
    Quadruped quad;
    quad.setQuadruped(1); // 1 for Go1, 2 for A1

    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController *legController = new LegController(quad);
    legController->zeroCommand();
    sendServoCmd();

    // start state estimation
    std::cout << "start state estimate" << std::endl;
    StateEstimatorContainer *stateEstimator = new StateEstimatorContainer(
        &lowState.imu, legController->data, &stateEstimate);

    // change this to test algorithms using simulated sensors BEFORE trying on hardware
    if (1)
    {
        // cheat data (ground truth)
        stateEstimator->addEstimator<ContactEstimator>();
        stateEstimator->addEstimator<CheaterOrientationEstimator>();
        stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    }
    else
    {
        // using sensors
        stateEstimator->addEstimator<ContactEstimator>();
        stateEstimator->addEstimator<VectorNavOrientationEstimator>();
        stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
    }

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    ControlFSMData *_controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_legController = legController;

    // make the robot stand on 4 legs
 //   motion_init();
    cout << "end of standing" << endl
         << endl;
   // cin.get();
    // prepare the robot to stand on 3 legs
 
    cout << "end of standing on 3 legs" << endl
         << endl;


    legController->updateData();
    stateEstimator->run();

    // cpg related vectors
    // Vec4<double> x_out, z_out;
    double sideSign[4] = {-1, 1, -1, 1};
    double foot_y = 0.08;

    Mat3<double> kpJoint = Mat3<double>::Zero();
    Mat3<double> kdJoint = Mat3<double>::Zero();
    kpJoint.diagonal() << 100, 100, 100;
    kdJoint.diagonal() << 2, 2, 2;
    //kpJoint.diagonal() << 55, 55, 55;
    //kdJoint.diagonal() << 0.8, 0.8, 0.8;
    //  kpJoint.diagonal() << 15, 15, 15;
    //  kdJoint.diagonal() << 0.3, 0.3, 0.3;

    bool ADD_CARTESIAN_PD = false;
    Mat3<double> kpCartesian = Mat3<double>::Zero();
    Mat3<double> kdCartesian = Mat3<double>::Zero();
    //    kpCartesian.diagonal() << 500, 500, 500;
    //    kdCartesian.diagonal() << 10, 10, 10;
    kpCartesian.diagonal() << 50, 50, 50;
    kpCartesian.diagonal() << 2, 2, 2;


    // xxx: adapt to make one leg move one after the other. 
    stand3legs();
    //motion_reset();
    param_reset();

        

    // write out
    // int full_state_len = 76; // however many states you would like
    // std::ofstream full_traj("full_state_gazebo_cpg.txt");
    // std::vector<double> full_state(full_state_len);

    double pos[12] = {-0.0, 0.67, -1.3, 0.1, 0.3, -2.0, -0.1, 0.6, -1.5,
                      +0.1, 0.6, -1.5};

    int counter = 0;
    int i = 0; //durée d'un mouvement, relancé des qu'une nouvelle position arrive (faire dans callback du ros suscriber)

    double lastPos[12], percent;
    /*
    for(int j=0; j<12; j++) 
    {
        lastPos[j] = lowState.motorState[j].q;
        cout << lastPos[j] << endl;
    }*/
    
    //check if new ee position, without adding new global variable
    Vec3<double> target_ee_old = target_ee;


  //  bool new_ee = (target_ee != target_ee) ? true : false;

    while (ros::ok()) //&& counter < 10000)
    {
       

        //while to join i est the movement counter -> is reset when movement is 
        while(percent<0.95)
        {
        //    cout <<"TTTTTTTTTTTTTTTTTT"<<endl;
        //    cout << percent <<endl;
         //   cout << endl;
            
            // update current robot state
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();

            // control logic

            Vec3<double> pDes, qDes, tau;
            //pDes = {0.3, -0.1, -0.2};
            pDes = target_ee;
            cout << "target ee: " << pDes <<endl; 
            
            double duration = 1000;

            computeInverseKinematics(_controlData->_legController->_quadruped, pDes, 1, &qDes);
            tau = kpJoint * (qDes - _controlData->_legController->data[0].q) +
                kdJoint * (-_controlData->_legController->data[1].qd);

         //   cout << "Q desired <<" << qDes << endl;

            double joint_targets[12] = {pos[0], pos[1], pos[2],qDes[0], qDes[1], qDes[2], pos[6], pos[7], pos[8],
                            pos[9], pos[10], pos[11]};

          //  cin.get();
            
            if(!ros::ok()) break;

            percent = (double)i/duration;
           // cout << percent << endl;
           /* 
           for(int j=0; j<3; j++){
                    lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + qDes[j]*percent; 
                    lowCmd.motorCmd[j].dq = 0;
                    lowCmd.motorCmd[j].tau = 0;  //normalement 0 
                    lowCmd.motorCmd[j].Kp = kpJoint(j, j);
                    lowCmd.motorCmd[j].Kd = kdJoint(j, j);
           }*/

            for(int j=0; j<12; j++){
                if(j>=3 && j<6)
                {
                 //   cout << "motor " << j << " in leg control" <<endl; 
                    lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + qDes[j-3]*percent; 
//                    cout << "Kd"<< lowCmd.motorCmd[j].Kd << endl;

                    lowCmd.motorCmd[j].dq = 0;
                    lowCmd.motorCmd[j].tau = 0;  //normalement 0 
                    lowCmd.motorCmd[j].Kp = kpJoint(0, 0);
                    lowCmd.motorCmd[j].Kd = kdJoint(0, 0);
                }
                else
                {
                 //   cout << "motor " << j << " in standing control" <<endl; 
                    lowCmd.motorCmd[j].q = lastPos[j];
                    lowCmd.motorCmd[j].dq = 0;
                    lowCmd.motorCmd[j].tau = 0;  //normalement 0 
                    lowCmd.motorCmd[j].Kp = kpJoint(j/4, j/4);
                    lowCmd.motorCmd[j].Kd = kdJoint(j/4, j/4);
                }
           }

            for (int i = 0; i < 4; i++)
            {

                for (int j = 0; j < 3; j++)
                {
                    _controlData->_legController->commands[i].vDes[j] = 0;
                }

                /*
                if (ADD_CARTESIAN_PD)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        // add Cartesian space contribution
                        _controlData->_legController->commands[i].pDes[j] = pDes[j];
                        _controlData->_legController->commands[i].vDes[j] = 0;
                        _controlData->_legController->commands[i].kpCartesian(j, j) = kpCartesian(j, j);
                        _controlData->_legController->commands[i].kdCartesian(j, j) = kdCartesian(j, j);
                        _controlData->_legController->commands[i].feedforwardForce[j] = 0;
                    }
                    _controlData->_legController->updateCommandNoSend();
                }
                */
            }

            // [TODO] write a function to save data, by querying the below for body states ...
            // auto& seResult = _controlData->_stateEstimator->getResult();
            // ... and querying joints with lowState.motorState[jnt_idx].q... etc.
            // can also include CPG states for debugging

/*
            // send command to motors 
            for(int i=0;i<12;i++)
            {
                cout << "mot numero : "<< i<<endl;
                cout << "target of mot: "<< lastPos[i] <<endl;
                cout<< "target entering motor " << lowCmd.motorCmd[i].q<<endl;
            }
            //std::cout << "lowCmd.motorCmd[0]" <<  lowCmd.motorCmd[0].q  << std::endl;
*/
            sendServoCmd();
            //xxx test to add sleep?

            usleep(500);
            i++;
          //  if (i==duration) i=0;   //i never egal to duration because percent limited to 0.95
            loop_rate.sleep();
            counter++;
        }

        //fin du mouvement : sauvegarde des joints du moteur
        for(int j=3; j<6; j++) 
        {
        lastPos[j] = lowState.motorState[j].q;
        }
        
       //  cout << "newest target"<<endl;
       // cout << endl << target_ee <<endl;
       // cout << "old target"<<endl;
       // cout << target_ee_old <<endl;

        if((target_ee_old != target_ee)){
            i=0;
            percent=0;
      //      cout << endl << "NEW POS i RESET" << endl <<endl;
            target_ee_old = target_ee;
            }

        usleep(500);

    }

    std::cout << "finished" << std::endl;
    // delete quad;
    // delete &pyb_interface_obj;
    delete legController;
    delete stateEstimator;
    delete _controlData;

    return 0;
}
