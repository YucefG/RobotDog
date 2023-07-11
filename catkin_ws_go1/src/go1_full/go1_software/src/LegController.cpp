#include "../include/LegController.h"

// upper level of joint controller 
// send data to joint controller
using namespace unitree_model;
/*!
 * Zero leg command
 */ 
void LegControllerCommand::zero(){
    tau = Vec3<double>::Zero();
    qDes = Vec3<double>::Zero();
    qdDes = Vec3<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat3<double>::Zero();
    kdJoint = Mat3<double>::Zero();
}

/*!
 * Zero leg data
 */ 
void LegControllerData::zero(){
    q = Vec3<double>::Zero();
    qd = Vec3<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J = Mat3<double>::Zero();
    tau = Vec3<double>::Zero();
}

void LegController::publishPosCmd(){
    //lowCmd_pub = n.advertise<laikago_msgs::LowCmd>("/laikago_gazebo/LowCmd/Command",1);
    double targetPos[12];
    for (int i = 0; i<4; i++){
        for (int j = 0; j<3; j++){
        lowCmd.motorCmd[i*3+j].q = commands[i].qDes(j);
        //std::cout << commands[i].qDes(j) << std::endl;
        lowCmd.motorCmd[i*3+j].dq = commands[i].qdDes(j);
        lowCmd.motorCmd[i*3+j].tau = commands[i].tau(j);
        targetPos[i*3+j] = commands[i].qDes(j);
        }
    }
    moveAllPosition(targetPos, 2*1000);
}

void LegController::zeroCommand(){
    for (int i = 0; i<4; i++){
        commands[i].zero();
    }
}

void LegController::updateData(){
    for (int leg = 0; leg < 4; leg++){
        for(int j = 0; j<3; j++){
            data[leg].q(j) = lowState.motorState[leg*3+j].q;
            data[leg].qd(j) = lowState.motorState[leg*3+j].dq;
            data[leg].tau(j) = lowState.motorState[leg*3+j].tauEst;
        }

        computeLegJacobianAndPosition(_quadruped, data[leg].q,&(data[leg].J),&(data[leg].p),leg);

         // v
        data[leg].v = data[leg].J * data[leg].qd;
    }
    //std::cout << "data updated" << std::endl;
}

void LegController::updateCommand(){

    for (int i = 0; i <4; i++){
        computeLegJacobianAndPosition(_quadruped, data[i].q,&(data[i].J),&(data[i].p),i);
        // tauFF
        //commands[i].tau = Vec3<double>::Zero();
        Vec3<double> legTorque = commands[i].tau;
        // std::cout << "commmand" << commands[i].tau << std::endl;
        // forceFF

        Vec3<double> footForce = commands[i].feedforwardForce;

        footForce +=
            commands[i].kpCartesian * (commands[i].pDes - data[i].p);
         
        footForce +=
            commands[i].kdCartesian * (commands[i].vDes - data[i].v);
       // std::cout << "leg: " << i << std::endl;
       // std::cout << footForce << std::endl;
        // torque
        legTorque = data[i].J.transpose() * footForce;
        //std::cout << data[i].J << std::endl;
        commands[i].tau = legTorque;
        for (int j = 0; j<3; j++){
            lowCmd.motorCmd[i*3+j].tau = commands[i].tau(j);
            
         //std::cout << "motor torque cmd: " << lowCmd.motorCmd[i*3+j].torque << std::endl;
         //std::cout << commands[i].tau(j) << std::endl;
        }
    }
    sendServoCmd();
    //std::cout << "cmd sent" << std::endl;
   
}

void LegController::updateCommandNoSend(){

    for (int i = 0; i <4; i++){
        computeLegJacobianAndPosition(_quadruped, data[i].q,&(data[i].J),&(data[i].p),i);
        // tauFF
        //commands[i].tau = Vec3<double>::Zero();
        Vec3<double> legTorque = commands[i].tau;
        // std::cout << "commmand" << commands[i].tau << std::endl;
        // forceFF

        Vec3<double> footForce = commands[i].feedforwardForce;

        footForce +=
            commands[i].kpCartesian * (commands[i].pDes - data[i].p);
         
        footForce +=
            commands[i].kdCartesian * (commands[i].vDes - data[i].v);
       // std::cout << "leg: " << i << std::endl;
       // std::cout << footForce << std::endl;
        // torque
        legTorque = data[i].J.transpose() * footForce;
        //std::cout << data[i].J << std::endl;
        commands[i].tau = legTorque;
        for (int j = 0; j<3; j++){
            lowCmd.motorCmd[i*3+j].tau = commands[i].tau(j);
            
        //  std::cout << "motor torque cmd: " << lowCmd.motorCmd[i*3+j].tau << std::endl;
         //std::cout << commands[i].tau(j) << std::endl;
        }
    }
    //sendServoCmd();
    //std::cout << "cmd sent" << std::endl;
   
}

void computeLegJacobianAndPosition(Quadruped& _quad, Vec3<double>& q, Mat3<double>* J,Vec3<double>* p, int leg)
{
    double l1 = _quad.hipLinkLength; // ab_ad
    double l2 = _quad.thighLinkLength;
    double l3 = _quad.calfLinkLength;

    int sideSign = 1; // 1 for Left legs; -1 for right legs
    if (leg == 0 || leg == 2){
        sideSign = -1;
    }

    double s1 = std::sin(q(0));
    double s2 = std::sin(q(1));
    double s3 = std::sin(q(2));

    double c1 = std::cos(q(0));
    double c2 = std::cos(q(1));
    double c3 = std::cos(q(2));

    double c23 =  c2 * c3 - s2 * s3;
    double s23 =  s2 * c3 + c2 * s3; // sin(2+3))
   
   if(J){
    J->operator()(0, 0) = 0;
    J->operator()(1, 0) = -sideSign * l1 * s1 + l2 * c2 * c1 + l3 * c23 * c1;
    J->operator()(2, 0) = sideSign * l1 * c1 + l2 * c2 * s1 + l3 * c23 * s1;
    J->operator()(0, 1) = -l3 * c23 - l2 * c2;
    J->operator()(1, 1) = -l2 * s2 * s1 - l3 * s23 * s1;
    J->operator()(2, 1) = l2 * s2 * c1 + l3 * s23 * c1;
    J->operator()(0, 2) = -l3 * c23;
    J->operator()(1, 2) = -l3 * s23 *s1;
    J->operator()(2, 2) = l3 * s23 * c1;   
   }

   if(p){
    p->operator()(0) = -l3 * s23 - l2 * s2;
    p->operator()(1) = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
   }
}

/* Different from master branch (bug)
 * see: https://github.com/OpenQuadruped/spot_mini_mini/blob/spot/spotmicro/Kinematics/LegKinematics.py
 *
 */
void computeInverseKinematics(Quadruped& _quad, Vec3<double>& pDes, int leg, Vec3<double>* qDes)
{
    double l1 = _quad.hipLinkLength; // ab_ad
    double l2 = _quad.thighLinkLength;
    double l3 = _quad.calfLinkLength;

    double x = pDes[0];
    double y = pDes[1];
    double z = pDes[2];

    // get domain
    double D = (y*y + z*z - l1*l1 + x*x - l2*l2 - l3*l3) / (2*l3*l2);
    if (D > 1)
        D = 1;
    if (D < -1)
        D = -1;
    // D = std::clamp(D, -1, 1);

    // check right vs left leg for hip angles
    int sideSign = 1; // 1 for Left legs; -1 for right legs
    if (leg == 0 || leg == 2){
        sideSign = -1;
    }

    // right leg inverse kinematics solver
    double wrist_angle = atan2(-sqrt(1-D*D), D);
    double sqrt_component = y*y + z*z - l1*l1;
    if (sqrt_component < 0){
        sqrt_component = 0;
    }
    double shoulder_angle = -atan2(z,y) - atan2(sqrt(sqrt_component), sideSign*l1);
    double elbow_angle = atan2(-x, sqrt(sqrt_component)) - atan2(l3*sin(wrist_angle), l2+l3*cos(wrist_angle));

    qDes->operator()(0) = -shoulder_angle; // was negative
    qDes->operator()(1) = elbow_angle;
    qDes->operator()(2) = wrist_angle;

}