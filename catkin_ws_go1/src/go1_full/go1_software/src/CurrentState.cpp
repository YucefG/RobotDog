#include "../include/CurrentState.h"

using namespace unitree_model;

CurrentState::CurrentState(string rname) {
    robot_name = rname;
    imu_sub = nm.subscribe("/trunk_imu", 1, &CurrentState::imuCallback, this);
    state_sub = nm.subscribe("/gazebo/model_states", 1, &CurrentState::cheaterCallback, this);
    footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &CurrentState::FRfootCallback, this);
    footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &CurrentState::FLfootCallback, this);
    footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &CurrentState::RRfootCallback, this);
    footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &CurrentState::RLfootCallback, this);
    servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &CurrentState::FRhipCallback, this);
    servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &CurrentState::FRthighCallback, this);
    servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &CurrentState::FRcalfCallback, this);
    servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &CurrentState::FLhipCallback, this);
    servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &CurrentState::FLthighCallback, this);
    servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &CurrentState::FLcalfCallback, this);
    servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &CurrentState::RRhipCallback, this);
    servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &CurrentState::RRthighCallback, this);
    servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &CurrentState::RRcalfCallback, this);
    servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &CurrentState::RLhipCallback, this);
    servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &CurrentState::RLthighCallback, this);
    servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &CurrentState::RLcalfCallback, this);
}


void CurrentState::cheaterCallback(const gazebo_msgs::ModelStates& msg)
{

    // rotate from IMU to base origin... 
    lowState.cheat.orientation[0] = msg.pose[2].orientation.w;
    lowState.cheat.orientation[1] = msg.pose[2].orientation.x;
    lowState.cheat.orientation[2] = msg.pose[2].orientation.y;
    lowState.cheat.orientation[3] = msg.pose[2].orientation.z;

    lowState.cheat.position[0] = msg.pose[2].position.x;
    lowState.cheat.position[1] = msg.pose[2].position.y;
    lowState.cheat.position[2] = msg.pose[2].position.z;

    // lowState.cheat.vBody[0] = msg.twist[2].linear.x;
    // lowState.cheat.vBody[1] = msg.twist[2].linear.y;
    // lowState.cheat.vBody[2] = msg.twist[2].linear.z;
    lowState.cheat.vWorld[0] = msg.twist[2].linear.x;
    lowState.cheat.vWorld[1] = msg.twist[2].linear.y;
    lowState.cheat.vWorld[2] = msg.twist[2].linear.z;

    lowState.cheat.omegaBody[0] = msg.twist[2].angular.x;
    lowState.cheat.omegaBody[1] = msg.twist[2].angular.y;
    lowState.cheat.omegaBody[2] = msg.twist[2].angular.z;
}


void CurrentState::imuCallback(const sensor_msgs::Imu & msg)
{ 
    lowState.imu.quaternion[0] = msg.orientation.w;
    lowState.imu.quaternion[1] = msg.orientation.x;
    lowState.imu.quaternion[2] = msg.orientation.y;
    lowState.imu.quaternion[3] = msg.orientation.z;

    lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    // [TODO] check acceleration vs accelerometer? 
    lowState.imu.accelerometer[0] = msg.linear_acceleration.x; //accelerometer
    lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}


void CurrentState::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    // start_up = false;
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].q = msg.q;
    lowState.motorState[0].dq = msg.dq;
    lowState.motorState[0].tauEst = msg.tauEst;
}

void CurrentState::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].q = msg.q;
    lowState.motorState[1].dq = msg.dq;
    lowState.motorState[1].tauEst = msg.tauEst;
}

void CurrentState::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].q = msg.q;
    lowState.motorState[2].dq = msg.dq;
    lowState.motorState[2].tauEst = msg.tauEst;
}

void CurrentState::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    // start_up = false;
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].q = msg.q;
    lowState.motorState[3].dq = msg.dq;
    lowState.motorState[3].tauEst = msg.tauEst;
}

void CurrentState::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].q = msg.q;
    lowState.motorState[4].dq = msg.dq;
    lowState.motorState[4].tauEst = msg.tauEst;
}

void CurrentState::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].q = msg.q;
    lowState.motorState[5].dq = msg.dq;
    lowState.motorState[5].tauEst = msg.tauEst;
}

void CurrentState::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    // start_up = false;
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].q = msg.q;
    lowState.motorState[6].dq = msg.dq;
    lowState.motorState[6].tauEst = msg.tauEst;
}

void CurrentState::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].q = msg.q;
    lowState.motorState[7].dq = msg.dq;
    lowState.motorState[7].tauEst = msg.tauEst;
}

void CurrentState::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].q = msg.q;
    lowState.motorState[8].dq = msg.dq;
    lowState.motorState[8].tauEst = msg.tauEst;
}

void CurrentState::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    // start_up = false;
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].q = msg.q;
    lowState.motorState[9].dq = msg.dq;
    lowState.motorState[9].tauEst = msg.tauEst;
}

void CurrentState::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].q = msg.q;
    lowState.motorState[10].dq = msg.dq;
    lowState.motorState[10].tauEst = msg.tauEst;
}

void CurrentState::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].q = msg.q;
    lowState.motorState[11].dq = msg.dq;
    lowState.motorState[11].tauEst = msg.tauEst;
}

void CurrentState::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[0].x = msg.wrench.force.x;
    lowState.eeForce[0].y = msg.wrench.force.y;
    lowState.eeForce[0].z = msg.wrench.force.z;
    lowState.footForce[0] = msg.wrench.force.z;
}

void CurrentState::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[1].x = msg.wrench.force.x;
    lowState.eeForce[1].y = msg.wrench.force.y;
    lowState.eeForce[1].z = msg.wrench.force.z;
    lowState.footForce[1] = msg.wrench.force.z;
}

void CurrentState::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[2].x = msg.wrench.force.x;
    lowState.eeForce[2].y = msg.wrench.force.y;
    lowState.eeForce[2].z = msg.wrench.force.z;
    lowState.footForce[2] = msg.wrench.force.z;
}

void CurrentState::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[3].x = msg.wrench.force.x;
    lowState.eeForce[3].y = msg.wrench.force.y;
    lowState.eeForce[3].z = msg.wrench.force.z;
    lowState.footForce[3] = msg.wrench.force.z;
}



