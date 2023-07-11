/************************************************************************

Hopf Polar CPG

************************************************************************/
#ifndef HOPFPOLAR_H
#define HOPFPOLAR_H

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <random>
#include <chrono>
#include "../include/cppTypes.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "../include/body.h"


using namespace std;
using namespace Eigen;

class HopfPolar
{
public:
    HopfPolar();
    double get_random();
    void init();
    /*
    Gaits:
    1 bounding
    2 trotting
    3 walking
    4 pacing;
    */
    void SetGait(int gaitNum );
    void update(Vec4<double>& x_out,Vec4<double>& z_out);
    void Integrate();

    MatrixXd X = MatrixXd::Zero(2,4);
    MatrixXd X_prev = MatrixXd::Zero(2,4);
    MatrixXd X_dot = MatrixXd::Zero(2,4);
    MatrixXd d2X = MatrixXd::Zero(1,4);

    double ground_clearance = 0.05;   
    double ground_penetration = 0.01;
    double des_step_len = 0.02; //0.05; 
    double h_max = 0.32; 
    double _a = 50; // amplitude convergence 
    double x_offset = -0.02;

    int LEG_INDICES[4] = { 0, 1, 2, 3};


private:

    double dt = 0.001;
    double mu = 1;

    // walk
    // test....
    // double omega_swing = 0.8*2*M_PI;
    // double omega_stance = 0.3*2*M_PI;
    // double omega_swing = 3.2*2*M_PI;
    // double omega_stance = 0.8*2*M_PI;
    // double omega_swing = 8*2*M_PI;
    // double omega_stance = 2*2*M_PI;

    // trot
    // double omega_swing = 4*2*M_PI;
    // double omega_stance = 2*2*M_PI;
    // double omega_swing = 5*2*M_PI;
    // double omega_stance = 2*2*M_PI;

    // slow and suboptimal
    double omega_swing = 4.2*2*M_PI;
    double omega_stance = 1.2*2*M_PI;   

    //bound
    // double omega_swing = 5*2*M_PI;
    // double omega_stance = 1.3*2*M_PI;
    
    /*
    Gaits:
    1 bounding
    2 trotting
    3 walking
    4 pacing;
    */
    int gait = 2;
    Mat4<double>PHI        = Mat4<double>::Zero();
    Mat4<double>K_tegotae  = Mat4<double>::Zero();
    double coupling_strength = 1;
    bool couple = true;
    bool tegotae_feedback = false;
    bool coupled_tegotae_feedback = false;

};

#endif // HOPFPOLAR_H