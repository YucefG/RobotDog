/*!
 * @file Quadruped.h
 * @brief stores dynamics information
 * Leg 0: Left front; Leg 1: right front;
 * Leg 2: Left rear ; Leg 3: right rear;
 */ 
#ifndef PROJECT_QUADRUPED_H
#define PROJECT_QUADRUPED_H

#include <vector>
#include "cppTypes.h"
class Quadruped{
  public:
    void setQuadruped(int robot_id){
        // 1 is Go1
        robot_index = robot_id;
        if(robot_id == 1){
            mass = 12.84;

            leg_offset_x = 0.1881;
            leg_offset_y = 0.04675;
            // leg_offset_z = -0.016; // not clear in repo

            hipLinkLength = 0.08;
            thighLinkLength = 0.213;
            calfLinkLength = 0.213;
        }
        // 2 is A1
        if(robot_id == 2){
            mass = 12;

            // updated April 13, 2022
            leg_offset_x = 0.1805; //0.183;
            leg_offset_y = 0.047;
            leg_offset_z = 0.01675; //-0.016;

            hipLinkLength = 0.0838;
            thighLinkLength = 0.2;
            calfLinkLength = 0.2;
        }
    }
    int robot_index; // 1 for Go1, 2 for A1
    double hipLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double mass;
    Vec3<double> getHipLocation(int leg){
        assert(leg >=0 && leg <4);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0){
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1){
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 2){
            pHip(0) = -leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 3){
            pHip(0) = -leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }

        return pHip;
    };

};

#endif