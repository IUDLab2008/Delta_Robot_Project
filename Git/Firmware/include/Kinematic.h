#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <math.h>
#include "Data.h"

const float e = 115.0;                                                                                              //Real parameters of the Delta Robot, given by CAD model
const float f = 457.3;                                                                                              //      rf : Base Radius           -        re : End-effector Radius
const float re = 232.0;                                                                                             //      f  : Forearm length        -        e  : Lowerarm length
const float rf = 112.0;

const float sqrt3 = sqrt(3.0);  
const float pi = 3.141592653;    
const float sin120 = sqrt3/2.0;   
const float cos120 = -0.5;        
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

class _Kinematic{
    private:
    
    public:

    void _Forward_Kinematic(float _angle_1, float _angle_2, float _angle_3, float &x, float &y, float &z);          //Calculating the Forward Kinematics of Delta Robot

    void _Auxiliary_Function(float x, float y, float z, float &_angle);                                             //An auxiliary function which later supports computing Inverse Kinematics of Delta Robot 

    void _Inverse_Kinematic(float &_angle_1, float &_angle_2, float &_angle_3, float x, float y, float z);          //Calculating the Inverse Kinematics of Delta Robot

    bool _Check_Point_Validation(float _angle_1, float _angle_2, float _angle_3);                                   //Checking whether a point is in the workspace.
                                                                                                                    //Returned value is 1(true) or 0(false)

};

extern _Kinematic Kinematics;

#endif