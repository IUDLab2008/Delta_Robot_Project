#include "Interpolation.h"


float _Interpolation::_Distance(float _x_0, float _y_0, float _z_0, float _x_1, float _y_1, float _z_1)                     //Computing the distance (in MM) between (x0, y0, z0) and (x1, y1, z1)                                  
{

    return sqrt((_x_0 - _x_1) * (_x_0 - _x_1) + (_y_0 - _y_1) * (_y_0 - _y_1) + (_z_0 - _z_1) * (_z_0 - _z_1) );

}

bool _Interpolation::_Linear_Interpolation(float _x_1, float _y_1, float _z_1)                                              //Perform Linear Interpolation, in which discretizes the path from current position to (x1, y1, z1)
{                                                                                                                           //in to smaller parts. For each step, update the value of element [t][i] in _Segment
    if ( !Kinematics._Check_Point_Validation(_x_1, _y_1, _z_1) )                                                            //Check whether given point is in predescribed workspace. If no, abort the function
    {

        return false;

    }

    int _Number_of_Segment = this -> _Distance(Data._current_x, Data._current_y, Data._current_z, _x_1, _y_1, _z_1) / Data._MM_per_Segment;        //Computing number of segments (steps to go) in the entire path

    if ( _Number_of_Segment < 0.3 && _Number_of_Segment > -0.3 )                                                            //If the total step number is too small, abort the function                                                    
    {

        return false;

    }

    for (int i = 0; i <= _Number_of_Segment; i++)                                                                           //For each step, calcute the new desired angle values and update them in the _Segment
    {
        
        float _buffer   = i / _Number_of_Segment;
        float _x_buffer = (1 - _buffer) * Data._current_x + _buffer * _x_1;
        float _y_buffer = (1 - _buffer) * Data._current_y + _buffer * _y_1;
        float _z_buffer = (1 - _buffer) * Data._current_z + _buffer * _z_1;

        float temp1 = 0;
        float temp2 = 0;
        float temp3 = 0;

        Kinematics._Inverse_Kinematic(_x_buffer, _y_buffer, _z_buffer, temp1, temp2, temp3);

        _element temp = {temp1, temp2, temp3}; 
        _GCodeHandler._Segment.push(&temp);
    }

    return true;
}

bool _Interpolation::_Circle_Interpolation(float i, float j, float _x_f, float _y_f, bool _rotating_direction)              
{

    if (i == 0 && j == 0)                                                                                                   //If the reference center's relative coordinate w.r.t current position equals (0, 0), abort the function
    {
        return false;
    }

    float _radius = sqrt(i * i + j * j);                                                                                    //Compute the relative radius
    float _o_x    = Data._current_x + i;                                                                                    //Compute the reference center's absolute XY-coordinate
    float _o_y    = Data._current_y + j;
    

    float _current_xy_angle = acosf(-i / _radius);                                                                          //Compute the relative angle between the line , which connects current position and reference center, and the x-axis
    if (j > 0)
    { 
        _current_xy_angle = -_current_xy_angle;
    }


    float _desired_xy_angle = acosf((_x_f - _o_x) / _radius);                                                               //Compute the relative angle between the line , which connects desired position and reference center, and the x-axis
    if (_y_f - _o_y <= 0)
    {
        _desired_xy_angle = -_desired_xy_angle;
    }

    float _Angular_Distance = _current_xy_angle - _desired_xy_angle;
    if ( ( _Angular_Distance < 0.3 ) && ( _Angular_Distance > -0.3) )                                                       //If the Angular Distance is too small or equals to 0, assign the Angular Distance to Full-Round Travel
    {
        _Angular_Distance = _rotating_direction ? -2.0 * pi : 2.0 * pi;
    }

    if ( (_current_xy_angle < _desired_xy_angle) && _rotating_direction)                
    {

    }

    float _number_of_arc_segment = _Angular_Distance * _radius / Data._MM_per_Segment;                                      //Compute the total of Segment made and Angular step per segment
    int _number_of_segment = _Angular_Distance / _number_of_arc_segment;
    float _Rad_per_Segment = Data._MM_per_Segment / _radius;


    for (int t = 0; t <= _number_of_segment; t++)
    {

        float dx = Data._current_x * _radius * cos(_Rad_per_Segment) - Data._current_y * _radius * sin(_Rad_per_Segment);
        float dy = Data._current_y * _radius * cos(_Rad_per_Segment) + Data._current_x * _radius * sin(_Rad_per_Segment);

        float temp1 = 0;
        float temp2 = 0;
        float temp3 = 0;

        Kinematics._Inverse_Kinematic(dx, dy, Data._current_z, temp1, temp2, temp3);                                        //For each step, calcute the new desired angle values and update them in the _Segment

        _element temp = {temp1, temp2, temp3}; 
        _GCodeHandler._Segment.push(&temp);
    }

    return true;

}

_Interpolation Interpolation;