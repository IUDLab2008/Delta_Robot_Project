//This file is intended for storing necessary datas, which are: Current position of End-effector; Desired angle of each invidual motor;
//MMs per segment made; Boundaries of Angles and Angular velocities   

#ifndef DATA_H
#define DATA_H

class _Data 
{
    private:
    public:
        
        float _current_x;                                                           //Store the current point coordinate
        float _current_y;
        float _current_z;

        float _desired_angle_1;                                                     //Store the desired point's angular position
        float _desired_angle_2;
        float _desired_angle_3;

        float _MM_per_Segment;                                                      //Store the smallest line per segment (in MM)

        int _angle_upper_bound;                                                     //Store the angular bounds and speed bounds
        int _angle_lower_bound;
        float _speed_upper_bound;
        float _speed_lower_bound;
     
};

extern _Data Data;

#endif 