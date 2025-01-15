//This function is intended to perform Interpolation, which parses the movement into smaller movements because of computational resources
// and linearization for fine motion.


#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <math.h>
#include "Kinematic.h"
#include "Data.h"
#include "GCodeHandler.h"

class _Interpolation
{

    private:
    struct _element
    {
        float _angle_1;
        float _angle_2;
        float _angle_3;
    };

    public:

        float _Distance(float _x_0, float _y_0, float _z_0, float _x_1, float _y_1, float _z_1);
        bool _Linear_Interpolation(float _x_1, float _y_1, float _z_1);
        bool _Circle_Interpolation(float x, float y, float _x_f, float _y_f, bool _rotating_direction);

};

extern _Interpolation Interpolation;

#endif 