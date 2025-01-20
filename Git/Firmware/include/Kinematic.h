#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "Constant.h"
#include "Arduino.h"
#include "Data.h"

using namespace std;

class Kinematic {
    private:

    public:
        bool checkAngleValidation(float angle1, float angle2, float angle3);
        bool checkPointValidation(float x, float y, float z);
        void forwardKinematic(float angle1, float angle2, float angle3, float& x, float& y, float& z);
        void inverseKinematic(float& angle1, float& angle2, float& angle3, float x, float y, float z);
        void auxiliaryFunction(float x, float y, float z, float& angle);
        void calculateSubVelocities(float absoluteVelocity, float x0, float y0, float z0, float x1, float y1, float z1, float& xDot, float& yDot, float& zDot);
        element calculateAngularVelocities(float x, float y, float z, float xDot, float yDot, float zDot);
        
};

#endif 