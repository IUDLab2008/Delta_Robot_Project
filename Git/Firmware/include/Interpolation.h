#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "Kinematic.h"
#include "Constant.h"
#include "GCodeReceiver.h"
#include "Data.h"

#include "ArduinoSTL.h"
#include <vector>
#include <queue>
#include <array>

using namespace std;

class Interpolation {
    private:
    public:
        float distanceBet2Point(float x0, float y0, float z0, float x1, float y1, float z1);
        queue<element> linearInterpolation(Kinematic kinematicInstance, float x0, float y0, float z0, float x1, float y1, float z1);
        queue<element> convert2AngularVelocities(GCodeReceiver& GCodeReceiverInstance, Kinematic kinematicInstance);
        QueueSet convert2Angles(GCodeReceiver& GCodeReceiverInstance, Kinematic kinematicInstance);
};

#endif