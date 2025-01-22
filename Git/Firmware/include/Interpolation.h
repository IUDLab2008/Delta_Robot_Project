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
        inline float distanceBet2Point(const float x0, const float y0, const float z0, const float x1, const float y1, const float z1) const;
        queue<element> linearInterpolation(Kinematic kinematicInstance, const float x0, const float y0, const float z0, const float x1, const float y1, const float z1);
        queue<element> convert2AngularVelocities(GCodeReceiver& GCodeReceiverInstance, Kinematic kinematicInstance);
        QueueSet convert2Angles(GCodeReceiver& GCodeReceiverInstance, Kinematic kinematicInstance);
};

#endif