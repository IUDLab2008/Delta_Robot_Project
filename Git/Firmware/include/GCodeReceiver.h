#ifndef GCODERECEIVER_H
#define GCODERECEIVER_H

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <queue>

#include "Data.h"
#include "Kinematic.h"
#include "Constant.h"

using namespace std;

class GCodeReceiver {
    private:
    public:
        queue<String> instructionQueue;                                         
        queue<bool> key;
        queue<float> XValue;
        queue<float> YValue;
        queue<float> ZValue;
        queue<float> FValue;
        queue<float> timeStep;
        bool validateInstruction(String instruction);
        void parseInstruction(queue<String>& instructionQueue);
};

#endif 