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
    /*
        Args:
            instructionQueue:   Store the unprocessed GCode instructions
            key:   Store the key after prefix 'G', i.e 1 for G1
            XValue: Store the value after prefix 'X'
            XValue: Store the value after prefix 'Y'
            XValue: Store the value after prefix 'Z'
            XValue: Store the value after prefix 'F'
            timeStep: Store the value to fully execute the current GCode instruction
    */
        queue<String> instructionQueue;                                         
        queue<bool> key;
        queue<float> XValue;
        queue<float> YValue;
        queue<float> ZValue;
        queue<float> FValue;
        queue<float> timeStep;
        inline bool validateInstruction(const String instruction) const;
        void parseInstruction(queue<String>& instructionQueue);
};

#endif 