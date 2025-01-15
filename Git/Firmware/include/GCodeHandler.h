//This file is intended for performing various task on handling fed-in GCode Instructions. Its attributes involve _G_Code_Queue, a 
//FIFO queue which stores provided G-Code Instructions, ready to be translated. And, _Segment, a Queue with each of its element being a 3-D
//vector consists of namely, _angle_1 , _angle_2 and _angle_3.
//The _G_Code_Handler 's methods include _G_Code_Execute(), a function which parses the GCode Instruction into key values: Type of Motion (Translating / Rotating),
//if Rotating determine its rotating direction and its reference center; Coordinate of the End Position. Moreover, _Segment_Update() translates those key values into
//invidual angles via _Interpolation class, then, update the _Segment  


#ifndef GCODE_H
#define GCODE_H

#include <Arduino.h>
#include <cppQueue.h>

#include "Interpolation.h"

#define IMPLEMENTATION FIFO

struct _element
{
    float _angle_1;
    float _angle_2;
    float _angle_3;

    float& get(int index)
    {
        switch (index)
        {
        case 1: return _angle_1;
        case 2: return _angle_2;
        case 3: return _angle_3;
        }

    }
}; 

class _G_Code_Handler 
{
    private:

    public:

        cppQueue _G_Code_Queue; 
        cppQueue _Segment;      
        
        _G_Code_Handler() 
            : _G_Code_Queue(sizeof(String), 16, IMPLEMENTATION, false), 
              _Segment(sizeof(_element), 10000, IMPLEMENTATION, false) {}

        void _G_Code_Queue_Update(const String &_Instruction);
        void _Segment_Update();
        int* _G_Code_Execute(const String& input);
};

extern _G_Code_Handler _GCodeHandler;

#endif
