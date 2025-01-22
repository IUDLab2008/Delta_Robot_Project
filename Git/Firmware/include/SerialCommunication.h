#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <Arduino.h>
#include <ArduinoSTL.h>
#include <vector>
#include <queue>

#include "GCodeReceiver.h"

using namespace std;

class UART {
    private:
        volatile bool endOfTransmission = 0;
    public:
    /*
        Args:
            endOfTransmission:  Determine whether the first-time transaction of the GCode have been fully executed yet
    */
        UART(unsigned int baudRate);
        void UARTFlush(void) const;
        void UARTTransmit(const unsigned char data);
        unsigned char UARTReceive (void) const;
        void transmitFloat(const float data);
        void ISRAngleHandle(float& angle);
        void ISRInstructionHandle(queue<String>& instructionSet, GCodeReceiver GCodeReceiverInstance);
        inline bool getEndOfTransmission() const { return endOfTransmission; }
};

#endif 