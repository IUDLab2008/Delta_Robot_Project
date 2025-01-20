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
    public:
        UART(unsigned int baudRate);
        void UARTFlush(void);
        void UARTTransmit(unsigned char data);
        unsigned char UARTReceive(void);
        void transmitFloat(float data);
        void ISRAngleHandle(float& angle);
        void ISRInstructionHandle(queue<String>& instructionSet, GCodeReceiver GCodeReceiverInstance);
};

#endif 