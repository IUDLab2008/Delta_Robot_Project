#ifndef ENCODERFEEDBACK_H
#define ENCODERFEEDBACK_H

#include <Arduino.h>
#include "AS5600.h"
#include "TCA9548.h"
#include "Wire.h"

#include "Stepper.h"

// Convert the raw angle to the shaft angle
#define RAW_TO_ANGLE 360/4096/13.7

class _EncoderFeedback
{
    private:
    public:
        _EncoderFeedback(float offset1, float offset2, float offset3)
        {
            // Create instances of AS5600 Encoders and TCA9548 Multiplexer
            AS5600 encoder1;
            AS5600 encoder2;
            AS5600 encoder3;

            TCA9548 tca9548(0x70);
            
            Wire.begin();

            // Automatic encoder calibration after initialization
            tca9548.selectChannel(0);
            delayMicroseconds(50);
            encoder1.setOffset(offset1);
            delayMicroseconds(50);

            tca9548.selectChannel(1);
            delayMicroseconds(50);
            encoder1.setOffset(offset2);
            delayMicroseconds(50);

            tca9548.selectChannel(2);
            delayMicroseconds(50);
            encoder1.setOffset(offset3);
            delayMicroseconds(50);            
        }

        // Handle Encoder angle feedback, stored received data in each stepper current angle
        void _readEncoder1(_Stepper_Motor& _stepperInstance1);
        void _readEncoder2(_Stepper_Motor& _stepperInstance2);
        void _readEncoder3(_Stepper_Motor& _stepperInstance3);

};

#endif