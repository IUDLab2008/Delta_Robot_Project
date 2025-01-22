#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include <ArduinoSTL.h>

#include "Constant.h"
#include "PinOut.h"
#include "Data.h"
#include <queue>
#include <cmath>

using namespace std;

class Stepper {
    private:
    /*
        Args:
            rotateDirection: Specify the conventional Stepper initial Rotating Direction
                                0   -   ClockWise;
                                1   -   CounterClockWise;
            stepPerRev:      Specify the total Steps to complete one Revolution
            order     :      Specify the order of StepperMotor (0 - 1st Motor, 1 - 2nd Motor, 2 - 3rd Motor)
            ENA_PIN   :      Specify the Enable Pin of Driver
            DIR_PIN   :      Specify the Direction Pin of Driver
            STEP_PIN  :      Specify the Pulse Pin of Driver
            ENCODER_PIN:     Specify the input Pin of Potentiometer
            angularResolution: Specify the degree per one Step
            angleOffset:     Offset of feedback encoder due to mouting
            isRunning :      Store the operating state of Motor
            ellapsedInterrupt: Store the ellapsed number of Interrupt 
    */

        bool rotateDirection;  
        int stepPerRev;
        byte order;
        byte mode;
        byte ENA_PIN;
        byte DIR_PIN;
        byte STEP_PIN;
        byte ENCODER_PIN;
        float angularResolution;
        float angleOffset;
        volatile bool isRunning = 0;
        volatile float rpm = HOMING_RPM;
        volatile int ellapsedInterrupt = 0;
        volatile float desiredAngle = 0;
        volatile bool isHoming = 1;

    public:
        Stepper(const byte _ORDER, const bool _rotateDirection);
        void timerEnable(void);
        inline void timerDisable(void);
        inline void timerReEnable(void);
        inline float getAngle() const noexcept;
        inline void setRPM(const float rpm);
        bool getIsHoming() const { return isHoming; }
        inline void setTimeStep(const float timeStep);
        void setAngle(const float _desiredAngle);
        void setDirection(const bool direction);
        inline void ISRAngleExecute();
        inline void HomingISRAngleExecute();      
};

#endif