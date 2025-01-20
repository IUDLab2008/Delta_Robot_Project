#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
#include <ArduinoSTL.h>

#include "Constant.h"
#include "PinOut.h"
#include "Data.h"
#include <queue>

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
            numInterrupt:    Store the number of Interrupt after fetching GCode
            rpmQueue:        Store the angular velocities after fetching GCode
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
        bool isRunning;
        bool isHoming = 1;

        int ellapsedInterrupt = 0;
        queue<int> numInterrupt;
        queue<float> rpmQueue;

        float desiredAngle = 0;

    public:
        Stepper(byte _ORDER, bool _rotateDirection);
        void timerEnable(void);
        void timerDisable(void);
        void timerReEnable(void);
        float getAngle();
        void setRPM(float rpm);
        void instructionExecution(queue<int> _numInterrupt, queue<float> _rpmQueue);
        void setTimeStep(float timeStep);
        void setAngle(float _desiredAngle);
        void setDirection(bool direction);
        void ISRAngleExecute();
        void HomingISRAngleExecute();      
};

#endif