#include "Stepper.h"

Stepper::Stepper (byte _ORDER, bool _rotateDirection)
{
    this -> mode = MODE;
    this -> stepPerRev = STEP;
    this -> angularResolution = ANGULAR_RESOLUTION;
    this -> order = _ORDER;

    this -> isRunning = 0;
    this -> rotateDirection = _rotateDirection;

    switch (this -> order)
    {
    case 0:
        this -> ENA_PIN = COMMON_ENA_PIN;
        this -> DIR_PIN = STEPPER_1_DIR_PIN;
        this -> STEP_PIN = STEPPER_1_PULSE_PIN;
        this -> ENCODER_PIN = ENCODER_1_PIN;

        this -> angleOffset = ANGLE_OFFSET_1;
        break;
    
    case 1:
        this -> ENA_PIN = COMMON_ENA_PIN;
        this -> DIR_PIN = STEPPER_2_DIR_PIN;
        this -> STEP_PIN = STEPPER_2_PULSE_PIN;
        this -> ENCODER_PIN = ENCODER_2_PIN;

        this -> angleOffset = ANGLE_OFFSET_2;
        break;

    case 2:
        this -> ENA_PIN = COMMON_ENA_PIN;
        this -> DIR_PIN = STEPPER_3_DIR_PIN;
        this -> STEP_PIN = STEPPER_3_PULSE_PIN;
        this -> ENCODER_PIN = ENCODER_3_PIN;

        this -> angleOffset = ANGLE_OFFSET_3;
        break;
    }

    pinMode(this -> ENA_PIN, OUTPUT);
    pinMode(this -> DIR_PIN, OUTPUT);
    pinMode(this -> ENCODER_PIN, INPUT);

    digitalWrite(this -> DIR_PIN, (this -> rotateDirection) ? 1 : 0);
}

void Stepper::timerEnable(void)
{
    pinMode(this -> STEP_PIN, OUTPUT);
    this -> isRunning = 1;
    switch (this -> order)
    {
    case 0:
        TCCR3A = TCCR3B = TCNT3 = 0;
        TIMSK3 = 0;
        PRR1   &= ~(1 << PRTIM3);
        TCCR3A &= ~( (1 << WGM30) | (1 << WGM31) );
        TCCR3B |=  ( (1 << WGM33) | (1 << WGM32) );

        TCCR3B |=  (1 << CS31);
        TCCR3B &= ~( (1 << CS32)  | (1 << CS30) );

        TIMSK3 |=  (1 << OCIE3A);
        TCCR3A |= (1 << COM3A0);
        break;

    case 1:
        TCCR4A = TCCR4B = TCNT4 = 0;
        TIMSK4 = 0;
        PRR1   &= ~(1 << PRTIM4);
        TCCR4A &= ~( (1 << WGM40) | (1 << WGM41) );
        TCCR4B |=  ( (1 << WGM43) | (1 << WGM42) );

        TCCR4B |=  (1 << CS41);
        TCCR4B &= ~( (1 << CS42)  | (1 << CS40) );

        TIMSK4 |=  (1 << OCIE4A);
        TCCR4A |= (1 << COM4B0);
        break;

    case 2:
        TCCR1A = TCCR1B = TCNT1 = 0;
        TIMSK1 = 0;
        PRR0   &= ~(1 << PRTIM1);
        TCCR1A &= ~( (1 << WGM10) | (1 << WGM11) );
        TCCR1B |=  ( (1 << WGM13) | (1 << WGM12) );

        TCCR1B |=  (1 << CS11);
        TCCR1B &= ~( (1 << CS12)  | (1 << CS10) );

        TIMSK1 |=  (1 << OCIE1A);
        TCCR1A |= (1 << COM1A0);
    }
}

void Stepper::timerDisable(void)
{
    switch (this -> order)
    {
    case 0:
        TCCR3A &= ~0b11111100;
        break;
    
    case 1:
        TCCR4A &= ~0b11111100;
        break;

    case 2:
        TCCR1A &= ~0b11111100; 
        break;
    }
    this -> isRunning = 0;
}

void Stepper::timerReEnable(void)
{
    switch (this -> order)
    {
    case 0:
        TCCR3A |= (1 << COM3A0);
        break;
    
    case 1:
        TCCR4A |= (1 << COM4B0);
        break;
    
    case 2:
        TCCR1A |= (1 << COM1A0);
        break;
    }
    this -> isRunning = 1;
}

void Stepper::setRPM(float rpm)
{
    int COUNTER_TOP = ceil(( ( 16e6 * 60 / (8L * rpm  * this -> mode * this -> stepPerRev) ) - 1 ) / 4);
    switch (this -> order)
    {
    case 0:
        ICR3 = COUNTER_TOP;
        break;
    
    case 1:
        ICR4 = COUNTER_TOP;
        break;

    case 2:
        ICR1 = COUNTER_TOP;
        break;
    }
}

void Stepper::setAngle(float _desiredAngle)
{   
    this -> desiredAngle = _desiredAngle;
    this -> timerReEnable();
}

void Stepper::ISRAngleExecute()
{
    if (!this -> numInterrupt.empty() && !this -> isHoming)
    {
        if (this -> rotateDirection)
        {
            this -> ellapsedInterrupt --;
        } else {
            this -> ellapsedInterrupt ++;
        }

        if (abs(this -> ellapsedInterrupt) >= numInterrupt.front())
        {
            this -> ellapsedInterrupt = 0;
            
            this -> numInterrupt.pop();
            this -> rpmQueue.pop();
            
            if (!this -> numInterrupt.empty())
            {
                this -> setRPM(this -> rpmQueue.front());
            } else {
                this -> timerDisable();
            }
        }
    }
}

void Stepper::HomingISRAngleExecute()
{   
    if (this -> isHoming == 1)
    {
        if (abs(analogRead(this -> ENCODER_PIN) * RAW_TO_ANGLE) < this -> angularResolution)
        {
            this -> timerDisable();
            this -> isHoming == 0;
        }
    }
}

void Stepper::instructionExecution(queue<int> _numInterrupt, queue<float> _rpmQueue)
{
    this -> numInterrupt = _numInterrupt;
    this -> rpmQueue = _rpmQueue;
}

float Stepper::getAngle()
{
    return analogRead(this -> ENCODER_PIN) * RAW_TO_ANGLE;
}