#include "Stepper.h"

Stepper::Stepper (byte _ORDER, bool _rotateDirection)
{
    /*
        Summary:
            This constructor function initializes the Motor with specifications
        Args:
            _ORDER: The order of the Motor, i.e     0 <-> 1st Motor
            _rotateDirection: The default rotating Direction of the Motor.
    */
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
    /*
        Summary:
            This function set-up Timer with CTC mode configuration but not yet set-up the TOP value to 
            temporarily pause the activity of the Timer
        Args:
            None
    */
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
    /*
        Summary:
            This function temporarily disable the Timer output action by simply disable the corresponding output pin configuration
        Args:
            None
    */
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
    /*
        Summary:
            This function re-enable the Timer output action by simply enable the corresponding output pin configuration
        Args:
            None
    */
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

void Stepper::setRPM(float _rpm)
{
    /*
        Summary:
            This function configures the RPM of the Motor by adjust the TOP Value for Timer matching.
            Also, this adjust the rotating direction accordingly.
        Args:
            None
    */
    int COUNTER_TOP = ceil(( ( 16e6 * 60 / (8L * _rpm  * this -> mode * this -> stepPerRev) ) - 1 ) / 4);
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

    this -> rpm = _rpm;

    if ( _rpm > 0 )
    {
        if (!this -> rotateDirection)
        {
            digitalWrite(this -> DIR_PIN, HIGH);                                                        
        } else {
            digitalWrite(this -> DIR_PIN, LOW);                                                        
        }

    } else {

        if (!this -> rotateDirection)
        {
            digitalWrite(this -> DIR_PIN, LOW);                                                        
        } else {
            digitalWrite(this -> DIR_PIN, HIGH);                                                        
        }

    }     
}

void Stepper::setAngle(float _desiredAngle)
{   
    this -> desiredAngle = _desiredAngle;
    this -> timerReEnable();
}

void Stepper::HomingISRAngleExecute()
{   
    /*
        Summary:
            This function handles the first-time Homing procedure of the Motor by moving at predefined HOMING_RPM
            until the angle is between (-angularResolution/2, angularResolution/2) then clear the Homing State flag 
            to indicate Homing is over.
        Args:
            None
    */
    if (std::abs(analogRead(this -> ENCODER_PIN) * RAW_TO_ANGLE) < this -> angularResolution)
    {
        this -> timerDisable();
        this -> isHoming = 0;
    }
}

void Stepper::ISRAngleExecute()
{
    /*
        Summary:
            This function update the ellapsed steps made since the beginning of GCode processing.
        Args:
            None
    */
    if (this -> rpm)
    {
        if (this -> rpm > 0)
        {
            this -> ellapsedInterrupt ++;
        } else {
            this -> ellapsedInterrupt --;
        }
    }
}

inline float Stepper::getAngle() const noexcept
{
    return this -> ellapsedInterrupt * RAW_TO_ANGLE;
}