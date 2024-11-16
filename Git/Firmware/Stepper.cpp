#include "Stepper.h"
#include <Arduino.h>


// This is a simple implementation of constant-speed Stepper motor Angle control using Angle feedback from a Rotary Encoder (here, we use Potentiometer instead)
// If it is nessessary to implement Speed control, changing the Timer mode to CTC Mode 4, hence, offer double-buffer - which is suitable for varying frequency implementation
// but, we have to change the Output Compare Pin to OC3[B:C] instead of using OC3A - which is now occupied for Matching value.

// For Constant Speed mode (Angle mode), the desired Angular Speed is received directly from available Communication Protocols (UART, I2C,...) and is executed right after, no need
// for storing values of desired Angular Speed for realtime operating.
 

/**** CHECKED ****/
_Stepper_Motor::_Stepper_Motor(byte MS[], byte mode, byte reset_pin, byte sleep_pin, byte enable_pin, byte dir_pin, byte encoder_pin, int step, byte step_pin, bool rotate_dir)
{

    this -> _mode = mode;
    this -> _step = step;
    this -> _res  = 1;

    this -> _reset_pin   = reset_pin;
    this -> _sleep_pin   = sleep_pin;
    this -> _enable_pin  = enable_pin;
    this -> _encoder_pin = encoder_pin;
    this -> _step_pin    = step_pin;
    this -> _dir_pin     = dir_pin;

    this -> _timer_state = 0;
    this -> _rotate_direction = rotate_dir;

    pinMode(this -> MS[0], OUTPUT);
    pinMode(this -> MS[1], OUTPUT);
    pinMode(this -> MS[2], OUTPUT);

    pinMode(this -> _reset_pin, OUTPUT);
    pinMode(this -> _sleep_pin, OUTPUT);
    pinMode(this -> _enable_pin, OUTPUT);
    pinMode(this -> _dir_pin, OUTPUT);
    pinMode(this -> _encoder_pin, INPUT);

    switch (this -> _mode)
    {
        case 1:
            digitalWrite(this -> MS[0], 0);
            digitalWrite(this -> MS[1], 0);
            digitalWrite(this -> MS[2], 0);
            break;
        case 2:
            digitalWrite(this -> MS[0], 1);
            digitalWrite(this -> MS[1], 0);
            digitalWrite(this -> MS[2], 0);
            break;
        case 4:
            digitalWrite(this -> MS[0], 0);
            digitalWrite(this -> MS[1], 1);
            digitalWrite(this -> MS[2], 0);
            break;
        case 8:
            digitalWrite(this -> MS[0], 1);
            digitalWrite(this -> MS[1], 1);
            digitalWrite(this -> MS[2], 0);
            break;
        case 16:
            digitalWrite(this -> MS[0], 1);
            digitalWrite(this -> MS[1], 1);
            digitalWrite(this -> MS[2], 1);
            break;
    }

    digitalWrite(this -> _enable_pin, LOW);
    digitalWrite(this -> _sleep_pin, HIGH);
    digitalWrite(this -> _reset_pin, HIGH);

    if (this -> _rotate_direction)
    {
        digitalWrite(this -> _dir_pin, 1);
    } else {
        digitalWrite(this -> _dir_pin, 0);
    }

}



/**** CHECKED ****/
void _Stepper_Motor::_configure_additional_specifications(float angle_offset, int angle_upper_bound, int angle_lower_bound, float speed_upper_bound, float speed_lower_bound)
{
    this -> _angle_offset      = angle_offset;

    this -> _angle_upper_bound = angle_upper_bound;
    this -> _angle_lower_bound = angle_lower_bound;
    this -> _speed_upper_bound = speed_upper_bound;
    this -> _speed_lower_bound = speed_lower_bound;

}



/**** CHECKED ****/
void _Stepper_Motor::_Timer_enable(void)
{
    pinMode(this -> _step_pin, OUTPUT);
    this -> _timer_state = 1;    
    
    switch (this -> _step_pin)
    { 
        case 11:
        case 12:
        case 13:
            TCCR1A = TCCR1B = TCNT1 = 0;
            TIMSK1 = 0;
            PRR0   &= ~(1 << PRTIM1);
            TCCR1A &= ~( (1 << WGM10) | (1 << WGM11) );
            TCCR1B |=  ( (1 << WGM13) | (1 << WGM12) );

            TCCR1B |=  (1 << CS11);
            TCCR1B &= ~( (1 << CS12)  | (1 << CS10) );

            TIMSK1 |=  (1 << OCIE1A);
            break;
        case 5:
        case 2:
        case 3:
            TCCR3A = TCCR3B = TCNT3 = 0;
            TIMSK3 = 0;
            PRR1   &= ~(1 << PRTIM3);
            TCCR3A &= ~( (1 << WGM30) | (1 << WGM31) );
            TCCR3B |=  ( (1 << WGM33) | (1 << WGM32) );

            TCCR3B |=  (1 << CS31);
            TCCR3B &= ~( (1 << CS32)  | (1 << CS30) );

            TIMSK3 |=  (1 << OCIE3A);
            break;
        case 6:
        case 7:
        case 8:
            TCCR4A = TCCR4B = TCNT4 = 0;
            TIMSK4 = 0;
            PRR1   &= ~(1 << PRTIM4);
            TCCR4A &= ~( (1 << WGM40) | (1 << WGM41) );
            TCCR4B |=  ( (1 << WGM43) | (1 << WGM42) );

            TCCR4B |=  (1 << CS41);
            TCCR4B &= ~( (1 << CS42)  | (1 << CS40) );

            TIMSK4 |=  (1 << OCIE4A);
            break;
        case 44:
        case 45:
        case 46:
            TCCR5A = TCCR5B = TCNT5 = 0;
            TIMSK5 = 0;
            PRR1   &= ~(1 << PRTIM5);
            TCCR5A &= ~( (1 << WGM50) | (1 << WGM51) );
            TCCR5B |=  ( (1 << WGM53) | (1 << WGM52) );

            TCCR5B |=  (1 << CS51);
            TCCR5B &= ~( (1 << CS52)  | (1 << CS50) );

            TIMSK5 |=  (1 << OCIE5A);
            break;
    }

    switch (this -> _step_pin)
    {
        case 11:
            TCCR1A |= (1 << COM1A0);
            break;
        case 12:
            TCCR1A |= (1 << COM1B0);
            break;
        case 13:
            TCCR1A |= (1 << COM1C0);
            break;
        case 5:
            TCCR3A |= (1 << COM3A0);
            break;
        case 2:
            TCCR3A |= (1 << COM3B0);
            break;
        case 3:
            TCCR3A |= (1 << COM3C0);
            break;
        case 6:
            TCCR4A |= (1 << COM4A0);
            break;
        case 7:
            TCCR4A |= (1 << COM4B0);
            break;
        case 8:
            TCCR4A |= (1 << COM4C0);
            break;
        case 44:
            TCCR5A |= (1 << COM5A0);
            break;
        case 45:
            TCCR5A |= (1 << COM5B0);
            break;
        case 46:
            TCCR5A |= (1 << COM5C0);
            break;
    }
}

/**** CHECKED ****/
void _Stepper_Motor::_Timer_disable(void)                                                                    
{

    switch (this -> _step_pin)
    {
        case 11:
        case 12:
        case 13:    
                TCCR1A &= ~0b11111100;                                                       
            break;
        case 5:
        case 2:
        case 3:
                TCCR3A &= ~0b11111100;
            break;
        case 6:
        case 7:
        case 8:
                TCCR4A &= ~0b11111100;                                                       
            break;
        case 44:
        case 45:
        case 46:
                TCCR5A &= ~0b11111100;                                                       
            break;
    } 

    this -> _timer_state = 0;                                                                           //ReSetting this -> _timer_state to 1 means that Timer is being out of service 
    
}


/**** CHECKED ****/
void _Stepper_Motor::_Timer_re_enable(void)
{
    switch (this -> _step_pin)
    {
        case 11:
            TCCR1A |= (1 << COM1A0);
            break;
        case 12:
            TCCR1A |= (1 << COM1B0);
            break;
        case 13:
            TCCR1A |= (1 << COM1C0);
            break;
        case 5:
            TCCR3A |= (1 << COM3A0);
            break;
        case 2:
            TCCR3A |= (1 << COM3B0);
            break;
        case 3:
            TCCR3A |= (1 << COM3C0);
            break;
        case 6:
            TCCR4A |= (1 << COM4A0);
            break;
        case 7:
            TCCR4A |= (1 << COM4B0);
            break;
        case 8:
            TCCR4A |= (1 << COM4C0);
            break;
        case 44:
            TCCR5A |= (1 << COM5A0);
            break;
        case 45:
            TCCR5A |= (1 << COM5B0);
            break;
        case 46:
            TCCR5A |= (1 << COM5C0);
            break;
    }
    this -> _timer_state = 1;
}


/**** CHECKED ****/
void _Stepper_Motor::_Set_rpm(int rpm)
{
    if (rpm >= this -> _speed_lower_bound && rpm <= _speed_upper_bound)
    {
        this -> _rpm = rpm;
        int COUNTER_TOP = ( ( 16e6 * 60 / (8L * this -> _rpm  * this -> _mode * this -> _step) ) - 1 ) / 4;
        switch (this -> _step_pin)
        {
            case 11:
            case 12:
            case 13:
                ICR1 = COUNTER_TOP  ;
                break;
            case 5:
            case 2:
            case 3:
                ICR3 = COUNTER_TOP  ;
                break;
            case 6:
            case 7:
            case 8:
                ICR4 = COUNTER_TOP  ;
                break;
            case 44:
            case 45:
            case 46:
                ICR5 = COUNTER_TOP  ;
                break;
        }
    } else {
        ICR1 = 0;
        ICR3 = 0;
        ICR4 = 0;
        ICR5 = 0;                                                                           //Cần modify lại
    }
}


/**** CHECKED ****/
void _Stepper_Motor::_Set_Angle(float desired_value)
{
    if (desired_value >= this -> _angle_lower_bound && desired_value <= _angle_upper_bound)
    {
        this -> _desired_value = desired_value;
        this -> _Timer_re_enable();
    }
}



/**** CHECKED ****/
void _Stepper_Motor::_execute_Homing()                                                                        
{

    if (this -> _get_Angle())                                                                           //Check whether the current angle equals to 0. If yes, initialize a queue and put in front 0 for executing Homing 
    {
        
        this -> _desired_value = 0;

    }

    this -> _Timer_enable();                                                                            //Initialize Timer
    this -> _Set_rpm(10);                                                                               //Set RPM to physically Rotate the Stepper
    
    this -> _timer_state = 1;                                                                           //Setting this -> _timer_state to 1 means that Timer is being used 

    while (this -> _timer_state)                                                                        //While the _timer_state is on (hence, the value is 1), wait till the _timer_state is off (hence, the value is 0)
    {                                                                                                   //owing to |_desired_value - _get_Angle()| < _res
        ;
    }
    this -> _Timer_disable();
}



/**** CHECKED ****/
float _Stepper_Motor::_get_Angle(void)                                                                        //Return the Current angle value by Encoder
{

    return analogRead(this -> _encoder_pin ) * (300.0f / 1023.0f) - this -> _angle_offset;
    // float currentAngle = analogRead(this -> _encoder_pin) * (300.0f / 1023.0f) - this -> _angle_offset;

    //     // If the buffer is full, subtract the oldest value from the sum
    // if (count == WINDOW_SIZE) {
    //     this -> sumAngles -= this -> angleBuffer[bufferIndex];
    // } else {
    //     this -> count++; // Increase the count until the buffer is full
    // }

    // // Store the new angle in the buffer and add it to the sum
    // this -> angleBuffer[bufferIndex] = currentAngle;
    // this -> sumAngles += currentAngle;

    // // Update the buffer index to point to the next slot
    // this -> bufferIndex = (this -> bufferIndex + 1) % WINDOW_SIZE;

    // // Return the moving average
    // return this -> sumAngles / this -> count;
}


void _Stepper_Motor::_Set_Direction(void)
{

    if ( this -> _desired_value > this -> _get_Angle() )
    {
        if (!this -> _rotate_direction)
        {
            digitalWrite(this -> _dir_pin, HIGH);                                                        
        } else {
            digitalWrite(this -> _dir_pin, LOW);                                                        
        }

    } else {

        if (!this -> _rotate_direction)
        {
            digitalWrite(this -> _dir_pin, LOW);                                                        
        } else {
            digitalWrite(this -> _dir_pin, HIGH);                                                        
        }

    }                    
    //this -> _Timer_re_enable();
}


void _Stepper_Motor::_ISR_execute_Angle()
{
    if ( abs( this -> _get_Angle() - this -> _desired_value ) <= this -> _res )
    {
                                                    
        _Timer_disable();                   

    }

}
