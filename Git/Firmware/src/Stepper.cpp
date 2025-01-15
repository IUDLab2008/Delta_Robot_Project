#include "Stepper.h"
#include <Arduino.h>


// This is a simple implementation of constant-speed Stepper motor Angle control using Angle feedback from a Rotary Encoder (here, we use Potentiometer instead)
// If it is nessessary to implement Speed control, changing the Timer mode to CTC Mode 4, hence, offer double-buffer - which is suitable for varying frequency implementation
// but, we have to change the Output Compare Pin to OC3[B:C] instead of using OC3A - which is now occupied for Matching value.

// For Constant Speed mode (Angle mode), the desired Angular Speed is received directly from available Communication Protocols (UART, I2C,...) and is executed right after, no need
// for storing values of desired Angular Speed for realtime operating.
 

_Stepper_Motor::_Stepper_Motor(byte PUL_PIN, byte DIR_PIN, byte ENA_PIN, byte MODE, byte STEP, byte ORDER, bool rotate_dir)
{
    /*
        Summary: This module is to initialize a Stepper motor with its specifications
        Args:
            byte PUL_PIN: Pin which outputs Pulses for the motor to step
            byte DIR_PIN: Pin which determines the rotating direction of the motor
            byte ENA_PIN: Pin which switches between ON/OFF state of the motor
            byte MODE: Value of operating mode, i.e. 2 for Half-step, etc...
            byte STEP: Number of steps needed to complete one revolution
            bool rotate_dir: Current rotating direction of the motor, specified here to correct the real motion
            byte ORDER: Storing The Order of the Motor
        Output:
            None
    */
    this -> _mode = MODE * 13;
    this -> _step = STEP;
    this -> _res  = 0.1;
    this -> _order = ORDER;

    this -> _enable_pin  = ENA_PIN;
    this -> _step_pin    = PUL_PIN;
    this -> _dir_pin     = DIR_PIN;

    this -> _timer_state = 0;
    this -> _rotate_direction = rotate_dir;

    pinMode(this -> _enable_pin, OUTPUT);
    pinMode(this -> _dir_pin, OUTPUT);

    digitalWrite(this -> _enable_pin, HIGH);

    if (this -> _rotate_direction)
    {
        digitalWrite(this -> _dir_pin, 1);
    } else {
        digitalWrite(this -> _dir_pin, 0);
    }

}



/**** CHECKED ****/
void _Stepper_Motor::_configure_additional_specifications(float angle_offset, int angle_upper_bound, int angle_lower_bound, float speed_upper_bound, float speed_lower_bound, TCA9548 multiplexerInstance)
{
    /*
        Summary: This module specifies other settings of the motor
        Args:
            float _angle_offset: Calibrate the sensor after mounting
            int _angle_upper_bound, angle_lower_bound: Boundaries of angle values
            float _speed_upper_bound, _speed_lower_bound: Boundaries of angular velocities values
        Output:
            None
    */
    this -> _angle_upper_bound = angle_upper_bound;
    this -> _angle_lower_bound = angle_lower_bound;
    this -> _speed_upper_bound = speed_upper_bound;
    this -> _speed_lower_bound = speed_lower_bound;

    multiplexerInstance.selectChannel(this -> _order);
    delayMicroseconds(50);
    this -> encoder.setOffset(angle_offset);
    delayMicroseconds(50);

}


void _Stepper_Motor::_Timer_enable(void)
{
    /*
        Summary: This module initialize the Timer with its Output Compare Mode setting, but, without specifying COUNTER_TOP to prevent the Timer to function instantly
                 When the Timer is back-on-line, the _timer_state = 1
        Args:
            None
        Output:
            None
    */
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

void _Stepper_Motor::_Timer_disable(void)                                                                    
{
    /*
        Summary: This module enable the according Timer to function manually
                 When the Timer is offline, the _timer_state = 0
        Args:
            None
        Output:
            None
    */
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

void _Stepper_Motor::_Timer_re_enable(void)
{
    /*
        Summary: This module enable the according Timer to function manually
                 When the Timer is online, the _timer_state = 1
        Args:
            None
        Output:
            None
    */
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

void _Stepper_Motor::_Set_rpm(int rpm)
{
    /*
        Summary: This module specifies the motor RPM by calculate and assign the value of COUNTER_TOP to activate the Timer
                 Use this module one time, at the start of the program
        Args:
            int rpm: The absolute RPM of the shaft 
        Output:
            None
    */
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

void _Stepper_Motor::_Set_Angle(float desired_value)
{
    /*
        Summary: This module assign the given value for the motor to aim for.
                 After reaching the last angle, the Timer went off to hold the Shaft position. This Module re-enable the Timer.
        Args:
            float desired_value: Desired value to aim for
        Output:
            None 
    */
    if (desired_value >= this -> _angle_lower_bound && desired_value <= _angle_upper_bound)
    {
        this -> _desired_value = desired_value;
        this -> _Timer_re_enable();
    }
}

void _Stepper_Motor::_Set_Direction(TCA9548 multiplexerInstance)
{
    /*
        Summary: This modules set the rotating direction of the motor, corresponding to the relative position between the shaft and the desired value
        Args:
            _EncoderFeedback encoderInstance: Instance of the corresponding encoder
        Output:
            None 
    */
    multiplexerInstance.selectChannel(this -> _order);
    this -> _current_angle = this -> encoder.readAngle() / RAW_TO_ANGLE;
    if ( this -> _desired_value > this -> _current_angle )
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
}


void _Stepper_Motor::_ISR_execute_Angle(TCA9548 multiplexerInstance)
{
    /*
        Summary: This module is executed in each Interrupt Service Routine.
                 This module continuously checks whether the desired angle has been reached yet.
        Args:
            _EncoderFeedback encoderInstance: Instance of the corresponding encoder
        Output:
            None
    */
    multiplexerInstance.selectChannel(this -> _order);
    this -> _current_angle = this -> encoder.readAngle() / RAW_TO_ANGLE;
    if ( abs( this -> _current_angle - this -> _desired_value ) <= this -> _res )
    {
                                                    
        _Timer_disable();                   

    }
}
