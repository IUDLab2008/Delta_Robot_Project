#include "Stepper.h"

#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

TCA9548 tca9548(0x70);

//Configure common Driver PINs
byte MS[] = {65, 66, 67};
byte _reset_pin = 68;
byte _sleep_pin = 69;
byte _enable_pin = 64;

//Configure exclusive Driver PINs
byte _dir_pin_1 = 49;
byte _encoder_pin_1 = 61;
byte _step_pin_1 = 5;                             

byte _dir_pin_2 = 53;
byte _encoder_pin_2 = 60;
byte _step_pin_2 = 7;

byte _dir_pin_3 = 51;
byte _encoder_pin_3 = 59;
byte _step_pin_3 = 11;

// Configure common additional specifications of Stepper Motor
bool _operating_mode = 0;
float _speed_upper_bound = 120;
float _speed_lower_bound = 5;

// Configure exclusive additional specifications of Stepper Motor
float _angle_offset_1    = 0;
int _angle_upper_bound_1 = 360;
int _angle_lower_bound_1 = 0;
bool _rotate_direction_1 = 1;

float _angle_offset_2    = 0;
int _angle_upper_bound_2 = 360;
int _angle_lower_bound_2 = 0;
bool _rotate_direction_2 = 1;

float _angle_offset_3    = 0;
int _angle_upper_bound_3 = 360;
int _angle_lower_bound_3 = 0;
bool _rotate_direction_3 = 1;

// Configure necessary specifications for Constant Speed mode
int _step = 200;
byte _mode = 1;
int _rpm = 50;

// Global Stepper Object
_Stepper_Motor stepper1(_step_pin_1, _dir_pin_1, _enable_pin, _mode, _step, 0, _rotate_direction_1);
_Stepper_Motor stepper2(_step_pin_2, _dir_pin_2, _enable_pin, _mode, _step, 1, _rotate_direction_2);
_Stepper_Motor stepper3(_step_pin_3, _dir_pin_3, _enable_pin, _mode, _step, 2, _rotate_direction_3);

void _UART_init(unsigned int _baud_settings)
{
    /*
        Summary: This Module configures the UART setting
        Args:
            unsigned int _baud_settings: Value of BaudRate
        Output:
            None
    */
    UBRR0H = (unsigned char)(_baud_settings >> 8);
    UBRR0L = (unsigned char)(_baud_settings);
    
    UCSR0A  = 0;                                                                                    //Reset UCSR0A register
    UCSR0A |=  (1 << UDRE0);                                                                        //Set to indicate Transmit Buffer is empty, ready to be used
    UCSR0A &= ~(1 << U2X0);

    UCSR0B |=  ( (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) );                                     //Enable Receive/Transmit + TX/RX Complete Interrupt + USART Data Register Empty Interrupt Enable

    UCSR0C &= ~( (1 << UMSEL01) | (1 << UMSEL00) | (1 << UPM00) | (1 << USBS0) );                   //Enable Asynchronous USART + Even Parity mode  
    UCSR0C |=  ( (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00) );                                    //Enable Even Parity mode + 1-bit Stop bit mode + 8-bit Frame  

}

void _UART_Flush(void)
{
    /*
        Summary: This Module flush excessive data left in UDR0 register
        Args: 
            None
        Output:
            None
    */
    unsigned char dummy;
    while ( UCSR0A & ( 1 << RXC0 ) )
    {
        dummy = UDR0;
    }

}

void _UART_Transmit(unsigned char data)
{
    /*
        Summary: This Module transmits one byte of data
        Args: 
            unsigned char data: One byte of Data
        Output:
            None
    */
    while ( !(UCSR0A & (1 << UDRE0)) )
    {
        ;
    }
    UDR0 = data;    

}

unsigned char _UART_Receive(void)                                                                          
{
    /*
        Summary: This Module receives one byte of data
        Args: 
            None
        Output:
            None
    */
    while ( !( UCSR0A & ( 1 << RXC0 ) ) )
    {
        ;
    }
    return UDR0;

}

void _Transmit_Float(float _data)                                                                               
{
    /*
        Summary: This Module transmits 4 bytes of data (Float)
        Args: 
            float _data: One float Data
        Output:
            None
    */
    char index = 3;
    uint8_t* ptr  = (uint8_t* )&_data;

    while(index >= 0)
    {
        _UART_Transmit ( *(ptr + index) );
        index --;
    }

}

void _UART_ISR_handle(float& _angle)                                                           
{
    /*
        Summary: This Module receives 4 bytes of data (Float)
                 Execute in USART0 Rx for receving float data type
        Args: 
            float& _angle: Stores value
        Output:
            None
    */
    char index = 3;
    uint8_t* ptr = (uint8_t*)&_angle;

    while (index >= 0)
    {
        *(ptr + index) = _UART_Receive();
        index --;
    }

}




/***** MAIN PROGRAM *****/

void setup() {
    Wire.begin();
    _UART_init(MYUBRR);

    /***** Stepper initializing phase *****/
    stepper1._configure_additional_specifications(_angle_offset_1, _angle_upper_bound_1, _angle_lower_bound_1, _speed_upper_bound, _speed_lower_bound, tca9548);
    stepper2._configure_additional_specifications(_angle_offset_2, _angle_upper_bound_2, _angle_lower_bound_2, _speed_upper_bound, _speed_lower_bound, tca9548);
    stepper3._configure_additional_specifications(_angle_offset_3, _angle_upper_bound_3, _angle_lower_bound_3, _speed_upper_bound, _speed_lower_bound, tca9548);


    /***** Stepper Timer Enable */
    stepper1._Timer_enable();
    stepper1._Set_rpm(_rpm);

    stepper2._Timer_enable();
    stepper2._Set_rpm(_rpm);

    stepper3._Timer_enable();
    stepper3._Set_rpm(_rpm);

}


ISR(USART0_RX_vect)                                     
{
    _UART_ISR_handle(stepper1._desired_value);
    _UART_ISR_handle(stepper2._desired_value);
    _UART_ISR_handle(stepper3._desired_value);

    char terminator = _UART_Receive();

    stepper1._Set_Direction(tca9548);
    stepper1._Timer_re_enable();

    stepper2._Set_Direction(tca9548);
    stepper2._Timer_re_enable();

    stepper3._Set_Direction(tca9548);
    stepper3._Timer_re_enable();

}

ISR(TIMER4_COMPA_vect)
{
    stepper1._ISR_execute_Angle(tca9548);
    stepper2._ISR_execute_Angle(tca9548);
    stepper3._ISR_execute_Angle(tca9548);

}

ISR(TIMER3_COMPA_vect)
{
    ;
}

ISR(TIMER1_COMPA_vect)
{
    ;
}

void loop() {

    _Transmit_Float(stepper1._current_angle);
    _Transmit_Float(stepper2._current_angle);
    _Transmit_Float(stepper3._current_angle);

    delay(1000);
}
