#include "Stepper.h"
#include <Arduino.h>


#define FOSC 16000000UL// Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1



/***** Configure common Driver PINs *****/
byte MS[] = {65, 66, 67};
byte _reset_pin = 68;
byte _sleep_pin = 69;
byte _enable_pin = 64;

/***** Configure exclusive Driver PINs *****/
byte _dir_pin_1 = 49;
byte _encoder_pin_1 = 61;
byte _step_pin_1 = 5;                              //mau xanh tren cung

byte _dir_pin_2 = 53;
byte _encoder_pin_2 = 60;
byte _step_pin_2 = 7;

byte _dir_pin_3 = 51;
byte _encoder_pin_3 = 59;
byte _step_pin_3 = 11;

/***** Configure common additional specifications of Stepper Motor *****/
bool _operating_mode = 0;
float _speed_upper_bound = 120;
float _speed_lower_bound = 5;

/***** Configure exclusive additional specifications of Stepper Motor *****/

float _angle_offset_1    = 39.5894;
int _angle_upper_bound_1 = 360;
int _angle_lower_bound_1 = 0;
bool _rotate_direction_1 = 1;

float _angle_offset_2    = 191.7888 + 22.2874 - 44.2815 + 82.1114 -84.4574;
int _angle_upper_bound_2 = 360;
int _angle_lower_bound_2 = 0;
bool _rotate_direction_2 = 1;

float _angle_offset_3    = 0;
int _angle_upper_bound_3 = 360;
int _angle_lower_bound_3 = 0;
bool _rotate_direction_3 = 1;


/***** Configure necessary specifications for Constant Speed mode *****/
int _step = 200;
byte _mode = 1;
int _rpm = 10;




/***** Global Stepper Object *****/
_Stepper_Motor stepper1(MS, _mode, _reset_pin, _sleep_pin, _enable_pin, _dir_pin_1, _encoder_pin_1, _step, _step_pin_1, _rotate_direction_1);
_Stepper_Motor stepper2(MS, _mode, _reset_pin, _sleep_pin, _enable_pin, _dir_pin_2, _encoder_pin_2, _step, _step_pin_2, _rotate_direction_2);
_Stepper_Motor stepper3(MS, _mode, _reset_pin, _sleep_pin, _enable_pin, _dir_pin_3, _encoder_pin_3, _step, _step_pin_3, _rotate_direction_3);




/**** UART *****/
void _UART_init(unsigned int _baud_settings)
{
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

    unsigned char dummy;
    while ( UCSR0A & ( 1 << RXC0 ) )
    {
        dummy = UDR0;
    }

}

void _UART_Transmit(unsigned char data)
{

    while ( !(UCSR0A & (1 << UDRE0)) )
    {
        ;
    }
    UDR0 = data;    

}

unsigned char _UART_Receive(void)                                                                          
{

    while ( !( UCSR0A & ( 1 << RXC0 ) ) )
    {
        ;
    }
    return UDR0;

}

void _Transmit_Float(float _data)                                                                               
{
    
    uint8_t* ptr  = (uint8_t* )&_data;

    for (int i = 0; i < 4; i++)
    {
        _UART_Transmit (*(ptr + i));
    }
}

void _UART_ISR_handle(float& _angle)                                                            //Execute in USART0 Rx for receving float data type
{

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
    _UART_init(MYUBRR);

    /***** Stepper initializing phase *****/
    // stepper1._configure_additional_specifications(_angle_offset_1, _angle_upper_bound_1, _angle_lower_bound_1, _speed_upper_bound, _speed_lower_bound);
    // stepper2._configure_additional_specifications(_angle_offset_2, _angle_upper_bound_2, _angle_lower_bound_2, _speed_upper_bound, _speed_lower_bound);
    // stepper3._configure_additional_specifications(_angle_offset_3, _angle_upper_bound_3, _angle_lower_bound_3, _speed_upper_bound, _speed_lower_bound);



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

    char terminator = _UART_Receive(); // Expect the '\n' at the end of the 3-float package
    if (terminator != '\n') {
        //Handle failing case
    }

    stepper1._Set_Direction();
    stepper1._Timer_re_enable();

    stepper2._Set_Direction();
    stepper2._Timer_re_enable();

    stepper3._Set_Direction();
    stepper3._Timer_re_enable();

}

ISR(TIMER4_COMPA_vect)
{
    stepper1._ISR_execute_Angle();
    stepper2._ISR_execute_Angle();
    stepper3._ISR_execute_Angle();
    ;
}

ISR(TIMER3_COMPA_vect)
{
    // stepper1._ISR_execute_Angle();
    // stepper2._ISR_execute_Angle();
    // stepper3._ISR_execute_Angle();
    ;
}

ISR(TIMER1_COMPA_vect)
{
    // stepper1._ISR_execute_Angle();
    // stepper2._ISR_execute_Angle();
    // stepper3._ISR_execute_Angle();
    ;
}

void loop() {

    _Transmit_Float(stepper1._get_Angle());
    _Transmit_Float(stepper2._get_Angle());
    _Transmit_Float(stepper3._get_Angle());
    _UART_Transmit('\n');
    delay(1000);

}
