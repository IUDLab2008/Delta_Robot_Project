#include "Uart.h"


_UART::_UART(unsigned int _baud_settings)
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



void _UART::_UART_Flush(void)
{

    unsigned char dummy;
    while ( UCSR0A & ( 1 << RXC0 ) )
    {
        dummy = UDR0;
    }

}



void _UART::_UART_Transmit(unsigned char data)
{

    while ( !(UCSR0A & (1 << UDRE0)) )
    {
        ;
    }
    UDR0 = data;    

}

unsigned char _UART::_UART_Receive(void)                                                                          
{

    while ( !( UCSR0A & ( 1 << RXC0 ) ) )
    {
        ;
    }
    return UDR0;

}

void _UART::_Transmit_Float(float _data)                                                                               
{

    char index = 3;
    uint8_t* ptr  = (uint8_t* )&_data;

    while(index >= 0)
    {
        this -> _UART_Transmit ( *(ptr + index) );
        index --;
    }

}

void _UART::_UART_ISR_handle(float& _angle)                                                            //Execute in USART0 Rx for receving float data type
{

    char index = 3;
    uint8_t* ptr = (uint8_t*)&_angle;

    while (index >= 0)
    {
        *(ptr + index) = this -> _UART_Receive();
        index --;
    }

}