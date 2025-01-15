
#ifndef UART_H
#define UART_H

#include <Arduino.h>
#include <avr/interrupt.h>

class _UART {

    private:

        int _baudRate;                                                  //Storing Baud Rate value
    
    public:

        _UART(unsigned int _baud_settings);

        void _UART_Flush(void);                                         //Flush UART transmit and receive buffer

        void _UART_Transmit(unsigned char _data);                       //Transmit given 1 byte of data
        unsigned char _UART_Receive (void);                             //Receiving 1 byte of data

        void _Transmit_Float(float _data);                              //Transmit given float32 data

        void _UART_ISR_handle (float& _angle);                          //UART interrupt service routine handler

};

#endif  
