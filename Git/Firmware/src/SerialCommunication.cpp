#include "SerialCommunication.h"

UART::UART(unsigned int baudRate)
{
    UBRR0H = (unsigned char)(baudRate >> 8);
    UBRR0L = (unsigned char)(baudRate);
    
    UCSR0A  = 0;                                                                                    
    UCSR0A |=  (1 << UDRE0);                                                                        
    UCSR0A &= ~(1 << U2X0);

    UCSR0B |=  ( (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) );                                     

    UCSR0C &= ~( (1 << UMSEL01) | (1 << UMSEL00) | (1 << UPM00) | (1 << USBS0) );                   
    UCSR0C |=  ( (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00) );                                   

}

void UART::UARTFlush()
{
    unsigned char dummy;
    while ( UCSR0A & ( 1 << RXC0 ) )
    {
        dummy = UDR0;
    }
}

void UART::UARTTransmit(unsigned char data)
{
    while ( !(UCSR0A & (1 << UDRE0)) )
    {
        ;
    }
    UDR0 = data; 
}

unsigned char UART::UARTReceive(void)                                                                          
{
    while ( !( UCSR0A & ( 1 << RXC0 ) ) )
    {
        ;
    }
    return UDR0;
}

void UART::transmitFloat(float data)
{
    char index = 3;
    uint8_t* ptr  = (uint8_t* )&data;

    while(index >= 0)
    {
        this -> UARTTransmit ( *(ptr + index) );
        index --;
    }
}


void UART::ISRAngleHandle(float& angle)
{
    char index = 3;
    uint8_t* ptr = (uint8_t*)&angle;

    while (index >= 0)
    {
        *(ptr + index) = this -> UARTReceive();
        index --;
    }
}

void UART::ISRInstructionHandle(queue<String>& instructionSet, GCodeReceiver GCodeReceiverInstance)
{
    static String temporaryString = "";
    while (UCSR0A & (1 << RXC0))
    {
        char receivedChar = this -> UARTReceive();
        if (receivedChar != '\n')
        {
            temporaryString += receivedChar;
        } else {
            if (GCodeReceiverInstance.validateInstruction(temporaryString))
                {
                    instructionSet.push(temporaryString);
                    temporaryString = "";
                }
        }
    }
}

