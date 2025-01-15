#include "GCodeHandler.h"


void _G_Code_Handler::_G_Code_Queue_Update(const String& _Instruction)                          //Adding to the end of the Queue with provided Instruction
{
    String _temporary_Copy = _Instruction;
    this -> _G_Code_Queue.push(&_temporary_Copy);
    return;
}

void _G_Code_Handler::_Segment_Update()                                                         //This function aims to translate the first GCode instruction in GCode Queue to Segment
{

    if (this -> _G_Code_Queue.isEmpty())                                                        //If the Size of GCode Queue equals 0, abort the function
    {
        return;
    }

    //_element temp = this -> _G_Code_Execute(this -> _G_Code_Queue.peek());             //Initialize a temporary storage vector which stores the returned vector from executing a GCode line
    
    String tempInstruction;
    this -> _G_Code_Queue.pop(& tempInstruction);                                               //Delete the first element of GCode Queue
    
    int* temp = this -> _G_Code_Execute(tempInstruction);
    
    if (temp[0] == 0)                                                                           //If the first letter in GCode instruction is G, activate Linear Interpolation
    {                                                                                           //Otherwise, activate Linear Interpolation
        Interpolation._Circle_Interpolation(temp[2], temp[3], temp[4], temp[5], temp[1]);
    } else {
        Interpolation._Linear_Interpolation(temp[1], temp[2], temp[3]);
    }

    delete[] temp;

    return;
}

int* _G_Code_Handler::_G_Code_Execute(const String& input)
{
    int* keyvalue = new int[6]();                                                                   //Initialize a vector which encapsulates key values derived from fed-in GCode instruction


    if (input[0] == 'G')                                                                            //Check the first word: If it is "G" then assigne keyvalue[0] to 0
    {                                                                                               //Otherwise, assign it to 1

        keyvalue[0] = 0;

    } else if (input[0] == 'F') 
    {

        keyvalue[0] = 1;

    }


    if (!keyvalue[0])
    {

        keyvalue[1] = input[1];                                                                     //Store value of Rotating Direction

    }

                                                                                                    //Read the Values associate with X, Y, Z respectively
                                                                                                    //The format of customised GCode Instruction: G0 Iiii.i Jjjj.j Xxxx.x Yyyy.y (indicate rotating clockwise with the center's absolute XY-coordinates are (iii,jjj) and end position coordinates are (xxx, yyy)) 
                                                                                                    //                                            G1 Iiii.i Jjjj.j Xxxx.x Yyyy.y (indicate rotating counter-clockwise with the center's absolute XY-coordinates are (iii,jjj) and end position coordinates are (xxx, yyy))
    if (!keyvalue[0])                                                                               //                                            F  Xxxx.x Yyyy.y Zzzz.z      (indicate translating from current absolute position to new absolute position (xxx, yyy, zzz))
    {
       
        keyvalue[2] = (input[4] - '0')*100 + (input[5] - '0')*10 + (input[6] - '0');                                        //Store value of Iiii
        keyvalue[3] = (input[9] - '0')*100 + (input[10] - '0')*10 + (input[11] - '0');                                      //Store value of Jjjj
        keyvalue[4] = (input[14] - '0')*100 + (input[15] - '0')*10 + (input[16] - '0');                                     //Store value of Xxxx
        keyvalue[5] = (input[19] - '0')*100 + (input[20] - '0')*10 + (input[21] - '0');                                     //Store value of Yyyy


    } else {
        
        keyvalue[2] = (input[3] - '0')*100 + (input[4] - '0')*10 + (input[5] - '0');                                        //Store value of Xxxx
        keyvalue[3] = (input[8] - '0')*100 + (input[9] - '0')*10 + (input[10] - '0');                                       //Store value of Yyyy
        keyvalue[4] = (input[13] - '0')*100 + (input[14] - '0')*10 + (input[15] - '0');                                     //Store value of Zzzz

    }

    return keyvalue;
};

_G_Code_Handler _GCodeHandler;