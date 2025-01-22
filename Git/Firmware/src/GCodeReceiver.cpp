#include "GCodeReceiver.h"

bool GCodeReceiver::validateInstruction(String instruction) const
{
    if (!instruction.length())
    {
        return false;
    }

    if (!instruction.startsWith("G1") && !instruction.startsWith("G0"))
    {
        return false;
    }

    return true;
}

void GCodeReceiver::parseInstruction(queue<String>& instructionQueue)
{
    static String instruction = "";

    while (!instructionQueue.empty())
    {
        instruction = instructionQueue.front();
        instructionQueue.pop();
        this -> key.push(instruction[1] - '0');

        bool FIdentifier = false;
        bool XIdentifier = false;
        bool YIdentifier = false;
        bool ZIdentifier = false;
        
        for (int i = 2; i < instruction.length(); i++)
        {
            if (isalpha(instruction[i]))
            {   
                String value = "";
                int j = i + 1;
                while ((j < instruction.length()) && (isdigit(instruction[j]) || instruction[j] == '.' || instruction[j] == '-'))
                {
                    value += instruction[j];
                    j++;
                }

                char prefixIndentifier = instruction[i];
                i = j - 1;

                switch (prefixIndentifier)
                {
                case 'F':
                    FIdentifier = true;
                    this -> FValue.push(value.toFloat());
                    break;
                
                case 'X':
                    XIdentifier = true;
                    this -> XValue.push(value.toFloat());
                    break;

                case 'Y':
                    YIdentifier = true;
                    this -> YValue.push(value.toFloat());
                    break;

                case 'Z':
                    ZIdentifier = true;
                    this -> ZValue.push(value.toFloat());
                    break;
                }
            }
        }
        if (!FIdentifier && !this -> key.back())
        {
            this -> FValue.push((this -> FValue.back() > 0) ? -SPEED_UPPER_BOUND : SPEED_UPPER_BOUND);
        } else if (!FIdentifier && this -> key.back())
        {
            this -> FValue.push(this -> FValue.back());
        }

        if (!XIdentifier) {
            if (!this -> XValue.empty()) {
                this -> XValue.push(this -> XValue.back());
            } else {
                this -> XValue.push(0.0); 
            }
        }

        if (!YIdentifier) {
            if (!this -> YValue.empty()) {
                this -> YValue.push(this -> YValue.back());
            } else {
                this -> YValue.push(0.0); 
            }
        }

        if (!ZIdentifier) {
            if (!this -> ZValue.empty()) {
                this -> ZValue.push(this -> ZValue.back());
            } else {
                this -> ZValue.push(0.0); 
            }
        }
        this -> timeStep.push(float(sqrt(pow(this -> XValue.back(), 2) + pow(this -> YValue.back(), 2) + pow(this -> ZValue.back(), 2)) / this -> FValue.back()));
    }
}