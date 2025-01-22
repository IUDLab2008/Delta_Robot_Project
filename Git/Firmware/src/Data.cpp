#include "Data.h"

void Data::setUpTimeWatcher()
{
    /*
        Summary:
            This function set up Timer 5 at CTC mode. The Timer intention is to globally interrupt the system everytime the 
            counter's value is matched with the timeStep to remove the first element in sub-queues.
    */
    TCNT5 = 0;
    TCCR5A = 0;
    TCCR5B = 0;
    TIMSK5 = 0;
    TCCR5A |= (1 < WGM52);
    TCCR5B |= (1 << CS51);

    TIMSK5 |= (1 << OCIE5A);
}

void Data::setTimeWatcherValue(float timeStep)
{
    /*
        Summary:
            This function set up Timer 5 match value according to given timeStep.
        Args:
            timeStep:   Time needed to execute the current fetched instruction.
    */
    OCR5A = (F_CPU / 1000000) * timeStep / 8 - 1;
}

void Data::timeWatcher(QueueSet& queueT)
{   
    /*
        Summary:
            This function handle the Timer5 ISR. Once interrupting, it remove the first element in each sub-queues
            and assign new timeStep value for the next Interrupt cycle.
        Args:
            queueT:    Fetched GCode instruction queue.
    */
    queueT.RPMQueue1.pop();
    queueT.RPMQueue2.pop();
    queueT.RPMQueue3.pop();
    queueT.timeStep.pop();

    this -> setTimeWatcherValue(queueT.timeStep.front());
}