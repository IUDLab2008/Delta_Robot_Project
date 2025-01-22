#ifndef DATA_H
#define DATA_H

#include "Arduino.h"

struct element
{
    float element1;
    float element2;
    float element3;

    float& get(int index)
    {
        switch (index)
        {
        case 0: return element1;
        case 1: return element2;
        case 2: return element3;
        }
    }
};

struct QueueSet
{
    queue<float> timeStep;

    queue<float> RPMQueue1;
    queue<float> RPMQueue2;
    queue<float> RPMQueue3;
};

class Data {
    private:
    public:
    /*
        Args:
            currentX:   Store the current X coordinate of the End-Effector
            currentY:   Store the current Y coordinate of the End-Effector
            currentZ:   Store the current Z coordinate of the End-Effector
            timeStep:   Store the current time needed to fully execute the first element in QueueSet
    */
        float currentX;
        float currentY;
        float currentZ;

        float timeStep;
        void setUpTimeWatcher();
        void setTimeWatcherValue(float timeStep);
        void timeWatcher(QueueSet& queueT);
};

#endif