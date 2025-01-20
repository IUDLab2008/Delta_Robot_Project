#ifndef DATA_H
#define DATA_H

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
    queue<int> numInterruptQueue1;
    queue<int> numInterruptQueue2;
    queue<int> numInterruptQueue3;

    queue<float> RPMQueue1;
    queue<float> RPMQueue2;
    queue<float> RPMQueue3;
};

class Data {
    private:
    public:
        float currentX;
        float currentY;
        float currentZ;

        float timeStep;
};

#endif