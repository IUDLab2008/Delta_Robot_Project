#include "Interpolation.h"

float Interpolation::distanceBet2Point(float _x_0, float _y_0, float _z_0, float _x_1, float _y_1, float _z_1) {
    return sqrt(pow(_x_0 - _x_1, 2) + 
                pow(_y_0 - _y_1, 2) + 
                pow(_z_0 - _z_1, 2));
}

queue<element> Interpolation::linearInterpolation(Kinematic kinematicInstance, 
                                                float x0, float y0, float z0, 
                                                float x1, float y1, float z1) 
{   
    queue<element> linearInterpolationQueue;
    
    if (!kinematicInstance.checkPointValidation(x1, y1, z1)) 
    {
        linearInterpolationQueue.push({NAN, NAN, NAN});
        return linearInterpolationQueue;
    }

    float distance = this->distanceBet2Point(x0, y0, z0, x1, y1, z1);
    if (distance < 0.001f) 
    {
        linearInterpolationQueue.push({x1, y1, z1});
        return linearInterpolationQueue;
    }

    int segmentNum = ceil(distance / MM_PER_SEGMENT);

    for (int i = 0; i <= segmentNum; i++) 
    {
        float buffer = static_cast<float>(i) / segmentNum;
        float xBuffer = (1 - buffer) * x0 + buffer * x1;
        float yBuffer = (1 - buffer) * y0 + buffer * y1;
        float zBuffer = (1 - buffer) * z0 + buffer * z1;
        linearInterpolationQueue.push({xBuffer, yBuffer, zBuffer});
    }
    return linearInterpolationQueue;
}

queue<element> Interpolation::convert2AngularVelocities(GCodeReceiver& GCodeReceiverInstance, 
                                                      Kinematic kinematicInstance) 
{
    queue<element> angularVelocitiesQueue;
    
    if (GCodeReceiverInstance.XValue.empty()) {
        return angularVelocitiesQueue;  
    }

    float currentX = X_BASE;
    float currentY = Y_BASE;
    float currentZ = Z_BASE;
    
    while (!GCodeReceiverInstance.XValue.empty()) {
        
        float targetX = GCodeReceiverInstance.XValue.front();
        float targetY = GCodeReceiverInstance.YValue.front();
        float targetZ = GCodeReceiverInstance.ZValue.front();
        float absoluteVelocity = GCodeReceiverInstance.FValue.front();
        
        GCodeReceiverInstance.XValue.pop();
        GCodeReceiverInstance.YValue.pop();
        GCodeReceiverInstance.ZValue.pop();
        GCodeReceiverInstance.FValue.pop();

        queue<element> pathSegments = this->linearInterpolation(
            kinematicInstance, 
            currentX, currentY, currentZ,
            targetX, targetY, targetZ
        );

        if (!pathSegments.empty() && isnan(pathSegments.front().element1)) {
            angularVelocitiesQueue.push({NAN, NAN, NAN});
            return angularVelocitiesQueue;
        }

        float xDot, yDot, zDot;
        kinematicInstance.calculateSubVelocities(
            absoluteVelocity,
            currentX, currentY, currentZ,
            targetX, targetY, targetZ,
            xDot, yDot, zDot
        );

        while (!pathSegments.empty()) 
        {
            element segment = pathSegments.front();
            pathSegments.pop();
            
            element angularVelocity = kinematicInstance.calculateAngularVelocities(
                segment.element1,
                segment.element2,
                segment.element3,
                xDot, yDot, zDot
            );
            
            angularVelocitiesQueue.push(angularVelocity);
        }

        currentX = targetX;
        currentY = targetY;
        currentZ = targetZ;
    }

    return angularVelocitiesQueue;
}

QueueSet Interpolation::convert2Angles(GCodeReceiver& GCodeReceiverInstance,
                                                     Kinematic kinematicInstance)
{
    QueueSet resQueue;

    if (GCodeReceiverInstance.XValue.empty()) {
        return resQueue;  
    }

    float currentX = X_BASE;
    float currentY = Y_BASE;
    float currentZ = Z_BASE;

    while(!GCodeReceiverInstance.XValue.empty())
    {
        float targetX = GCodeReceiverInstance.XValue.front();
        float targetY = GCodeReceiverInstance.YValue.front();
        float targetZ = GCodeReceiverInstance.ZValue.front();
        float timeStep = GCodeReceiverInstance.timeStep.front();
        
        GCodeReceiverInstance.XValue.pop();
        GCodeReceiverInstance.YValue.pop();
        GCodeReceiverInstance.ZValue.pop();
        GCodeReceiverInstance.timeStep.pop();

        queue<element> pathSegments = this->linearInterpolation(
            kinematicInstance, 
            currentX, currentY, currentZ,
            targetX, targetY, targetZ
        );

        float currentAngle1 = 0;
        float currentAngle2 = 0;
        float currentAngle3 = 0;

        while(!pathSegments.empty())
        {
            float nextAngle1, nextAngle2, nextAngle3;
            element segment = pathSegments.front();
            pathSegments.pop();

            kinematicInstance.inverseKinematic(
                nextAngle1,
                nextAngle2,
                nextAngle3,
                segment.element1,
                segment.element2,
                segment.element3
                );

            resQueue.numInterruptQueue1.push(ceil((nextAngle1 - currentAngle1) / DEGREE_PER_STEP) * 2);
            resQueue.numInterruptQueue2.push(ceil((nextAngle2 - currentAngle2) / DEGREE_PER_STEP) * 2);
            resQueue.numInterruptQueue3.push(ceil((nextAngle3 - currentAngle3) / DEGREE_PER_STEP) * 2);
            
            resQueue.RPMQueue1.push(resQueue.numInterruptQueue1.back() / timeStep);
            resQueue.RPMQueue2.push(resQueue.numInterruptQueue2.back() / timeStep);
            resQueue.RPMQueue3.push(resQueue.numInterruptQueue3.back() / timeStep);

            currentAngle1 = nextAngle1;
            currentAngle2 = nextAngle2;
            currentAngle3 = nextAngle3;
        }
    }
    return resQueue;
}