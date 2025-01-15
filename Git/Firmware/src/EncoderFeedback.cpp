#include "EncoderFeedback.h"

void _EncoderFeedback :: _readEncoder1(_Stepper_Motor& _stepperInstance1)
{
    tca9548.selectChannel(0);
    _stepperInstance1._current_angle = encoder1.readAngle() / RAW_TO_ANGLE;
}

void _EncoderFeedback :: _readEncoder2(_Stepper_Motor& _stepperInstance2)
{
    tca9548.selectChannel(1);
    _stepperInstance2._current_angle = encoder2.readAngle() / RAW_TO_ANGLE;
}

void _EncoderFeedback :: _readEncoder3(_Stepper_Motor& _stepperInstance3)
{
    tca9548.selectChannel(2);
    _stepperInstance3._current_angle = encoder3.readAngle() / RAW_TO_ANGLE;
}