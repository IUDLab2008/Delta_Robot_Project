#include "Stepper.h"
#include "SerialCommunication.h"
#include "GCodeReceiver.h"
#include "Interpolation.h"
#include "Data.h"

Stepper stepperMotor1(0, true);
Stepper stepperMotor2(1, true);
Stepper stepperMotor3(2, true);

UART uartInstance(MYUBRR);

GCodeReceiver gCodeReceiverInstance;
queue<String> instructionSet;
Data dataInstance;

Interpolation interpolationInstance;
Kinematic kinematicInstance;

QueueSet queueRes;

ISR(USART0_TX_vect)
{
  uartInstance.ISRInstructionHandle(instructionSet, gCodeReceiverInstance);
  gCodeReceiverInstance.parseInstruction(instructionSet);

  queueRes = interpolationInstance.convert2Angles(gCodeReceiverInstance, kinematicInstance);
}

ISR(TIMER1_COMPA_vect)
{
  if (stepperMotor3.getIsHoming())
  {  
    stepperMotor3.HomingISRAngleExecute();
  } else {
    stepperMotor3.ISRAngleExecute();
  }
}

ISR(TIMER3_COMPA_vect)
{
  if (stepperMotor2.getIsHoming())
  {  
    stepperMotor2.HomingISRAngleExecute();
  } else {
    stepperMotor2.ISRAngleExecute();
  }
}

ISR(TIMER4_COMPA_vect)
{
  if (stepperMotor1.getIsHoming())
  {  
    stepperMotor1.HomingISRAngleExecute();
  } else {
    stepperMotor1.ISRAngleExecute();
  }
}

ISR(TIMER5_COMPA_vect)
{
  sei();
  
  dataInstance.timeWatcher(queueRes);
  stepperMotor1.setRPM(queueRes.RPMQueue1.front());
  stepperMotor2.setRPM(queueRes.RPMQueue2.front());
  stepperMotor3.setRPM(queueRes.RPMQueue3.front());
}

void setup() {
  uartInstance.UARTFlush();

  dataInstance.setUpTimeWatcher();

  stepperMotor1.timerEnable();
  stepperMotor1.setRPM(HOMING_RPM);

  stepperMotor2.timerEnable();
  stepperMotor2.setRPM(HOMING_RPM);

  stepperMotor3.timerEnable();
  stepperMotor3.setRPM(HOMING_RPM);

  while (stepperMotor1.getIsHoming() || stepperMotor2.getIsHoming() || stepperMotor3.getIsHoming() || !uartInstance.getEndOfTransmission())
  {
    ;
  }

  dataInstance.setTimeWatcherValue(queueRes.timeStep.front());
  stepperMotor1.setRPM(queueRes.RPMQueue1.front());
  stepperMotor2.setRPM(queueRes.RPMQueue2.front());
  stepperMotor3.setRPM(queueRes.RPMQueue3.front());

  sei();
}

void loop() {
  uartInstance.transmitFloat(stepperMotor1.getAngle());
  uartInstance.transmitFloat(stepperMotor2.getAngle());
  uartInstance.transmitFloat(stepperMotor3.getAngle());
  delay(500);
}
