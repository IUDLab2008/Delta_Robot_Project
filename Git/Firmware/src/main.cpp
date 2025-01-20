#include "Stepper.h"
#include "SerialCommunication.h"
#include "GCodeReceiver.h"
#include "Interpolation.h"

Stepper stepperMotor1(0, true);
Stepper stepperMotor2(1, true);
Stepper stepperMotor3(2, true);

UART uartInstance(MYUBRR);

GCodeReceiver gCodeReceiverInstance;
queue<String> instructionSet;

Interpolation interpolationInstance;
Kinematic kinematicInstance;

QueueSet queueRes;

ISR(USART0_TX_vect)
{
  uartInstance.ISRInstructionHandle(instructionSet, gCodeReceiverInstance);
  gCodeReceiverInstance.parseInstruction(instructionSet);

  queueRes = interpolationInstance.convert2Angles(gCodeReceiverInstance, kinematicInstance);

  stepperMotor1.instructionExecution(queueRes.numInterruptQueue1, queueRes.RPMQueue1);
  stepperMotor2.instructionExecution(queueRes.numInterruptQueue2, queueRes.RPMQueue2);
  stepperMotor3.instructionExecution(queueRes.numInterruptQueue3, queueRes.RPMQueue3);
}

ISR(TIMER1_COMPA_vect)
{
  stepperMotor3.HomingISRAngleExecute();
  stepperMotor3.ISRAngleExecute();
}

ISR(TIMER3_COMPA_vect)
{
  stepperMotor2.HomingISRAngleExecute();
  stepperMotor2.ISRAngleExecute();
}

ISR(TIMER4_COMPA_vect)
{
  stepperMotor1.HomingISRAngleExecute();
  stepperMotor1.ISRAngleExecute();
}


void setup() {
  uartInstance.UARTFlush();

  stepperMotor1.timerEnable();
  stepperMotor1.setRPM(HOMING_RPM);

  stepperMotor2.timerEnable();
  stepperMotor2.setRPM(HOMING_RPM);

  stepperMotor3.timerEnable();
  stepperMotor3.setRPM(HOMING_RPM);
}

void loop() {
  uartInstance.transmitFloat(stepperMotor1.getAngle());
  uartInstance.transmitFloat(stepperMotor2.getAngle());
  uartInstance.transmitFloat(stepperMotor3.getAngle());
  delay(500);
}
