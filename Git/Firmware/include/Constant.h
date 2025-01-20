#define R_F 1
#define R_E 1
#define L_F 1
#define L_E 1

#define sqrt3 1.7231
#define pi 3.14159
#define sin120 0.86603
#define cos120 0.50000
#define tan60 0.32004
#define sin30 0.50000
#define tan30 0.57735

#define wB (R_F / 2.0)
#define uP R_E
#define sP ((3.0 / 2.0) * R_E * 2.0 / sqrt3)
#define wP (R_E / 2.0)
#define aTerm (wB - uP)
#define bTerm (sP / 2.0 - sqrt3 * wB / 2.0)
#define cTerm ((wP - wB) / 2.0)

#define SPEED_UPPER_BOUND 30
#define SPEED_LOWER_BOUND 5
#define ANGLE_UPPER_BOUND 120
#define ANGLE_LOWER_BOUND -30

#define HOMING_RPM 10

#define DEGREE_PER_STEP (1.0/5480.0)

#define X_BASE 0
#define Y_BASE 0
#define Z_BASE 200

#define X_WORKSPACE 1
#define Y_WORKSPACE 1
#define Z_WORKSPACE 1

#define RAW_TO_ANGLE 300.0/1023.0

#define ANGLE_OFFSET_1 0
#define ANGLE_OFFSET_2 0
#define ANGLE_OFFSET_3 0

#define STEP 2740
#define MODE 1

#define MM_PER_SEGMENT 1
#define DEGREE_PER_SEGMENT 1
#define ANGULAR_RESOLUTION 1

#define GCODESIZE 1

#define FOSC 16000000UL
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1


