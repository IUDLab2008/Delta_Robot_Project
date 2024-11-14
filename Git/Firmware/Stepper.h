#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>
// #define WINDOW_SIZE 10 // Define the size of the moving average window


class _Stepper_Motor {

    private:
        bool _rotate_direction;                                 //Storing rotate direction
                                                                //0     -       Quay lên trời          (khi lật ngửa)
                                                                //1     -       Quay xuống             (khi lật ngửa)
                                                                
        int _rpm;                                               //Storing desired RPM (Revolution per Minute) for both Constant Speed mode (Angle mode) and Varying Speed mode (Angular Speed mode)

        int _step;                                              //Storing total number of Step per Revolution
        byte _mode;                                             //Storing Step mode of Stepper Motor 
                                                                //1     -       Full step
                                                                //2     -       Half step
                                                                //4     -       Quarter step
                                                                //8     -       Eighth step
                                                                //16    -       Sixteenth step

        byte MS[3];                                             //Storing MS[1:3] PIN
        byte _reset_pin;                                        //Storing Reset Pin
        byte _sleep_pin;                                        //Storing Sleep Pin
        byte _enable_pin;                                       //Storing Enable Pin
        byte _dir_pin;                                          //Storing Direction Pin

        byte _step_pin;                                         //Storing Step Pin, the Pin must be assigned from 4 available Pins (the selection must not be overlapped)
                                                                //OC1[A:B:C]      -       D11/12/13 or PB5/PB6/PB7                 (all can be used)
                                                                //OC3[A:B:C]      -       D5/2/3    or PE3/PE4/PE5                 (all can be used)
                                                                //OC4[A:B:C]      -       D6/7/8    or PH3/PH4/PH5                 (can not use D6)
                                                                //OC5[A:B:C]      -       D44/45/46 or PL5/PL4/PL3                 (can not use D44-46)                  (Least favorable)
                                                                

        float _res;                                             //Storing Resolution of the Stepper with assigned Step mode (as stored in _mode)

        bool _timer_state;                                      //Storing Timer status
                                                                //0 - Timer off
                                                                //1 - Timer on



        float _speed_upper_bound;                               //Storing Angular Speed Upper Bound of Stepper
        float _speed_lower_bound;                               //Storing Angular Speed Lower Bound of Stepper

        float _angle_offset;                                    //Storing Angular offset


        // float angleBuffer[WINDOW_SIZE]; // Circular buffer to store the angles
        // int bufferIndex = 0; // Current index in the circular buffer
        // float sumAngles = 0; // Sum of the angles in the buffer
        // int count = 0; // Number of valid entries in the buffer

    public:
        float _desired_value = 0;                               //Storing Desired Angle value                                              (Angle mode)
        byte _encoder_pin;                                      //Storing Encoder Pin

        int _angle_upper_bound;                                 //Storing Angular Upper Bound of Stepper
        int _angle_lower_bound;                                 //Storing Angular Lower Bound of Stepper

        _Stepper_Motor(byte MS[], byte mode, byte reset_pin, byte sleep_pin, byte enable_pin, byte dir_pin, byte encoder_pin, int step, byte step_pin, bool rotate_dir);                //Configure Stepper Motor  
        void _configure_additional_specifications(float angle_offset, int angle_upper_bound, int angle_lower_bound, float speed_upper_bound, float speed_lower_bound);                         //Configure additional specifications of Stepper Motor

        void _Timer_enable(void);                               //Enable the Timer                                          
                                                                //Execute once in Initializing phase                                                                 
        void _Timer_disable(void);                              //Disable the Timer
        void _Timer_re_enable(void);                            //Re-enable the Timer
                                                    
        
        void _Set_rpm(int _rpm);                                //Set desired RPM (Revolution per Minute) for Constant Speed mode and Varying Speed mode       (Angle mode & Speed mode)
                                                                //Invoke Once in Initialize phase                

        void _Set_Angle(float _desired_value);                   //Set desired Angle for Constant Speed mode                                                   (Angle mode)

        float _get_Angle(void);                                 //Derive the Angle value from the Encoder using Moving Average method

        void _execute_Homing(void);                             //Moving to Home (lower-bound of the Angle of Motor), execute Once in the Initializing phase
                                                                //Moving in predescibe Angular Speed, neglect of Operating mode

        void _Set_Direction(void);

        void _ISR_execute_Angle(void);                          //Moving to assigned desired Angle                                                             (Angle mode)
                                                                //Execute in ISR (Interrupt Service Routine)
                                                                //Could include other functions according to demands 


};

extern _Stepper_Motor Stepper;

#endif  