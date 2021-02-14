#ifndef ALGO_H
#define ALGO_H

#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEFT_MOTOR_ENABLE 27 
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_PIN 25   //27                       // PWM pin for left motor
#define RIGHT_MOTOR_PIN 13                      // PWM pin for right motor
#define LEFT_MOTOR_DIRECTION 26 
#define RIGHT_MOTOR_DIRECTION 14

#define LEFT_ENCODERA 36                       //Only signal A of both the motor is being used for now
#define LEFT_ENCODERB 39
#define RIGHT_ENCODERA 34                      //as the direction is set buy us, but for point we might need to use both
#define RIGHT_ENCODERB 35
#define GPIO_ENCODER_PIN_SEL  ((1ULL<<LEFT_ENCODERA) | (1ULL<<RIGHT_ENCODERA))  //64 bit mask
#define ESP_INTR_FLAG_DEFAULT 0

#define ULTRA1 3
#define ULTRA2 4
#define ULTRA3 5
#define ULTRA4 6
#define ULTRA5 7
#define GPIO_ULTRA_PIN_SEL ((1ULL<<ULTRA1)|(1ULL<<ULTRA2)|(1ULL<<ULTRA3)|(1ULL<<ULTRA4)|(1ULL<<ULTRA5))

#define ENCODERresolution 1200              //The resolution of encoder, current using any edge single channel
#define wheeldist_perTick 0.000445           //in meters
#define wheelbase 0.30                   //in meters
#define sampleTime 100000     //In microsec
#define sampleTimeInSec 0.1
extern int  DEFAULT_LIN_SPEED;              // meter/sec    //Temporary constants I used. To be deleted when encoder feedback is used
extern int DEFAULT_ANG_SPEED ;            //rad/sec        //Temporary constants I used. To be deleted when encoder feedback is used

extern int Lpwm;                       //Variables to pass pwm to the LEDC set duty function
extern int Rpwm;  
extern int leftRot;
extern int leftTicks;
extern int rightRot;
extern int rightTicks;
extern double left_vel;
extern double right_vel;
extern double prev_disL;
extern double prev_disR;
extern int64_t prev_tim;

extern double accumulated_errorL;             //integral term in PID formula
extern double current_errorL;                 //current error in PID formula
extern double prev_errorL;                    //error in the previous time step
extern double accumulated_errorR;             
extern double current_errorR;                 
extern double prev_errorR;                    

extern int pid_flag;                        //flag that signifies that the PID controller's job is done
extern double Kp;
extern double Kd;
extern double Ki;


extern xQueueHandle gpio_evt_queue;

ledc_channel_config_t motorL, motorR;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output
gpio_config_t ENCODER, ULTRASONIC;

struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};

void init_gpio();
void init_pwm();

void move_forward();
void move_left();
void move_right();
void move_back();
void move_stop();

void init_pid();
int pid_velLeft(double actualvel, double desiredvel);
int pid_velRight(double actualvel, double desiredvel);
int map1(float x, float in_min, float in_max, int out_min, int out_max);

char determine(int local_flag);
esp_err_t convert_paths(int n);
esp_err_t get_path(int local_flag);
void IRAM_ATTR gpio_encoder_isr_handler(void* arg);
void sensing();
void actuation();
void sensor_initilize();

#endif