#ifndef ALGO_H
#define ALGO_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

//configure the motor pins and encoder pins  change the pin numbers to associated pin numbers 
#define LEFT_ENCODERA 39                       //Only signal A of both the motor is being used for now
#define RIGHT_ENCODERA 36                      
#define GPIO_ENCODER_PIN_SEL  ((1ULL<<LEFT_ENCODERA) | (1ULL<<RIGHT_ENCODERA))  //64 bit mask
#define ESP_INTR_FLAG_DEFAULT 0


#define LEFT_MOTOR_ENABLE 33 
#define RIGHT_MOTOR_ENABLE 12
#define LEFT_MOTOR_PIN 27                         // PWM pin for left motor
#define RIGHT_MOTOR_PIN 13                      // PWM pin for right motor
#define LEFT_MOTOR_DIRECTION 32 
#define RIGHT_MOTOR_DIRECTION 14 


//configuring the ultrasonic pins if needed change the pin numbers where the sensors are connected
#define ULTRA1 4
#define ULTRA2 2
#define ULTRA3 15
#define ULTRA4 25
#define ULTRA5 26
#define GPIO_ULTRASONIC_PIN_SEL ((1ULL<<ULTRA1) | (1ULL<<ULTRA2) | (1ULL<<ULTRA3) | (1ULL<<ULTRA4) | (1ULL<<ULTRA5))

#define TXD_PIN (GPIO_NUM_17) // ULTRASONIC PINS for configuration 
#define RXD_PIN (GPIO_NUM_16)

//#define ENCODERresolution 1200              //The resolution of encoder, current using any edge single channel

//#define wheeldist_perTick 0.000445          //in meters
//#define wheelbase 0.30                     //in meters

#define ENCODERresolution 55              //The resolution of encoder, current using 'any edge' single channel
#define wheeldist_perTick 0.00343                   //in meters
#define wheelbase 0.1475                      //in meters

#define DEFAULT_LIN_SPEED 0.7             //meter/sec    
#define DEFAULT_ANG_SPEED 0.5               //rad/sec   
#define MIN_LINEAR_SPEED 0.005
#define MIN_ANGULAR_SPEED 0.05
     
#define sampleTime 100000                   //in microsec
#define sampleTimeInSec 0.1

#define ADC_SAMPLING_FREQ 1000000
#define ADC_AVERAGING_FREQ 10

extern int Lpwm;                       //Variables to pass pwm to the LEDC set duty function
extern int Rpwm;  

// the following parameters are used for encoder feedback

extern int leftRot;                    //Variable to take care to the encoder feedback
extern int leftTicks;
extern int rightRot;
extern int rightTicks;
extern int pwm_min;

extern double left_vel;                // left motor instantaneous velocity 
extern double right_vel;                // right motor instantaneous velocity


extern int point_index;                 // used to keep track of goal point position

 // used to set the lin and angular velocities of the robot
extern double lin_speed;               
extern double ang_speed;

extern int64_t prev_tim;
extern double current_point[2];            //current co-ordinates of the bot*/

// pid loop parameters
extern double accumulated_errorL;             //integral term in PID formula
extern double current_errorL;                 //current error in PID formula
extern double prev_errorL;                    //error in the previous time step
extern double accumulated_errorR;             
extern double current_errorR;                 
extern double prev_errorR;

extern double prev_time;               //stores the previous time step, gets updated to current time after sysCall_sensing
extern int batteryPercent;              // used to store the battery percentage which is displayed on the web page
extern int doneFlag;                   // used for checking if the robot has reached desired distance and orientaion with the goal point  

/* uncomment only if you r sure that the both motor responce is ideal and want to use one single pid loop do necessary changes to the algo.c code as well
extern double Kp;                //Common gains for now
extern double Kd;
extern double Ki;
*/

// PID gain parameters for left and right motor
extern double Kpl;                //Common gains for now
extern double Kdl;
extern double Kil;

extern double Kpr;                //Common gains for now
extern double Kdr;
extern double Kir;


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
void sensor_initilize();

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

void updateParams(double, double);
void actuationAuto();
void normal_motion();
void rotate();
void forward();
void update_stopPoint();
void recalculate();
void sensing();
void forwardSlow(int);
void rotateSlow(int);
esp_err_t point_update();
void battery_low();

#endif