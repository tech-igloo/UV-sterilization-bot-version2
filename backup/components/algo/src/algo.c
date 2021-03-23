#include "server.h"
#include "algo.h"

int Lpwm = 0; 
int Rpwm = 0;     //Always remember you can't define the variable in .h file     

int leftRot = 0;
int leftTicks = 0;
int rightRot = 0;
int rightTicks = 0;
double left_vel = 0;
double right_vel = 0;
double prev_disL = 0;
double prev_disR = 0;

double lin_speed = DEFAULT_LIN_SPEED;
double ang_speed = DEFAULT_ANG_SPEED;
int64_t prev_tim = 0;

int point_index=0;

double current_errorL = 0;
double accumulated_errorL = 0;
double prev_errorL = 0;
double current_errorR = 0;
double accumulated_errorR = 0;
double prev_errorR = 0;

double Kp = 1;                   //common gains for both the PID blocks for now
double Kd = 1/sampleTimeInSec;
double Ki = 1*sampleTimeInSec;

static double angle_required = 0;                //angle that the bot needs to rotate to align itself with its destination point
static double dist_required = 0;                 //distance from stop point that the bot needs to travel to reach the destination point
static int rotating_flag = 1;                   //flag that signifies whether the bot should rotate or move along a straight line

static double stop_point[2] = {0};                 //the position of the bot from which distance travlled is being calculated
static double prev_point[2] = {0};               //the next point that the bot needs to travel to
static double current_point[2] = {0};            //current co-ordinates of the bot*/

static double dist_traversed = 0;                //distance travlled by bot along straight line(+ve for forward and -ve for negative)
static double angle_rotated = 0;                 //angle rotated by bot(+ve for clockwise and -ve for anticlockwise)
static int doneFlag = 0;
double timediff;
static double current_time=0; // needed wehn for time based approach
static double prev_time = 0;                     //stores the previous time step, gets updated to current time after sysCall_sensing
static double time_flag = 0;

static int detect_flag = 0;
//static int run = 0;
static int obstacle_flag[5] = {0};              //To record ultrasonic value

xQueueHandle gpio_evt_queue = NULL;
char enpwmcmd[7]={0x44,0x00,0x55,0x01,0x99,0x02,0xaa}; // sensor commands

/*To initialize the motor direction, encoder, and sensor I/O pins*/ 
void init_gpio()
{
    ENCODER.intr_type = GPIO_INTR_ANYEDGE;      //Change according to the resolution, anyedge for quadrature when using both the pins
    ENCODER.mode = GPIO_MODE_INPUT;
    ENCODER.pull_down_en = 0;
    ENCODER.pull_up_en = 0;
    ENCODER.pin_bit_mask = GPIO_ENCODER_PIN_SEL; /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_config(&ENCODER);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(LEFT_ENCODERA, gpio_encoder_isr_handler, (void*) LEFT_ENCODERA);  //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RIGHT_ENCODERA, gpio_encoder_isr_handler, (void*) RIGHT_ENCODERA);
    
    gpio_evt_queue = xQueueCreate(20, sizeof(uint32_t));    //gpio interrupt queue

    gpio_intr_disable(LEFT_ENCODERA);  //Enabled when recording or in auto mode  
    gpio_intr_disable(RIGHT_ENCODERA);

    gpio_set_direction(LEFT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);

    ULTRASONIC.intr_type = 0;      //Change according to the resolution, anyedge for quadrature when using both the pins
    ULTRASONIC.mode = GPIO_MODE_INPUT;
    ULTRASONIC.pull_down_en = 0;
    ULTRASONIC.pull_up_en = 0;
    ULTRASONIC.pin_bit_mask = GPIO_ULTRASONIC_PIN_SEL; /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    gpio_config(&ULTRASONIC);
}
//UART sensor initlisation by installing the drivers and deleting them after that to save the resources
void sensor_initilize()
{
     const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
        uart_param_config(UART_NUM_2, &uart_config);

    // We won't use a buffer for sending data.
   uart_set_pin(UART_NUM_2,TXD_PIN, RXD_PIN,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);// Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: pin no change, CTS: pin no change )
    uart_driver_install(UART_NUM_2,  2*1024, 0, 0, NULL, 0);
    for(int i=0;i<8;i++){
        const char *data =&(enpwmcmd[i]);
        uart_write_bytes(UART_NUM_2,data,strlen(data));
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    uart_driver_delete(UART_NUM_2);

}

/*Used for setting up PWM Channel (Ref: Official Github Repo)*/
void init_pwm()
{
    ledc_timer_config_t led_timer = {
            .duty_resolution = LEDC_TIMER_12_BIT,  // resolution of PWM duty, means PWM value can go from 0 to (2^12-1 = 4095)
            .freq_hz = 10000,                      // frequency of PWM signal
            .speed_mode = LEDC_HIGH_SPEED_MODE,    // High speed mode for immiediate effect
            .timer_num = LEDC_TIMER_0,             // timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
    ledc_timer_config(&led_timer);
    
    motorL.channel = LEDC_CHANNEL_0;
    motorL.duty = 0;						//the duty value of the PWM signal
    motorL.gpio_num = LEFT_MOTOR_PIN;			//the GPIO pin connected to Left motor controller PWM port
    motorL.speed_mode = LEDC_HIGH_SPEED_MODE;
    motorL.hpoint = 0;                      //for left aligned PWM
    motorL.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&motorL);

    motorR.channel = LEDC_CHANNEL_1;
    motorR.duty = 0;						//the duty value of the PWM signal
    motorR.gpio_num = RIGHT_MOTOR_PIN;			//the GPIO pin connected to Right motor controller PWM port
    motorR.speed_mode = LEDC_HIGH_SPEED_MODE;
    motorR.hpoint = 0;
    motorR.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&motorR);
}

/*The following are the actuation functions for movement of the bot with velocity feedback control*/
void move_forward()
{   //ESP_LOGI(TAG,"FORWARD :CURRENT(x,y):%f %f path point(x,y): %f %f ",current_point[0],current_point[1],stop_point[0],stop_point[1]);
    if((esp_timer_get_time() - prev_tim) >= sampleTime) //Running the PID at a specific frequency
    {
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // 9Current_dis-prev_dis)/time VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disL = left_vel*0.1 + prev_disL;
        prev_disR = right_vel*0.1 + prev_disR;

        Lpwm = pid_velLeft(left_vel, lin_speed);     //values after being mapped to PWM range
        Rpwm = pid_velRight(right_vel, lin_speed);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION, 1);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);   //Update the PWM value to be used by this lED Channel.
    ledc_update_duty(motorL.speed_mode, motorL.channel);      //Use the Updated PWM values
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}                                                                       

void move_left()
{    //ESP_LOGI(TAG,"FORWARD :CURRENT(x,y):%f %f path point(x,y): %f %f ",current_point[0],current_point[1],stop_point[0],stop_point[1]);   
 if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN rad/sec
        prev_disL = left_vel*0.1 + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disR = right_vel*0.1 + prev_disR;
        right_vel = right_vel*2/wheelbase;

        Lpwm = pid_velLeft(left_vel, ang_speed);
        Rpwm = pid_velRight(right_vel, ang_speed);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION, 1);    //IN OPP DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_right()
{ //ESP_LOGI(TAG,"FORWARD :CURRENT(x,y):%f %f path point(x,y): %f  %f ",current_point[0],current_point[1],stop_point[0],stop_point[1]);    
if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN rad/sec
        prev_disL = left_vel*0.1 + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disR = right_vel*0.1 + prev_disR;
        right_vel = right_vel*2/wheelbase;

        Lpwm = pid_velLeft(left_vel, ang_speed);
        Rpwm = pid_velRight(right_vel, ang_speed);
        prev_tim = esp_timer_get_time();
    }    

    gpio_set_level(LEFT_MOTOR_DIRECTION, 0);    //IN OPP DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_back()
{   //ESP_LOGI(TAG,"FORWARD :CURRENT(x,y):%f %f path point(x,y): %f %f ",current_point[0],current_point[1],stop_point[0],stop_point[1]);   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disL = left_vel*0.1 + prev_disL;
        prev_disR = right_vel*0.1 + prev_disR;

        Lpwm = pid_velLeft(left_vel, lin_speed);
        Rpwm = pid_velRight(right_vel, lin_speed);
        prev_tim = esp_timer_get_time();
    }

    gpio_set_level(LEFT_MOTOR_DIRECTION, 0);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_stop()
{
    ledc_set_duty(motorL.speed_mode, motorL.channel, 0);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 0);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

/*To reset the PID error variable*/ 
void init_pid(){
    
    current_errorL = 0;
    accumulated_errorL = 0;
    prev_errorL = 0;
    prev_disL = 0;
    current_errorR = 0;
    accumulated_errorR = 0;
    prev_errorR = 0;
    prev_disR = 0;
    
    doneFlag = 0;

    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
   
}

/*Simple PID funtions, can be upgraded while testing according to the performance. Like integral windup etc.*/
int pid_velLeft(double actualvel, double desiredvel){
    current_errorL = desiredvel - actualvel;
    accumulated_errorL += current_errorL;
    double pid = Kp*current_errorL + Kd*(current_errorL-prev_errorL) + Ki*accumulated_errorL;
    //Main part after this is finding the maximum value of PID(after capping) and mapping it to the PWM 
    if (pid>10)
        pid=10;
    else if (pid<0)
        pid=0;
    prev_errorL = current_errorL;
    return map1(pid, 0, 10, 0, 4095);   //set maximum PWM as the top speed required
}

int pid_velRight(double actualvel, double desiredvel){
    current_errorR = desiredvel - actualvel;
    accumulated_errorR += current_errorR;
    double pid = Kp*current_errorR + Kd*(current_errorR-prev_errorR) + Ki*accumulated_errorR;
    if (pid>10)
        pid=10;
    else if (pid<0)
        pid=0;
    prev_errorR = current_errorR;
    //Main part after this is finding the maximum value of PID(after capping) and mapping it to the PWM 
    return map1(pid, 0, 10, 0, 4095);
}

/*Funtion to map pid value to the PWM range*/
int map1(float x, float in_min, float in_max, int out_min, int out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*Get the character to be used for storing the direction*/
char determine(int local_flag)
{
    char c;
    switch(local_flag){
        case -1:c = 's';break;
        case 0: c = 'f';break;
        case 1: c = 'l';break;
        case 2: c = 'r';break;
        case 3: c = 'b';break;
        default:c = 'a';break;
    }
    return c;
}

/*Algorithm for converting path to co-ordinate based format
  Remove DEFAULT_LIN_SPEED and DEFAULT_ANG_SPEED if the stored format is in distance and not time*/
esp_err_t convert_paths(int n){
    char str[LINE_LEN];
    char aux[500] = "", aux1[500] = "";
    int len = 0, linectr = 0, count_flag = 1;
    char ch, temp[9];
    double val;
    struct point prev = {0,0,0};
    struct point current = prev;
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    FILE* f_w = fopen("/spiffs/temp.txt", "w");
    if(f_r == NULL){
        ESP_LOGE(TAG, "Error opening file paths.txt\n");
        return ESP_FAIL;
    }
    if(f_w == NULL){
        ESP_LOGE(TAG, "Error opening file temp.txt\n");
        return ESP_FAIL;
    }
    while(!feof(f_r))
    {
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);
        // ESP_LOGI(TAG, "%s", str);
        // ESP_LOGI(TAG, "Length: %d", strlen(str));
        strcat(aux, str);
        if (count_flag){
            linectr++;
            count_flag = 0;
        }              
        if (strchr(str, '\n')){      //We only increment the linectr when we have \n, because of a single path having multiple lines
            if(linectr == n) //This before the reseting temp so that if deceted it should retain it
            {
                strcpy(aux1,aux);
                ESP_LOGI(TAG, "%s", aux1);
                //Length calculation for dynamic allocation
                char* temp_token = strtok(aux, "\t");
                while(temp_token!=NULL)
                {
                    ch = temp_token[0];
                    temp_token++;
                    if(ch == 'f' || ch == 'b')
                    len ++;                 
                    temp_token = strtok(NULL, "\t");
                }
                char* result = (char *)calloc((len*2*10+1),sizeof(char));
                strcpy(result, "");
                ESP_LOGI(TAG, "Length: %d", len*4*9+1);
                //Result string storage
                char* token = strtok(aux1, "\t");
                while(token!=NULL)
                {
                    ch = token[0];
                    token++;
                    val = atof(token);    //val is in meters 
                    if(ch == 'f'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;// for time based approach 

                        current.x = prev.x + val*cos(prev.theta);
                        current.y = prev.y + val*sin(prev.theta);
                        ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.x);
                        strcat(result, temp);
                        strcat(result, " ");
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.y);
                        strcat(result, temp);
                        strcat(result, " ");
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'b'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;// for time based approach 

                        current.x = prev.x - val*cos(prev.theta);  //don't convert as already in radians when using encoders
                        current.y = prev.y - val*sin(prev.theta);
                        ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.x);
                        strcat(result, temp);
                        strcat(result, " ");
                        strcpy(temp, "");
                        snprintf(temp, 9, "%f", current.y);
                        strcat(result, temp);
                        strcat(result, " ");
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'r'){
                        //val is in radian only
                        prev.theta = prev.theta + val;
                    }
                    else if(ch == 'l'){
                        prev.theta = prev.theta - val;
                    }
                    token = strtok(NULL, "\t");
                }
                strcat(result, "\n");
                ESP_LOGI(TAG, "%s", result);
                fprintf(f_w, "%s", result);
            }
            else
                fprintf(f_w, "%s", aux);
            count_flag=1;
            strcpy(aux, "");
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;
}

void batter_low()
{  
    double reverse[LINE_LEN];
    char data[LINE_LEN];
    char buf[20];
    FILE* f_a = fopen("/spiffs/currentpath.txt", "r");

    if(f_a == NULL){
        ESP_LOGE(TAG, "Error opening file temp.txt\n");
        return ESP_FAIL;
    }
    char str[LINE_LEN];
    char temp[500] = "";

    while(!feof(f_a))
    {     
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_a);
        //printf("%s",str);
        strcat(temp, str);  
        if (strchr(str, '\n'))
        {      //We only increment the linectr when we have \n, because of a single path having multiple lines
            strcat(temp,"");
        }
        //printf("%s",temp);
    }
    char* pointer = strtok(temp, " ");
    int i=0;
    while(pointer!=NULL)
    {     
    if(i<=point_index)					//iterate through each of the elements
    {
        reverse[i] = atof(pointer);       //Convert the value from string to float
        pointer = strtok(NULL, " ");             //Get the next element
        reverse[i+1] = atof(pointer); 
        pointer = strtok(NULL, " "); 
        i=i+2;
    }
    break;
    }
    fclose(f_a);
    remove("/spiffs/currentpath.txt");
    for(i=0;i<point_index;i++){
        sprintf(buf, "%f", reverse[point_index-i-1]);//convert (i+1) to string
        strcat(buf," ");
        strcat(data,buf);
    }
    FILE* f_w =fopen("/spiffs/currentpath.txt", "w");
    if (f_w == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    fprintf(f_w, "%s",data);
    ESP_LOGI(TAG, "path selected: %s length: %d", data, strlen(data));
    fclose(f_w);
    point_index=0;
    rotating_flag=1;
    angle_rotated=dist_traversed=0;
    point_update();
    auto_flag=1;
    



}
void point_update()
{ //auto_flag=1;
    int i=0;
    char str[LINE_LEN];
    //char space="";
    char temp[500] = ""; // Change this temp to calloc
    FILE* f_a = fopen("/spiffs/currentpath.txt", "r");
    if (f_a == NULL) 
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
    }
    while(!feof(f_a))
    {     
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_a);
        //printf("%s",str);
        strcat(temp, str);  
        if (strchr(str, '\n'))
        {      //We only increment the linectr when we have \n, because of a single path having multiple lines
            strcat(temp,"");
        }
        //printf("%s",temp);
    }
    char* pointer = strtok(temp, " ");
    //printf(strlen(temp));
    while(pointer!=NULL)
    {
    if(point_index >= strlen(temp)/2-1)
    {
        auto_flag=0;
        point_index=0;
        fclose(f_a);
        break;
    }
    if(point_index == i )					//iterate through each of the elements
    {
        double xCoor = atof(pointer);       //Convert the value from string to float
        pointer = strtok(NULL, " ");             //Get the next element
        double yCoor = atof(pointer);      
        ESP_LOGI(TAG, "Current (X,Y):(%f, %f) length:%d",xCoor, yCoor, strlen(temp));
        //All the code to reach the destination with obstacle avoidance
        updateParams(xCoor, yCoor);
        fclose(f_a);
        break;
        //pointer = strtok(NULL, " "); 

    }

    else
       {pointer = strtok(NULL, " ");
       pointer = strtok(NULL, " ");
        i=i+2;
        }
    }
}
void reset_automode_values()
{
    angle_rotated=0;
    dist_traversed=0;
    angle_required = 0;                //angle that the bot needs to rotate to align itself with its destination point
    dist_required = 0; 
    current_time=0; // needed wehn for time based approach
    prev_time = 0;                     //stores the previous time step, gets updated to current time after sysCall_sensing
    time_flag = 0;
    current_point[0]=current_point[1]=0;
    stop_point[0] =  stop_point[1] = 0;
    prev_point[0] =prev_point[1]=0 ;               //the next point that the bot needs to travel to    flag = -1; 
}
/*Execute the local_flag th path*/ //auto mode, need to enable the interrupts and use the algorithm
esp_err_t get_path(int local_flag)
{   remove("/spiffs/currentpath.txt");
    reset_automode_values();

    char str[LINE_LEN], temp[500] = ""; // Change this temp to calloc
    int linectr = 0, count_flag = 1;
    
    gpio_intr_enable(LEFT_ENCODERA);    //Enabled when in active mode(ready to move)
    gpio_intr_enable(RIGHT_ENCODERA);

    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    if (f_r == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    /*To get the path in temp variable, in the form of string */
    while(!feof(f_r))
    {
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);

        if(!feof(f_r))
        {
            strcat(temp, str);
            if (count_flag){
                linectr++;
                count_flag = 0;
            }  
                          
            if (strchr(str, '\n')){      //We only increment the linectr when we have \n, because of a single path having multiple lines
                if(linectr == (local_flag+1)) //This before the reseting temp so that if deceted it should retain it
                    break;
                count_flag=1;
                strcpy(temp, "");
            }
        }
    }
    //ESP_LOGI(TAG, "path selected: %s", temp);
    fclose(f_r);
    /*To get x,y coordinates of the path*/
    FILE* f_w =fopen("/spiffs/currentpath.txt", "w");
    if (f_w == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    fprintf(f_w, "%s",temp);
    ESP_LOGI(TAG, "path selected: %s length: %d", temp, strlen(temp));
    fclose(f_w);
    point_update();
    auto_flag = 1;
    /*gpio_intr_disable(LEFT_ENCODERA);  //Disabled interrupt once done  
    gpio_intr_disable(RIGHT_ENCODERA);*/

    return ESP_OK;
}

void updateParams(double xd, double yd)
{   point_index=point_index+2;

    
    ESP_LOGI(TAG,"I AM UPDATING THE POINT ");
    dist_required = sqrt(pow(yd - prev_point[1],2)+pow(xd - prev_point[0],2));  //Distance in meters
    /*The angle calculated here is absolute from positive x-axis, range is from +180 to -180. Clockwise -ve and anti +ve*/
    if (xd-prev_point[0] >= 0){
        angle_required = atan((yd-prev_point[1])/(xd-prev_point[0]))*180/M_PI;
    }
    else{
        angle_required = -atan((xd-prev_point[0])/(yd-prev_point[1]))*180/M_PI;
        if(yd-prev_point[1] >= 0)
            angle_required = angle_required + 90;
        else
            angle_required = angle_required - 90;
    }
    prev_point[0] = xd;
    prev_point[1] = yd;

    dist_traversed = 0;
}

void actuationAuto()
{  
    //dist_traversed=dist_traversed+lin_speed*timediff;

    //ESP_LOGI(TAG,"automode %d",auto_flag);
    if (flag == 0)    {dist_traversed=dist_traversed+lin_speed*timediff;
    }
    else if (flag == 1) {
           angle_rotated = (angle_rotated + (ang_speed*timediff)*180/M_PI);

    }
    else if(flag==2){
        angle_rotated = (angle_rotated - (ang_speed*timediff)*180/M_PI);
    }
    //{   //dist_traversed = (leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick; //In meters //To know if the target has been reached
       // }
    /*else if (flag == 1) //anti clockwise for positive angle
           {//printf("hello");
               angle_rotated = angle_rotated + ang_speed*timediff;
               }
            //angle_rotated = angle_rotated + ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick*2)/wheelbase; //Angle of the bot in radians
    else if (flag==2)          //Clockwise negative angle
            {//printf("hello");
            angle_rotated = angle_rotated - ang_speed*timediff;}*/
            //angle_rotated = angle_rotated - ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick*2)/wheelbase; //Angle of the bot in radians
    
    current_point[0] = stop_point[0] + dist_traversed*cos(angle_rotated); //calculates co-ordinates of the current point using the 
    current_point[1] = stop_point[1] + dist_traversed*sin(angle_rotated); //point where the orientation of the bot was last changed

    if((prev_time>=time_flag+0.5) && (prev_time<time_flag+1))  //this for moving forward slowly after sensing for 
        forwardSlow(1);
    if(detect_flag==0 && prev_time>=time_flag+1)
    	{normal_motion();  
        ESP_LOGI(TAG,"rota: %d distre:%f anreq:%f flag:%d current(x,y):(%f,%f) stop point(x,y):(%f,%f)angle rotat:%f timediff :%f angspeed:%f",rotating_flag,dist_required,angle_required,flag,current_point[0],current_point[1],prev_point[0],prev_point[1],angle_rotated,timediff,ang_speed);
 
        }
    else if(detect_flag==1 && prev_time>=time_flag+1){
        if(obstacle_flag[1]==1 && obstacle_flag[3]==0){
	    	time_flag = prev_time;
	    	rotateSlow(-1);
	    	dist_traversed = 0;
	    	update_stopPoint();
    	}
        else if(obstacle_flag[3]==1 && obstacle_flag[1]==0){
    		time_flag = prev_time;
    		rotateSlow(1);
    		dist_traversed = 0;
	    	update_stopPoint();
    	}
        else if(obstacle_flag[1]==1 && obstacle_flag[3]==1){
    		time_flag = prev_time;
    		rotateSlow(1);
    		dist_traversed = 0;
	    	update_stopPoint();
    	}
        else if(obstacle_flag[1]==0 && obstacle_flag[3]==0 && obstacle_flag[2]==1){
    		time_flag = prev_time;
    		rotateSlow(1);
            dist_traversed = 0;
            update_stopPoint();
        }
        else if(obstacle_flag[0]==1 || obstacle_flag[4]==1){
            time_flag = prev_time;
            forwardSlow(5);
    	}
        else if(obstacle_flag[0]==0 && obstacle_flag[4]==0){
    		recalculate();
    		detect_flag = 0;
    	}
    }
    
}

void normal_motion(){
    lin_speed = DEFAULT_LIN_SPEED;
    ang_speed = DEFAULT_ANG_SPEED;
    if(rotating_flag==1){
        //double val = get_PID_angle(angle_rotated, angle_required);
        if(doneFlag==1){
            init_pid();
            flag=-1;
            rotating_flag = 0;
        }else{
            rotate();
        }
    }else{
        //double val = get_PID_dist(dist_traversed, dist_required);
        if(doneFlag==1){
            init_pid();
            flag=-1;
            point_update();
            update_stopPoint();
            rotating_flag = 1;
        }else{
            forward();
        }
    }
}

void rotate()
{   if((angle_required - angle_rotated) > -7 && (angle_required - angle_rotated) < 7)  //set some threshold after which it can move 
        doneFlag = 1;
    if((angle_required - angle_rotated) > 5)
       { //printf("data:%f",(angle_required - angle_rotated));
        flag = 1; }   //If diff is positive, then left(+ve for anticlockwise)
    else if((angle_required - angle_rotated) < -5)
        {//printf("data:%f",(angle_required - angle_rotated));
        flag = 2; }


}

void forward(){
    flag = 0; 
    if((dist_required - dist_traversed)> -0.1 && (dist_required - dist_traversed)<0.1)  //set some threshold after which it can move 
        doneFlag = 1;
}

void update_stopPoint(){
    stop_point[0] = current_point[0];
    stop_point[1] = current_point[1];
}

void recalculate(){
    dist_required = sqrt(pow(prev_point[1]-current_point[1], 2) + pow(prev_point[0]-current_point[0], 2)); 

    if(prev_point[0]-current_point[0] >= 0){
        angle_required = atan((prev_point[1]-current_point[1])/(prev_point[0]-current_point[0]))*180/M_PI;
    }
    else{
        angle_required = -atan((prev_point[0]-current_point[0])/(prev_point[1]-current_point[1]))*180/M_PI;
        if(prev_point[1]-current_point[1] >= 0)
            angle_required = angle_required + 90;
        else 
            angle_required = angle_required - 90;
    }
    update_stopPoint();
    dist_traversed = 0;
    init_pid();
    rotating_flag = 1;
}

void sensing(){
    prev_time = esp_timer_get_time()/1000000;    //Getting current time in microseconds and storing in seconds
    timediff = prev_time-current_time;
    current_time=prev_time;
    //prev_time = esp_timer_get_time()/1000000;    //Getting current time in microseconds and storing in seconds
    
    // used to detect the obstacle as the ultrasonic sensors goes low when obstacle is present so is the obstacle flag 
    //register and resets back when the original state is returned
   	/*obstacle_flag[0] = !gpio_get_level(ULTRA1);
   	obstacle_flag[1] = !gpio_get_level(ULTRA2);
   	obstacle_flag[2] = !gpio_get_level(ULTRA3);
   	obstacle_flag[3] = !gpio_get_level(ULTRA4);
   	obstacle_flag[4] = !gpio_get_level(ULTRA5);
    */
    detect_flag = obstacle_flag[1] | obstacle_flag[2] | obstacle_flag[3];
    //ESP_LOGI(TAG," DETECTION FLAG: %d",detect_flag);
}

void forwardSlow(int num){
    flag = 0;
    lin_speed = MIN_LINEAR_SPEED*num;
}

void rotateSlow(int num){
    ang_speed = MIN_ANGULAR_SPEED*abs(num);
    if(num > 0)
        flag = 1;    //left
    else 
        flag = 2;    //right
}

