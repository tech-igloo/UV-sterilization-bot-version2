#include "server.h"
#include "algo.h"
#include<math.h>

int Lpwm = 0; 
int Rpwm = 0;     //Always remember you can't define the variable in .h file     

int leftRot = 0;
int leftTicks = 0;
int rightRot = 0;
int rightTicks = 0;
int lin_vel=5;
int ang_vel=4;
int time_flag=0;

double left_vel = 0;
double right_vel = 0;

int64_t prev_time;
int detect_flag=0;

double prev_disL = 0;
double prev_disR = 0;
int64_t prev_tim = 0;

double current_errorL = 0;
double accumulated_errorL = 0;
double prev_errorL = 0;
double current_errorR = 0;
double accumulated_errorR = 0;
double prev_errorR = 0;
int timediff;
float current_error;
float accumulated_error;
float prev_error;
int point_index=0;
double path_points[20];
double dist_traversed;
int stop_point[10];
int current_point[10];
int prev_point[10]; 
int rotation_flag;
int pid_flag;
static double dist_required = 0;                 //distance from stop point that the bot needs to travel to reach the destination point
static double angle_required = 0;   
char *str_path[LINE_LEN];
char temp_path[LINE_LEN];
int rotating_flag=1;
static double angle_rotated = 0;                 //angle rotated by bot(+ve for clockwise and -ve for anticlockwise)

int point_index;
int sensor_readings[5];//for storing the readings 

double Kp = 1;  //common gains for both the PID blocks for now
double Kd = 1/sampleTimeInSec;
double Ki = 1*sampleTimeInSec;
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
    
   // .intr_type=GPIO_INTR_DISABLE;

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(LEFT_ENCODERA, gpio_encoder_isr_handler, (void*) LEFT_ENCODERA);  //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RIGHT_ENCODERA, gpio_encoder_isr_handler, (void*) RIGHT_ENCODERA);
    
    gpio_evt_queue = xQueueCreate(20, sizeof(uint32_t));    //gpio interrupt queue

    gpio_intr_disable(LEFT_ENCODERA);  //Enabled when recording or in auto mode  
    gpio_intr_disable(RIGHT_ENCODERA);

    gpio_set_direction(LEFT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);
}

/*Used for setting up PWM Channel (Ref: Official Github Repo)*/
void init_pwm()
{
    gpio_reset_pin(RIGHT_MOTOR_ENABLE);
    gpio_reset_pin(RIGHT_MOTOR_DIRECTION);

    gpio_set_direction(RIGHT_MOTOR_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);

    gpio_reset_pin( LEFT_MOTOR_ENABLE);
    gpio_reset_pin(LEFT_MOTOR_DIRECTION);

    gpio_set_direction(LEFT_MOTOR_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEFT_MOTOR_DIRECTION, GPIO_MODE_OUTPUT);
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
    uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);// Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: pin no change, CTS: pin no change )
    uart_driver_install(UART_NUM_2,  2*1024, 0, 0, NULL, 0);
    for(int i=0;i<7;i++){
        const char *data =&(enpwmcmd[i]);
        uart_write_bytes(UART_NUM_2,data,strlen(data));
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
    uart_driver_delete(UART_NUM_2);

}
/*The following are dummy functions for movement of the bot*/
void move_forward()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disL = left_vel*0.1 + prev_disL;
        prev_disR = right_vel*0.1 + prev_disR;

        Lpwm = pid_velLeft(left_vel, DEFAULT_LIN_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_LIN_SPEED);
        prev_tim = esp_timer_get_time();
    }
gpio_set_level(LEFT_MOTOR_ENABLE,1);//TO ENABLE THE MOTORS NOT REQUIRED TOO MANY TIMES ..
gpio_set_level(RIGHT_MOTOR_ENABLE,1);
    gpio_set_level(LEFT_MOTOR_DIRECTION, 1);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);   //Update the PWM value to be used by this lED Channel.
    ledc_update_duty(motorL.speed_mode, motorL.channel);      //Use the Updated PWM values
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}                                                                       

void move_left()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN rad/sec
        prev_disL = left_vel*0.1 + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disR = right_vel*0.1 + prev_disR;
        right_vel = right_vel*2/wheelbase;

        Lpwm = pid_velLeft(left_vel, DEFAULT_ANG_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_ANG_SPEED);
        prev_tim = esp_timer_get_time();
    }
gpio_set_level(LEFT_MOTOR_ENABLE,1);
gpio_set_level(RIGHT_MOTOR_ENABLE,1);
    gpio_set_level(LEFT_MOTOR_DIRECTION, 1);    //IN OPP DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_right()
{
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN rad/sec
        prev_disL = left_vel*0.1 + prev_disL;
        left_vel = left_vel*2/wheelbase;    //angular velocity
        
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disR = right_vel*0.1 + prev_disR;
        right_vel = right_vel*2/wheelbase;

        Lpwm = pid_velLeft(left_vel, DEFAULT_ANG_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_ANG_SPEED);
        prev_tim = esp_timer_get_time();
    }    
gpio_set_level(LEFT_MOTOR_ENABLE,1);
gpio_set_level(RIGHT_MOTOR_ENABLE,1);
    gpio_set_level(LEFT_MOTOR_DIRECTION, 0);    //IN OPP DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 1);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_back()
{   
    if((esp_timer_get_time() - prev_tim) >= sampleTime){
        left_vel = ((leftRot*ENCODERresolution + leftTicks)*wheeldist_perTick - prev_disL)/0.1;  // VELCOTIY IN m/sec
        right_vel = ((rightRot*ENCODERresolution + rightTicks)*wheeldist_perTick - prev_disR)/0.1;
        prev_disL = left_vel*0.1 + prev_disL;
        prev_disR = right_vel*0.1 + prev_disR;

        Lpwm = pid_velLeft(left_vel, DEFAULT_LIN_SPEED);
        Rpwm = pid_velRight(right_vel, DEFAULT_LIN_SPEED);
        prev_tim = esp_timer_get_time();
    }
gpio_set_level(LEFT_MOTOR_ENABLE,1);
gpio_set_level(RIGHT_MOTOR_ENABLE,1);
    gpio_set_level(LEFT_MOTOR_DIRECTION, 0);    //BOTH IN SAME DIRECTION
    gpio_set_level(RIGHT_MOTOR_DIRECTION, 0);
    ledc_set_duty(motorL.speed_mode, motorL.channel, Lpwm);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, Rpwm);
    ledc_update_duty(motorR.speed_mode, motorR.channel);
}

void move_stop()
{gpio_set_level(LEFT_MOTOR_ENABLE,0);
gpio_set_level(RIGHT_MOTOR_ENABLE,0);
    /*ledc_set_duty(motorL.speed_mode, motorL.channel, 0);
    ledc_update_duty(motorL.speed_mode, motorL.channel);
    ledc_set_duty(motorR.speed_mode, motorR.channel, 0);
    ledc_update_duty(motorR.speed_mode, motorR.channel);*/
}

void init_pid(){
    current_errorL = 0;
    accumulated_errorL = 0;
    prev_errorL = 0;
    prev_disL = 0;
                        prev_tim = 0;
    current_errorR = 0;
    accumulated_errorR = 0;
    prev_errorR = 0;
    prev_disR = 0;

    current_error=0;
    accumulated_error=0;
    prev_error=0;
    pid_flag=0;
}

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
int get_pid_dist(int current_val,int target_val)
{ int local_val;//for pid 
    current_error=target_val-current_val;
    accumulated_error=accumulated_error+current_error*timediff;
    local_val=500*current_error + 0.1*accumulated_error ;
    if(local_val>5)local_val=5;
    else if(local_val<5)local_val=-5;
    if(abs(local_val)<0.1)pid_flag=1;
    prev_error=current_error;
    return(local_val);
}
int get_pid_angle(int current_val,int target_val){
    current_error=target_val-current_val;
    accumulated_error=accumulated_error+current_error*timediff;
    int local_val=0;
    if(timediff >0 && timediff <0.1 )
    {
        local_val = 0.1*current_error + 0.0005*accumulated_error + 0.001*(current_error-prev_error)/timediff; //kp=0.1 ki=0.0005 kd=0.001
    }
    else if(timediff==0){
        local_val = 0.1*current_error + 0.0005*accumulated_error ;
    }
    if(local_val>5)local_val=5;
    else if(local_val<5)local_val=-5;
    if(abs(local_val)<0.005)pid_flag=1;
    prev_error=current_error;
    return(local_val);
}
void path_update(){
    char str[LINE_LEN], temp[500] = "";
    FILE* f_r = fopen("/spiffs/path.txt", "r");
    while(f_r !=NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        //return ESP_FAIL;
    
    while(!feof(f_r))
    {
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);

        if(!feof(f_r))
        {
            strcat(temp, str);
                          
            if (strchr(str, '\n'))
            {      //We only increment the linectr when we have \n, because of a single path having multiple lines
                strcpy(temp, "");
            }
        }
    }
    }
    char* token = strtok(temp, "\t");    //The elements are seperated by "\t"
    int i=0;
    while(token!=NULL){
        if (i==point_index){
            char ch=token[i];
            path_points[0]=ch;
            ch=token[i+1];
            path_points[1]=ch;
            
            break;
            }
        i=i+2;
    }
    if(point_index>i){
        auto_flag=0;
    }
    fclose(f_r);
    
}
void update_points()
{   path_update();
    dist_required =(double)sqrt(pow((path_points[0]-prev_point[1]),2) - pow((path_points[0]-prev_point[2]),2));
    if(path_points[0]-prev_point[1]>=0)
    { 
        angle_required=atan((prev_point[1]-current_point[2])/(prev_point[0]-current_point[1]));
    }
    else
    {
         angle_required=(-1)*(atan((prev_point[1]-current_point[1])/(prev_point[2]-current_point[2])));
         if(path_points[2]-prev_point[2]>=0)
         {
              angle_required= angle_required+90;
         }
  
    }
    prev_point[0] = path_points[0];
    prev_point[1] = path_points[1];
    point_index=point_index+2;
    /*if(point_index> strlen(temp_path)/2){
        point_index =0;
    }*/
    dist_traversed=0;
    init_pid();
}
void update_stopPoint()
{
    stop_point[0]=current_point[0];
    stop_point[1]=current_point[1];
}
void recalculate()
{
    dist_required=(double) sqrt(pow((prev_point[1]-current_point[1]),2)+pow((prev_point[0]-current_point[0]),2));
    if(prev_point[0]-current_point[0] >= 0){
        angle_required=atan((prev_point[1]-current_point[2])/(prev_point[0]-current_point[1]));
    }
    else
    {
         angle_required=(-1)*(atan((prev_point[1]-current_point[1])/(prev_point[1]-current_point[1])));
         if(prev_point[1]-current_point[1] >= 0)
         {
              angle_required= angle_required-90;
         }
    }  
    update_stopPoint();
    dist_traversed=0;
    init_pid();
    rotation_flag=1;
}
void normal_motion(){
    if(rotating_flag==1){
        double val = get_pid_angle(angle_rotated, angle_required);
        if(pid_flag==1){
            init_pid();
            move_stop();//change to flag
            rotating_flag = 0;
        }else{
            move_right(val);//change needed
        }
    }else{
        double val = get_pid_dist(dist_traversed, dist_required);
        printf("VAL: %.17g\n", val);
        if(pid_flag==1){
            init_pid();
            move_stop();//change to flag
            update_points();
            update_stopPoint();
            rotating_flag = 1;
        }else{
            move_forward();//change to flag
        }
    }
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

int map1(float x, float in_min, float in_max, int out_min, int out_max) {
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
        //if(!feof(f_r))
        //{
        // ESP_LOGI(TAG, "%s", str);
        // ESP_LOGI(TAG, "Length: %d", strlen(str));
        //}
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
                //Length calc
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
                //result string storage
                char* token = strtok(aux1, "\t");
                while(token!=NULL)
                {
                    ch = token[0];
                    token++;
                    val = atof(token);
                    if(ch == 'f'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;   //units needs to be change with encoder
                        current.x = prev.x + val*cos(prev.theta*3.14/180.0);
                        current.y = prev.y + val*sin(prev.theta*3.14/180.0);
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
                        val = DEFAULT_LIN_SPEED * val/1000.0;
                        current.x = prev.x - val*cos(prev.theta*3.14/180.0);  //don't convert as already in pi when using encoders
                        current.y = prev.y - val*sin(prev.theta*3.14/180.0);
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
                        val = DEFAULT_ANG_SPEED * val/1000.0;    //you'll get it in 2pi terms itself
                        prev.theta = prev.theta + val;
                    }
                    else if(ch == 'l'){
                        val = DEFAULT_ANG_SPEED * val/1000.0;
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

/*Execute the local_flag th path*/
esp_err_t get_path(int local_flag)
{
    char str[LINE_LEN], temp[500] = ""; // Change this temp to calloc
    int linectr = 0, count_flag = 1;
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    FILE* f_w = fopen("/spiffs/path.txt", "w");
    if (f_r == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    if(f_w==NULL){
        ESP_LOGE(TAG,"Failed to open file for writing");
        return ESP_FAIL;
    }
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
                          
            if (strchr(str, '\n'))
            {      //We only increment the linectr when we have \n, because of a single path having multiple lines
                if(linectr == (local_flag+1)) //This before the reseting temp so that if deceted it should retain it
                    break;
                count_flag=1;
                strcpy(temp, "");
            }
        }
    }
    ESP_LOGI(TAG, "%s", temp);
    fclose(f_r);
    //return ESP_OK;
    fprintf(f_w,"%s",temp); 
    fclose(f_w); 
    //char* token = strtok(temp, "\t");    //The elements are seperated by "\t"
    //auto_flag = 1; 
    
    path_update();

/*
    while(token!=NULL)					//iterate through each of the elements
    { 
        
        
        
        
        
        
        char ch = token[0];             //Get the first character which denotes the direction to be travelled
        switch(ch){
            case 'f'://move_forward();break;
                        flag = 0; break;	//The flag values are set here. The infinite Loop in Task 1 checks these flag variables and calls the appropriate functions
            case 'l'://move_left();break;
                        flag = 1; break;
            case 'r'://move_right();break;
                        flag = 2; break;
            case 'b'://move_back();break;
                        flag = 3; break;
            default://move_stop();break;
                        flag = 4; break;
        }
        ESP_LOGI(TAG, "Direction: %c", ch);
        token++;                        //Increment the pointer to get the numerical value stored after the first character
        float time = atof(token);       //Convert the value from string to float
        ESP_LOGI(TAG, "Time: %f", time);
        vTaskDelay(time/portTICK_PERIOD_MS);    //Wait for the appropriate time
        token = strtok(NULL, "\t");             //Get the next element*/
    
    //move_stop();
    //auto_flag = 0;
    return ESP_OK;
}
void actuation(){
    dist_traversed = dist_traversed + lin_vel*timediff; //update the distance travlled using the linear velocity calculated in sysCall_sensing
    angle_rotated = angle_rotated + ang_vel*timediff*180.0/M_PI ;//  --update the orientation(in degrees, +ve for clockwise an -ve for anticlockwise)
    current_point[1] = stop_point[1] + dist_traversed*cos((angle_rotated*M_PI)/180);// --calculates co-ordinates of the current point using the point where the orientation of the bot was last changed
    current_point[2] = stop_point[2] + dist_traversed*sin((angle_rotated*M_PI)/180);
    if (prev_time >=time_flag+0.5 && prev_time <time_flag+1){
        move_forward();
    }
    if (detect_flag ==0 && prev_time >= time_flag+1){normal_motion();}
    else if (detect_flag ==1 && prev_time >= time_flag+1){
        if(sensor_readings[1]==1 && sensor_readings[3]==0){
            time_flag=prev_time;
            move_left();
            dist_traversed =0;
            update_stopPoint();
        }
        else if (sensor_readings[3]==1 && sensor_readings[1]==0){
            time_flag= prev_time;
            move_right();
            dist_traversed=0;
            update_stopPoint();
        }
        else if (sensor_readings[1]==1 && sensor_readings[3]==1 ){
            time_flag= prev_time;
            move_right();
            dist_traversed=0;
            update_stopPoint();
        }
        else if (sensor_readings[1]==0 && sensor_readings[3]==0 && sensor_readings[2]==0){
            time_flag= prev_time;
            move_right();
            dist_traversed=0;
            update_stopPoint();
        }
        else if (sensor_readings[1]==0 && sensor_readings[3]==0 && sensor_readings[2]==0){
            recalculate();
            detect_flag=0;
        }
    }
    
}   

void sensing(){
    
    int64_t curr_time= esp_timer_get_time();
    timediff=curr_time-prev_time;
    prev_time=curr_time;
    for (int i =0;i<5;i++){sensor_readings[i]=0;}
    if(gpio_get_level(ULTRA1) == 0)sensor_readings[0]=1;
    if(gpio_get_level(ULTRA2)== 0){sensor_readings[1]=1;detect_flag=1;}
    if(gpio_get_level(ULTRA3)== 0){sensor_readings[2]=1;detect_flag=1;}
    if(gpio_get_level(ULTRA4)== 0){sensor_readings[3]=1;detect_flag=1;}
    if(gpio_get_level(ULTRA5)== 0)sensor_readings[4]=1;

 
}