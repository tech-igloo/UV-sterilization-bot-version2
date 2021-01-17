#include "algo.h"
#include "server.h"

static ledc_channel_config_t led_channel[2];       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output. Will have to create another for controlling 2 motors
#define rightm 13    //used to enable right motor (when high motor starts working when low the motor is turned off)
#define right_dir 12 // used for controlling the direction of right motor 
#define rpwm 14      // PWM pin which controls the speed of the right motor 
#define leftm 27     //used to enable the left motor (when high motor starts working when low the motor is turned off)
#define left_dir 26  //used for controlling the direction of left motor
#define lpwm 25      //used for controlling the speed of left motor 
struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};
int pid_flag;
int stop_point[10];
int current_point[10];
int prev_point[10]; 
int rotation_flag;
float current_error;
float accumulated_error;
float prev_error;
double path_points[20][20];
double dist_required;
double dist_traversed;
int angle_required;
char temp_path[LINE_LEN];
int timediff;//can be float or double
/*Used for setting up PWM Channel (Ref: Official Github Repo)*/
void init_pwm()
{ //gpio configuration for motor enable and direction control
    gpio_reset_pin(rightm);
    gpio_reset_pin(right_dir);

    gpio_set_direction(rightm, GPIO_MODE_OUTPUT);
    gpio_set_direction(right_dir, GPIO_MODE_OUTPUT);

    gpio_reset_pin(leftm);
    gpio_reset_pin(left_dir);

    gpio_set_direction(leftm, GPIO_MODE_OUTPUT);
    gpio_set_direction(left_dir, GPIO_MODE_OUTPUT);

    ledc_timer_config_t led_timer = {
            .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty, means PWM value can go from 0 to (2^13-1)
            .freq_hz = 5000,                      // frequency of PWM signal
            .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
            .timer_num = LEDC_TIMER_0,            // timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
    ledc_timer_config(&led_timer);
    //configuring pwm pins to timer 
    ledc_channel_config_t led_channel[2] = 
    {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = rpwm,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = lpwm,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
    };
    for( int var = 0;var < 2; var++) {
        ledc_channel_config(&led_channel[var]);
    }
}
void init_pid()
{
    current_error=0;
    accumulated_error=0;
    prev_error=0;
    pid_flag=0;

}
/*The following are dummy functions for movement of the bot*/
void move_forward()
{   
    //ESP_LOGE("forward");
    gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 1);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 1);
    //Assuming that motors shows ideal responce when same pwm is given 
    for (int var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,650);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }    
}  //Similar functions below with different duty cycles

void move_left()
{    
   // ESP_LOGE( "left");
    gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 0);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 1);
    for (int var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,650);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }  
}

void move_right()
{   
   // ESP_LOGE("right");
    gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 1);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 0);
    for (int var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,650);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }   
}

void move_back()
{   
    //ESP_LOGE("back");
    gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 0);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 0);
    for (int var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,650);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }     
}

void move_stop()
{   
   // ESP_LOGE( "stop");
    gpio_set_level(rightm, 0);
    gpio_set_level(leftm, 0);

}

/*Execute the local_flag th path. get_path is mainly used in auto mode for retriving data from the file
to be completed by intregrating with encoders and ultrasonic sensors*/
esp_err_t get_path(int local_flag)
{ 
    int linectr = 0;
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    if (f_r == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    //gets the nth path form file by constantly copying string to str variable and breacking out ofy

    while(!feof(f_r))
    {   
        strcpy(temp_path, "\0");
        fgets(temp_path , LINE_LEN, f_r);
        if(!feof(f_r))
        {
            linectr++;
            if(linectr == (point_index+1))//1st line contains the number of valid paths, so nth path will be on (n+1)th line
            { 
              linectr=0;
               for(int i=0;i<strlen(temp_path);i=i+2)
               {   linectr++;
                   path_points[linectr][0 ]=  atof(temp_path[i]);
                   path_points[linectr][ 1 ]=  atof(temp_path[i+1]);
               }
               break;
            }    
        }
    }

    fclose(f_r);
    //return ESP_OK;
    //char* token = strtok(str, "\t");    //The elements are seperated by "\t"
    auto_flag = 1;                      
 /*   while(token!=NULL)					//iterate through each of the elements
    {
        char ch = token[0];             //Get the first character which denotes the direction to be travelled
        switch(ch){
            case 'f': move_forward();
                        flag = 0; break;	//The flag values are set here. The infinite Loop in Task 1 checks these flag variables and calls the appropriate functions
            case 'l': move_left();
                        flag = 1; break;
            case 'r': move_right();
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
        token = strtok(NULL, "\t");             //Get the next element
    }
    //move_stop();
    auto_flag = 0;*/
    return ESP_OK;
}

void update_points()
{   
    dist_required =(double)sqrt(pow((path_points[point_index][2]-prev_point[1]),2) - pow((path_points[point_index][1]-prev_point[2]),2));
    if(path_points[point_index][1]-prev_point[1]>=0)
    { 
        angle_required=atan((prev_point[2]-current_point[2])/(prev_point[1]-current_point[1]));
    }
    else
    {
         angle_required=(-1)*(atan((prev_point[1]-current_point[1])/(prev_point[2]-current_point[2])));
         if(path_points[point_index][2]-prev_point[2]>=0)
         {
              angle_required= angle_required+90;
         }
  
    }
    prev_point[0] = path_points[point_index][0];
    prev_point[1] = path_points[point_index][1];
    point_index++;
    if(point_index> strlen(temp_path)/2){
        point_index =0;
    }
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
    dist_required=(double) sqrt(pow((prev_point[2]-current_point[2]),2)+pow((prev_point[1]-current_point[1]),2));
    if(prev_point[1]-current_point[1] >= 0){
        angle_required=atan((prev_point[2]-current_point[2])/(prev_point[1]-current_point[1]));
    }
    else
    {
         angle_required=(-1)*(atan((prev_point[1]-current_point[1])/(prev_point[2]-current_point[2])));
         if(prev_point[2]-current_point[2] >= 0)
         {
              angle_required= angle_required-90;
         }
    }  
    update_stopPoint();
    dist_traversed=0;
    init_pid();
    rotation_flag=1;
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
/*Algorithm for converting path to co-ordinate based format
  Remove DEFAULT_LIN_SPEED and DEFAULT_ANG_SPEED if the stored format is in distance and not time*/
esp_err_t convert_paths(int n){
    char str[LINE_LEN];
    char str1[LINE_LEN];
    int len = 0, linectr = 0;
    char ch, temp[9];
    int val, counter;//double
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
            linectr++;
            if(linectr == n)
            {   
                fseek(f_r, -strlen(str), SEEK_CUR);//point to the nth position in the string 
                fgets(str1, LINE_LEN, f_r);//copy the string from position pointed in the above line to str1 variable
                char* temp_token = strtok(str1, "\t");// split the string based on the delimiter
                while(temp_token!=NULL)
                {
                    ch = temp_token[0];
                    temp_token++;//As the first place is a character to detrmine the direction followed by encoder ticks we increment the pointer
                    val = DEFAULT_LIN_SPEED * atof(temp_token)/1000.0;//atof converts the string to number pointed by the pointer
                    if(ch == 'f' || ch == 'b'){
                        len += ceil(val/resolution);}
                    temp_token = strtok(NULL, "\t");
                }
                char* result = (char *)calloc((len*2*10+1),sizeof(char));
                //strcpy(result,"");
                //char* result = (char *)calloc(2048,sizeof(char));
                strcpy(result, "");
                ESP_LOGI(TAG, "Length: %d", len*4*9+1);
                char* token = strtok(str, "\t");
                while(token!=NULL)
                {
                    ch = token[0];
                    token++;
                    val = atof(token);
                    if(ch == 'f'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;
                        //counter = resolution;
                        counter =val;
                       // while(counter == val){
                            current.x = prev.x + counter*cos(prev.theta*3.14/180.0);
                            current.y = prev.y + counter*sin(prev.theta*3.14/180.0);
                            ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                            //counter = counter + resolution;
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.x);
                            strcat(result, temp);
                            strcat(result, " ");
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.y);
                            strcat(result, temp);
                            strcat(result, " ");
                       // }
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'b'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;
                        //counter = resolution;
                        counter = val;
                      // while(counter == val){
                            current.x = prev.x - counter*cos(prev.theta*3.14/180.0);
                            current.y = prev.y - counter*sin(prev.theta*3.14/180.0);
                            ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                            //counter = counter + resolution;
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.x);
                            strcat(result, temp);
                            strcat(result, " ");
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.y);
                            strcat(result, temp);
                            strcat(result, " ");
                      //  }
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'r'){
                        val = DEFAULT_ANG_SPEED * val/1000.0;
                        prev.theta = prev.theta + val;
                    }
                    else if(ch == 'l'){
                        val = DEFAULT_ANG_SPEED * val/1000.0;
                        prev.theta = prev.theta - val;
                    }
                    token = strtok(NULL, "\t");
                }
                //strcat(result, "\b");
                strcat(result, "\n");
                ESP_LOGI(TAG, "%s", result);
                fprintf(f_w, "%s", result);
            }
            else
                fprintf(f_w, "%s", str);
        //}
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;
}
