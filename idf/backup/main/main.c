/*The following header files are included*/
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <esp_http_server.h>
#include <string.h>
#include <esp_log.h>
#include <sys/param.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_spiffs.h"
#include <stdbool.h>
#include <stdio.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "mdns.h"
#include "driver/ledc.h"
#include <math.h>

/*The various parameters that are used throughout the code*/
#define EXAMPLE_ESP_MAXIMUM_RETRY 5             //Maximum number of times the esp will try to connect to a network in STA mode
#define DEFAULT_SSID "myssid"                   //Default Network Name used in SAP and STA mode 
#define DEFAULT_PASS "qwerty1234"               //Default Password used in SAP and STA mode
#define MAX_CONN 5                              //Maximum number of devices that can connect to the ESP-32 in SAP mode
#define BASE_PATH "/spiffs"                     //Base path of the SPIFFS partition where all the Network and Path Files are Stored
#define MAX_FILES 5                             //Maximum number of files to be stored in the '/spiffs' partition
#define RESET_FLAG true                         //Whether '/spiffs' partition should be formatted if mount fails
#define WIFI_CONNECTED_BIT BIT0                 //Constants used to check whether connection to a network in STA mode was succesful ot not
#define WIFI_FAIL_BIT      BIT1                 //Constants used to check whether connection to a network in STA mode was succesful ot not
#define WIFI_NUM 5                              //Number of wifi credentials for STA mode to be stored
#define SSID_LEN 32                             //The maximum length of ssid that can be used by the esp is 32 (cannot be changed)
#define PASS_LEN 64                             //The maximum length of password that can be used by the esp is 64 (cannot be changed)
#define DATA_LEN 108                            //108 = 4 + 1 + 32 + 1 + 4 + 1 + 64 + 1 (The maximum length of a header data being sent during saving of a network credential)
#define LINE_LEN 98                             //98 = 32 + 1 + 64 + 1 (The maximum length of a sentence in 1 line in the network credential storage file)
#define PATH_NUM 5                              //The maximum number of paths that can be stored
#define resolution 0.1                            //The resolution of points used in the co-ordinate system
#define DEFAULT_LIN_SPEED 1.5                   //Temporary constants I used. To be deleted when encoder feedback is used
#define DEFAULT_ANG_SPEED 10                    //Temporary constants I used. To be deleted when encoder feedback is used

/*The variables that keep track of various things while the code is running*/
static EventGroupHandle_t s_wifi_event_group;   //Used for Wifi connections during SAP and STA mode
static const char *TAG = "wifi station";        //Name used for ESP_LOGI statements. Feel free to set it to whatever you want
static int flag = -1;                           //Used for determining in which direction it is travelling
static int s_retry_num = 0;                     //Number of times the esp has tried to connect to a network in STA mode
static int64_t prev_mili = 0;                   //Used for determining the time duration between 2 commands while controlling the bot in manual mode
static int64_t curr_mili = 0;                   //Used for determining the time duration between 2 commands while controlling the bot in manual mode
static float time_duration = 0;                 //Used for storing the time duration between 2 commands while controlling the bot in manual mode
static char buf[DATA_LEN];                      //Used for extracting ssid from POST header data whenever User stores new Network Credentials
static char copy[DATA_LEN];                     //Used for extracting password from POST header data whenevr User stores new Network Credentials
static int conn_flag = 0;                       //Denote which Wifi mode the ESP is used : 0 = SAP, 1 to WIFI_NUM = STA wifi index
static int record_flag = 0;                     //Denote whether path is being recorded or not
static char ssid[WIFI_NUM][SSID_LEN];           //Store all the network ssids that the ESP can use in STA mode
static char pass[WIFI_NUM][PASS_LEN];           //Store all the network passwords that the ESP can use in STA mode
static char EXAMPLE_ESP_WIFI_SSID[SSID_LEN];    //Store the network ssid that the esp is using currently in STA mode
static char EXAMPLE_ESP_WIFI_PASS[PASS_LEN];    //Store the network password that the esp is using currently in STA mode
static int total = 0;                           //Total number of network credentials for STA mode stored till now
static char line_str[LINE_LEN];                 //Used for extracting lines from stored files
static int total_paths = 0;                     //Total number of paths stored till now
static int auto_flag = 0;                       //Denote whether auto mode is on or off
static ledc_channel_config_t led_channel;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output. Will have to create another for controlling 2 motors
static TaskHandle_t Task1;                      //Task handle to keep track of created task running on Core 1

/*Variables for obstacle avoidance*/
static double* points = NULL;                   //stores the points in the path
static int points_len = 0;                      //length of the path
static double prev_time = 0;                    //stores the previous time step, gets updated to current time in sensing loop
static double timeDiff = 0;                     //time difference between two sensing time steps
static double dist_traversed = 0;               //distance travelled by bot along straight line(+ve for forward and -ve for negative)
static double angle_rotated = 0;                //angle rotated by bot(+ve for clockwise and -ve for anticlockwise)
static double lin_vel = 0;                      //linear velocity of the bot
static double ang_vel = 0;                      //angular velocity of the bot
static int point_index = 0;                     //index for accessing the appropriate point from points list
static double accumulated_error = 0;            //integral term in PID formula
static double current_error = 0;                //current error in PID formula
static double prev_error = 0;                   //error in the previous time step
static int pid_flag = 0;                        //flag that signifies that the PID controller's job is done
static double angle_required = 0;               //angle that the bot needs to rotate to align itself with its destination point
static double dist_required = 0;                //distance from stop point that the bot needs to travel to reach the destination point
static int rotating_flag = 1;                   //flag that signifies whether the bot should rotate or move along a straight line
static double stop_point[2] = {0};              //the position of the bot from which distance travlled is being calculated
static double prev_point[2] = {0};              //the next point that the bot needs to travel to
static double current_point[2] = {0};           //current co-ordinates of the bot*/
static int obstacle_flag[5] = {0};              //1 at the ith index indicates detection of obstacle by sensor[i]
static double time_flag = 0;                    //used for simulatiing delays
static int detect_flag = 0;                     //flag to denote whether obstacle has been detected by any of the front 3 sensors
static double minDist = 0.2;                    //thresholf distance for detection of an obstacle
static double proxSensDist[5] = {0.5};          //stores distance of obstacle from each sensor


/*Placeholder functions*/
void rotate(double val){
	return NULL;
}

void forward(double val){
	return NULL;
}

void stop(){
	return NULL;
}

/*Creates coordinate array of the path*/
double* create_coord_array(int n){
// int main(){
	int path_number = 0;
	//int n = 3;
    int line_length = 0;
    double val = 0;
    char c;
    FILE* f_r = fopen("./test.txt", "r");
    if(f_r == NULL){
        printf("Error opening file temp.txt\n");
    }
    // Have to handle the invalid path case
    int counter_to_path_start=0;
    int space_count = 0;
    for(;;){
        if(path_number+1==n){
            while(1){
                c = fgetc(f_r);
                if(c == ' ')
                    space_count+=1;
                if( c == EOF || c == '\n' ){
                    ++line_length;
                    break;
                }
                ++line_length;
            }
            fseek(f_r, -(line_length+1), SEEK_CUR);
            //c = fgetc(f_r);
            //printf("%c THE CHAR", c);
            char* path = (char *)calloc(line_length, sizeof(char));
            strcpy(path, "\0");
            double* points_array = (double *)calloc((space_count+1), sizeof(double));
            fgets(path, line_length, f_r);
            path[line_length-1] = '\0';
            int current_coord = 0;
            char* temp_token = strtok(path, "\t");
            char* temp_path = strstr(path," ");
            int iters = 0;
            while(iters<space_count){
                val = atof(temp_token);//1000.0;
                points_array[current_coord] = val;
                printf("%f\t", points_array[current_coord]);
                current_coord++;
                temp_token = strtok(temp_path+1, "\t");
                printf("%s THIS IS TEMP TOK\n" , temp_token);
            	temp_path = strstr(temp_path+1," ");
                printf("%s THIS IS TEMP PATH\n" , temp_path);
                iters++;
            }
            printf("%f\n", points_array[current_coord]);
            points_array[current_coord] = atof(temp_token);
            printf("%f\n", points_array[current_coord]);
            printf("%s IM HERE AFTER TOK\n", temp_token);
            points_len = current_coord;
            return points_array;
        }else{
            for(;;){
                counter_to_path_start += 1;
                c = fgetc(f_r);
                if( c == EOF || c == '\n' )
                    break;
            }
            path_number+=1;
        }
        if(c==EOF){
        	break;
        }
    }
    return NULL;
}

/*Updates the stop point, should be called everytime the orientation of the bot starts to change*/
void update_stopPoint(){
    stop_point[0] = current_point[0];
    stop_point[1] = current_point[1];
}

/*Reset the PID variables*/
void init_pid(){
    current_error = 0;
    accumulated_error = 0;
    prev_error = 0;
    pid_flag = 0;
}

/*Calculates the parameters for travelling to from point to next point in path(this is assuming it has reached its previous point correctly)*/
void update_points(){
    dist_required = sqrt(pow(points[point_index*2+1] - prev_point[1],2)+pow(points[point_index*2] - prev_point[0],2));
    if (points[point_index*2]-prev_point[0] >= 0){
        angle_required = atan((points[point_index*2+1]-prev_point[1])/(points[point_index*2]-prev_point[0]))*180/M_PI;
    }
    else{
        angle_required = -atan((points[point_index*2]-prev_point[0])/(points[point_index*2+1]-prev_point[1]))*180/M_PI;
        if(points[point_index*2+1]-prev_point[1] >= 0)
            angle_required = angle_required + 90;
        else
            angle_required = angle_required - 90;
    }
    prev_point[0] = points[point_index*2];
    prev_point[1] = points[point_index*2+1];
    point_index  = point_index + 1;
    if(point_index >= (points_len+1)/2){
        point_index = (points_len+1)/2-1;
        return;
    }
    dist_traversed = 0;
    init_pid();
}

/*PID Controller for rotating to a specific angle*/
double get_PID_angle(double current_val, double target_val){
    current_error = target_val - current_val;
    accumulated_error = accumulated_error + current_error*timeDiff;
    double val = 0;
    if(timeDiff != 0)
        val = 0.1*current_error + 0.0005*accumulated_error + 0.001*(current_error-prev_error)/timeDiff;
    else if(timeDiff==0)
        val = 0.1*current_error + 0.0005*accumulated_error;
    if(val>5)
        val = 5;
    else if(val<-5)
        val = -5;
    printf("VALROT: %.17g", val );
    if(val<0.005&&val>-0.005){
        pid_flag = 1;
    }
    prev_error = current_error;
    return val;
}

/*PID Controller for moving along a straight line*/
double get_PID_dist(double current_val, double target_val){
    current_error = target_val-current_val;
    accumulated_error = accumulated_error + current_error*timeDiff; 
    double val = 500*current_error + 0.1*accumulated_error;
    if(val > 5)
        val = 5;
    else if(val < -5)
        val = -5;
    if (val < 0.1&&val>-0.1)
        pid_flag = 1;
    prev_error = current_error;
    return val;
}

/*When no obstacle is present*/
void normal_motion(){
    if(rotating_flag==1){
        double val = get_PID_angle(angle_rotated, angle_required);
        if(pid_flag==1){
            init_pid();
            stop();
            rotating_flag = 0;
        }else{
            rotate(val);
        }
    }else{
        double val = get_PID_dist(dist_traversed, dist_required);
        printf("VAL: %.17g\n", val);
        if(pid_flag==1){
            init_pid();
            stop();
            update_points();
            update_stopPoint();
            rotating_flag = 1;
        }else{
            forward(val);
        }
    }
}

/*Recalculates the distance to be travelled and the angle to be rotated to reach its destination from its current position*/
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



struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};

/*The following are dummy functions for movement of the bot*/
void move_forward()
{
    ledc_set_duty(led_channel.speed_mode, led_channel.channel, 8192);   //Update the PWM value to be used by this lED Channel.
    ledc_update_duty(led_channel.speed_mode, led_channel.channel);      //Use the Updated PWM values
}                                                                       //Similar functions below with different duty cycles

void move_left()
{
    ledc_set_duty(led_channel.speed_mode, led_channel.channel, 4096);
    ledc_update_duty(led_channel.speed_mode, led_channel.channel);
}

void move_right()
{
    ledc_set_duty(led_channel.speed_mode, led_channel.channel, 2048);
    ledc_update_duty(led_channel.speed_mode, led_channel.channel);
}

void move_back()
{
    ledc_set_duty(led_channel.speed_mode, led_channel.channel, 1024);
    ledc_update_duty(led_channel.speed_mode, led_channel.channel);
}

void move_stop()
{
    ledc_set_duty(led_channel.speed_mode, led_channel.channel, 0);
    ledc_update_duty(led_channel.speed_mode, led_channel.channel);
}

/*Replaces the nth line in the file /wifi_conf.txt with the line supplied in the argument
  The 1st line contains "SAP" or "STA" to denote which connection mode to use
  Each of the WIFI_NUM lines after that stores a Network Credential and is of the format "[ssid] [password]\n"
  The next line contains which Network is to be used (0 if SAP mode)
  the next line contains the total number of Valid Network Credentials
  Note: argument *line should not end with \n because it is added seperately*/
esp_err_t replace_wifi(char* line, int n)
{
    char str[LINE_LEN];
    int linectr = 0;
    FILE* f_r = fopen("/spiffs/wifi_conf.txt", "r");	//open the file for reading
    FILE* f_w = fopen("/spiffs/temp.txt", "w");			//open a temporary file for writing which will replace the original file
    if(f_r == NULL){									//check if file open failed
        printf("Error opening file wifi_conf.txt\n");
        return ESP_FAIL;
    }
    if(f_w == NULL){									//check if file open failed
        printf("Error opening file temp.txt\n");
        return ESP_FAIL;
    }
    while(!feof(f_r))
    {
        strcpy(str, "\0");								//intialize to empty string
        fgets(str, LINE_LEN, f_r);						//get a single line from the file
        if(!feof(f_r))
        {
            linectr++;									//calculate which line it is currently on
            if(linectr == n){							//if matches
                fprintf(f_w, "%s", line);				//write the line passed in this function's argument
                fprintf(f_w, "%s", "\n");
            }
            else 										//else
                fprintf(f_w, "%s", str);				//write the line that was extracted from the file
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/wifi_conf.txt");
    rename("/spiffs/temp.txt", "/spiffs/wifi_conf.txt");
    return ESP_OK;  
}

/*Update the total number of Valid Paths by n (n can be positive or negative)*/
esp_err_t update_number(int n){
    char line[2];
    char str[LINE_LEN];
    int linectr = 0;
    total_paths = total_paths + n;          //Update the total_paths global variable
    sprintf(line, "%d", total_paths);		//convert total_paths from an integer to a string and store it on 'line' variable
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    FILE* f_w = fopen("/spiffs/temp.txt", "w");
    if(f_r == NULL){
        printf("Error opening file paths.txt\n");
        return ESP_FAIL;
    }
    if(f_w == NULL){
        printf("Error opening file temp.txt\n");
        return ESP_FAIL;
    }
    while(!feof(f_r))
    {
        strcpy(str, "\0");					//initialize to null string
        fgets(str, LINE_LEN, f_r);
        if(!feof(f_r))
        {
            linectr++;
            if(linectr == 1){				//since it is stored in the 1st line
                fprintf(f_w, "%s", line);   //Update the value stored in the file by writing the 'line' variable that contains the string representaion of total_paths
                fprintf(f_w, "%s", "\n");
            }
            else
                fprintf(f_w, "%s", str);	//else write the string that was extracted from the file
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;
}

/*Used for "deleting" network credentials. All network credentials below it are moved up and the last line is replaced by "example example\n"*/
esp_err_t delete(int n)
{
    char str[LINE_LEN];
    int linectr = 0;
    FILE* f_r = fopen("/spiffs/wifi_conf.txt", "r");
    FILE* f_w = fopen("/spiffs/temp.txt", "w");
    if(f_r == NULL){
        ESP_LOGE(TAG, "Error opening file wifi_conf.txt\n");
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
        if(!feof(f_r))
        {
            linectr++;
            if(linectr == n){
                if(linectr == WIFI_NUM+1)
                    fprintf(f_w, "%s", "example example\n");
                continue;
            }
            else if(linectr == WIFI_NUM+1){
                fprintf(f_w, "%s", str);
                fprintf(f_w, "%s", "example example\n");
            }
            else
                fprintf(f_w, "%s", str);
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/wifi_conf.txt");
    rename("/spiffs/temp.txt", "/spiffs/wifi_conf.txt");
    return ESP_OK;  
}


/*Deletes the nth path from paths.txt*/
esp_err_t delete_specific_path(int n)
{
    char str[LINE_LEN];
    int linectr = 0;
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
        if(!feof(f_r))
        {
            linectr++;
            if(linectr == (n+1)){           //Since the 1st line contains the number of valid paths, so the nth path will be on the (n+1)th Line
                continue;
            }
            else
                fprintf(f_w, "%s", str);
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;  
}


/*Algorithm for converting path to co-ordinate based format
  Remove DEFAULT_LIN_SPEED and DEFAULT_ANG_SPEED if the stored format is in distance and not time*/
esp_err_t convert_paths(int n){
    char str[LINE_LEN];
    char str1[LINE_LEN];
    int len = 0, linectr = 0;
    char ch, temp[9];
    double val, counter;
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
                fseek(f_r, -strlen(str), SEEK_CUR);
                fgets(str1, LINE_LEN, f_r);
                char* temp_token = strtok(str1, "\t");
                while(temp_token!=NULL)
                {
                    ch = temp_token[0];
                    temp_token++;
                    val = DEFAULT_LIN_SPEED * atof(temp_token)/1000.0;
                    if(ch == 'f' || ch == 'b')
                        len += ceil(val/resolution);
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
                        counter = resolution;
                        while(counter <= val){
                            current.x = prev.x + counter*cos(prev.theta*3.14/180.0);
                            current.y = prev.y + counter*sin(prev.theta*3.14/180.0);
                            ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                            counter = counter + resolution;
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.x);
                            strcat(result, temp);
                            strcat(result, " ");
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.y);
                            strcat(result, temp);
                            strcat(result, " ");
                        }
                        current.theta = prev.theta;
                        prev = current;
                    }
                    else if(ch == 'b'){
                        val = DEFAULT_LIN_SPEED * val/1000.0;
                        counter = resolution;
                        while(counter <= val){
                            current.x = prev.x - counter*cos(prev.theta*3.14/180.0);
                            current.y = prev.y - counter*sin(prev.theta*3.14/180.0);
                            ESP_LOGI(TAG, "(%f, %f)", current.x, current.y);
                            counter = counter + resolution;
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.x);
                            strcat(result, temp);
                            strcat(result, " ");
                            strcpy(temp, "");
                            snprintf(temp, 9, "%f", current.y);
                            strcat(result, temp);
                            strcat(result, " ");
                        }
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

/*Keeps all the lines in paths.txt till the nth line and deletes all the lines after that*/
esp_err_t delete_paths(int n){
    char str[LINE_LEN];
    int linectr = 0;
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
        strcpy(str, "\0");              //Used for intializing str to NULL before extracting each line
        fgets(str, LINE_LEN, f_r);
        if(!feof(f_r))
        {
            linectr++;
            if(linectr <= n)
                fprintf(f_w, "%s", str);
            else
                break;
        }
    }
    fclose(f_r);
    fclose(f_w);
    remove("/spiffs/paths.txt");
    rename("/spiffs/temp.txt", "/spiffs/paths.txt");
    return ESP_OK;
}

/*Initialize SPIFFS file system so that the stored files can be accessed. Needs to be called on every reboot. Standard Code gotten from Official Github Repo*/
void init_spiffs()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = BASE_PATH,
        .partition_label = NULL,
        .max_files = MAX_FILES,
        .format_if_mount_failed = RESET_FLAG
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}

/* This function updates the global variables used in the program by reading the wifi_conf.txt and the paths.txt file */
int main_update()
{
    int i, conn_flag_local = 0; //conn_flag_local is initially set to 0 for SAP mode
    struct stat st;
    struct stat st1;
    char line[LINE_LEN];
    char* pos;
    ESP_LOGI(TAG, "Checking connection type");
    if (stat("/spiffs/wifi_conf.txt", &st) != 0) {              //This checks if wifi_conf.txt is absent. This is done for the 1st time ESP is booted.
        FILE* f = fopen("/spiffs/wifi_conf.txt", "w");          //If it is absent, then create wifi_conf.txt in write mode
        if (f == NULL) {                                        //Return 0 if failed to open for any reason
            ESP_LOGE(TAG, "Failed to open file for writing");
            return 0;                                           //changed flag to 0
        }
        fprintf(f, "SAP\n");                                    //Initialize the first line with 'SAP' because SAP mode is the default mode of operation
        for(i = 0; i < WIFI_NUM; i++)                           //Initialize the next WIFI_NUM lines with dummy data that will be overwritten later when user enters a wifi credential
            fprintf(f, "example example\n");
        fprintf(f, "0\n0\n");                                   //The 1st zero = which wifi credential is used (0 because SAP mode is used), the 2nd Zero = total number of valid wifi credentials
        fclose(f);
        ESP_LOGI(TAG, "File written");
    }
    else{                                                       //This means file is present
        FILE* f = fopen("/spiffs/wifi_conf.txt", "r");          //Return 0 if failed to open for any reason
        if(f == NULL){
            printf("Failed to open file\n");
            return 0;
        }
        fgets(line, sizeof(line), f);                           //Get the 1st line in a string variable
        pos = strchr(line, '\n');
        if(pos){
            *pos = '\0';                                        //Put the terminating character at the appropriate position
        }
        if(strcmp(line, "SAP") == 0){                           //Check if it is in SAP mode
            char* token;
            for(i = 0; i <= WIFI_NUM+1; i++){
                fgets(line, sizeof(line), f);                   //Get the next line
                pos = strchr(line, '\n');
                if(pos){
                *pos = '\0';                                    //Put the terminating character at the appropriate position
                }
                if(i == WIFI_NUM)                               //Since it is in SAP mode we dont need to know which STA network is to be used
                    continue;
                if(i == WIFI_NUM+1){
                    total = atoi(line);                         //Update the total number of valid saved networks
                    continue;
                }
                ESP_LOGI(TAG, "%s", line);                      
                token = strtok(line, " ");                      //Update the SSID data matrix whether its dummy or valid
                strcpy(ssid[i], token);
                token = strtok(NULL, " ");                      //Update the password data matrix whether its dummy or valid
                strcpy(pass[i], token);
            }
            ESP_LOGI(TAG, "SAP Mode");
            for(i = 0; i < WIFI_NUM; i++){                      //This is for debugging purposes to check whether all the network credentials are extracted successfully or not
                ESP_LOGI(TAG, "%dth Network SSID: %s\n", (i+1), ssid[i]);
                ESP_LOGI(TAG, "%dth Network Password: %s\n", (i+1), pass[i]);
            }
            fclose(f);
        }
        else{                                                   //This means it is in STA mode
            char* token;
            for(i = 0; i <= WIFI_NUM+1; i++){
                fgets(line, sizeof(line), f);                   //Get the next line
                pos = strchr(line, '\n');
                if(pos){
                *pos = '\0';                                    //Put the terminating character at the appropriate position
                }
                if(i == WIFI_NUM){                              //Which STA network to connect to is stored in the WIFI_NUM+1 th line
                    conn_flag_local = atoi(line);               //Get which network to connect to (1 to WIFI_NUM)
                    continue;
                }
                if(i == WIFI_NUM+1){                            //Total number of valid saved networks is stores in the WIFI_NUM+2 th line
                    total = atoi(line);                         //Get total number of valid saved networks
                    continue;
                }
                token = strtok(line, " ");                      //Update the SSID data matrix whether its dummy or valid
                strcpy(ssid[i], token);
                token = strtok(NULL, " ");                      //Update the password data matrix whether its dummy or valid
                strcpy(pass[i], token);
            }
            ESP_LOGI(TAG, "STA Mode");
            for(i = 0; i < WIFI_NUM; i++){
                if((i+1) == conn_flag_local)                    //(i+1) is done because i is from 0:(WIFI_NUM-1) while conn_flag_local is from 1:WIFI_NUM
                {
                    strcpy(EXAMPLE_ESP_WIFI_SSID, ssid[i]);     //Get the valid network SSID to be used for connecting
                    strcpy(EXAMPLE_ESP_WIFI_PASS, pass[i]);     //Get the valid network Password to be used for connecting
                }
                ESP_LOGI(TAG, "%dth Network SSID: %s\n", (i+1), ssid[i]);
                ESP_LOGI(TAG, "%dth Network Password: %s\n", (i+1), pass[i]);
            }
            fclose(f);
        }
    }
    if (stat("/spiffs/paths.txt", &st1) != 0){                  //Check if paths.txt is absent. This is for when the ESP is booted for the 1st time
        FILE* f = fopen("/spiffs/paths.txt", "w");              //Create paths.txt in writing mode
        if(f == NULL)
            ESP_LOGE(TAG, "Failed to open paths.txt for writing");
        fprintf(f, "0\n");                                      //Put 0 because there are 0 valid paths stored till now
        fclose(f);
        total_paths = 0;                                        //Update the total number of valid paths
    }
    else{
        FILE* f = fopen("/spiffs/paths.txt", "r");              //This means the file is present. Open it in reading mode
        if(f == NULL)
            ESP_LOGE(TAG, "Failed to open path.txt for reading");
        pos = strchr(line, '\n');                               //Get the 1st line
        if(pos){
            *pos = '\0';                                        //Put the terminating character in the appropriate place                                         
        }
        total_paths = atoi(line);                               //Convert from string to int and Update the total number of valid paths
        fclose(f);
    }
    return conn_flag_local;                                     //Return which wifi network to be used (0 for SAP, 1:WIFI_NUM for STA)
}

/*Used for updating the SSID and PASS data matrices only with the values stored in wifi_conf.txt whenever requried so that main_update doesnt need to be called everytime*/
esp_err_t update_wifi()
{
    int i;
    char line[LINE_LEN], *pos;
    char* token;
    FILE* f = fopen("/spiffs/wifi_conf.txt", "r");
    if(f == NULL){
        ESP_LOGE(TAG, "Failed to open file");
        return ESP_FAIL;
    }
    fgets(line, sizeof(line), f);
    for(i = 0; i < WIFI_NUM; i++)
    {
        fgets(line, sizeof(line), f);
        pos = strchr(line, '\n');
        if(pos){
            *pos = '\0';
        }
        token = strtok(line, " ");
        strcpy(ssid[i], token);
        token = strtok(NULL, " ");
        strcpy(pass[i], token);
    }
    return ESP_OK;
}

/*Used for updating the total_paths variable only with the value stored in paths.txt whenever required so that the main_update doesnt need to be called everytime*/
esp_err_t update_paths()
{
    char line[LINE_LEN], *pos;
    FILE* f = fopen("/spiffs/paths.txt", "r");
    if(f == NULL){
        ESP_LOGE(TAG, "Failed to open path.txt");
        return ESP_FAIL;
    }
    fgets(line, sizeof(line), f);	//It is in the first line only so we dont need to go to other lines
    pos = strchr(line, '\n');
    if(pos){
        *pos = '\0';
    }
    total_paths = atoi(line);	//Convert the extracted string to a number
    fclose(f);
    return ESP_OK;
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

/*HTML Code for displaying the home page "/" */
char* default_page()
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)  //For checking which mode the ESP is currently operating in
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");
    strcat(ptr, "<p>Press to select Manual mode</p><a class=\"button button-on\" href=\"/manual\">MANUAL</a>\n");//On clicking go to "/manual"
    strcat(ptr, "<p>Press to select Auto mode</p><a class=\"button button-on\" href=\"/auto\">AUTO</a>\n");//On clicking go to "/auto"
    strcat(ptr, "<p>Press to choose Connection Mode</p><a class=\"button button-on\" href=\"/choose\">SAP/STA</a>\n");//On clicking go to "/choose"
    strcat(ptr, "<p>Press to reset the ESP</p><a class=\"button button-on\" href=\"/reset\">FACTORY\nRESET</a>\n");//On clicking go to "/reset"

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

/*HTML Code for displaying "/choose" which has options for choosing between SAP and STA Mode*/
char* choose_page()
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0) //For checking which mode the ESP is currently operating in
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<p>Press to select SAP mode</p><a class=\"button button-on\" href=\"/sap\">SAP</a>\n");//On clicking go to "/sap"
    strcat(ptr, "<p>Press to select STA mode</p><a class=\"button button-on\" href=\"/sta\">STA</a>\n");//On clicking go to "/sta"
    strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");//On clicking go back to "/"

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

/*HTML Code for displaing "/sta"*/
char* get_sta()
{
    int i = 0;
    char str[2];
    char* ptr = (char*)calloc(2048, sizeof(char));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)  //For checking which mode ESP is currently operating in
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<p>Press to store New network credentials</p><a class=\"button button-on\" href=\"/new\">NEW</a>\n");//On clicking go to "/new"
    for(i = 0; i < total; i++) //Since we have 'total' number of valid network credentials so display 'total' options
    {
        sprintf(str, "%d", i+1);//convert (i+1) to string
        strcat(ptr, "<p>Press for more options</p><a class=\"button button-on\" href=\"/sta");
        strcat(ptr, str); //On clicking go to "/sta1" or "/sta2" or so on dependingon value of (i+1)
        strcat(ptr, "\">Network ");
        strcat(ptr, str); //For displaying "Network 1" or "Network 2" and so on
        strcat(ptr, "</a>\n");
    }
    strcat(ptr, "<p>Press to go back</p><a class=\"button button-on\" href=\"/choose\">BACK</a>\n");
    return ptr;
}

/*HTML Code for /sta1, /sta2, /sta3, /sta4, /sta5 depending on value of local_flag value*/
char* get_sta_data(int local_flag) //local_flag min value is 1
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 120px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)  //Check which mode ESP is currently operating in
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<h3>Network SSID: ");
    strcat(ptr, ssid[local_flag-1]);        //Display the appropriate Network Name
    strcat(ptr, "</h3>\n");
    char str[2];
    sprintf(str, "%d", local_flag);         //Convert value of local_flag to string
    strcat(ptr, "<p>Press to modify</p><a class=\"button button-on\" href=\"/sta_mod_");
    strcat(ptr, str);
    strcat(ptr, "\">MODIFY</a>\n");
    strcat(ptr, "<p>Press to delete</p><a class=\"button button-on\" href=\"/sta_delete_");
    strcat(ptr, str);
    strcat(ptr, "\">DELETE</a>\n");
    strcat(ptr, "<p>Press to connect</p><a class=\"button button-on\" href=\"/sta_choose_");
    strcat(ptr, str);
    strcat(ptr, "\">CONNECT</a>\n");
    strcat(ptr, "<p>Press to go back</p><a class=\"button button-on\" href=\"/sta\">BACK</a>\n");
    return ptr;
}

/*Get the HTML form for submitting Network Credential Data*/
char* get_form(int local_flag)  //local_flag min value is 1.
{
    char str[2];
    sprintf(str, "%d", local_flag);
    char* ptr = (char *)calloc(2048, sizeof(char));
    strcat(ptr, "<form action=\"/data_"); //On submitting form, go to "/data_1" or "/data_2" or "/data_3" or "/data_4" or "/data_5" according to local_flag value
    strcat(ptr, str);
    strcat(ptr, "\" method = \"post\">\n");
    strcat(ptr, "<label for=\"ssid\">SSID:</label><br>\n");
    strcat(ptr, "<input type=\"text\" id=\"ssid\" name=\"ssid\" maxlength=SSID_LEN><br>\n");
    strcat(ptr, "<label for=\"pwd\">Password:</label><br>\n");
    strcat(ptr, "<input type=\"password\" id=\"pwd\" name=\"pwd\"maxlength=PASS_LEN><br><br>\n");
    strcat(ptr, "<input type=\"submit\" value=\"Submit\">\n");
    strcat(ptr, "<input type=\"reset\">\n");
    strcat(ptr, "</form>");
    return ptr;
}

/*HTML code for showing "/auto" page*/
char* get_auto()
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    int i;
    char str[20];
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else{
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");
    }
    for(i = 1; i <= total_paths; i++) //Since we have "total_paths" number of valid paths
    {
        sprintf(str, "%d", i);
        strcat(ptr, "<p>Press for more details</p><a class=\"button button-on\" href=\"/path_details");
        strcat(ptr, str); //Go to "/path_details1" or "/path_details2" and so on
        strcat(ptr, "\">Path ");
        strcat(ptr, str); //Display "Path 1 " or "Path 2 " and so on
        strcat(ptr, "</a>\n");
    }    
    strcat(ptr, "<p>Press to return to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

/*HTML Code for /path_details1, /path_details2, /path_details3, /path_details4, /path_details5 depending on value of local_flag value*/
char* get_path_specific(int local_flag)
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    char str[20];
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else{
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");
    }
    sprintf(str, "%d", local_flag);
    strcat(ptr, "<h3>PATH: ");
    strcat(ptr, str);
    strcat(ptr, "</h3>\n");
    strcat(ptr, "<p>Press to execute this path</p><a class=\"button button-on\" href=\"/path");
    strcat(ptr, str); //Go to "/path1" or "/path2" and so on
    strcat(ptr, "\">Execute</a>\n");
    strcat(ptr, "<p>Press to delete this path</p><a class=\"button button-on\" href=\"/delete_path");
    strcat(ptr, str); //Go to "/delete_path1" or "/delete_path2" and so on
    strcat(ptr, "\">Delete</a>\n");
    strcat(ptr, "<p>Press to return to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

/*Execute the local_flag th path*/
esp_err_t get_path(int local_flag)
{
    char str[LINE_LEN];
    int linectr = 0;
    FILE* f_r = fopen("/spiffs/paths.txt", "r");
    if (f_r == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    while(!feof(f_r))
    {
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);
        if(!feof(f_r))
        {
            linectr++;
            if(linectr == (local_flag+1)) //1st line contains the number of valid paths, so nth path will be on (n+1)th line
                break;
        }
    }
    ESP_LOGI(TAG, "%s", str);
    fclose(f_r);
    //return ESP_OK;
    char* token = strtok(str, "\t");    //The elements are seperated by "\t"
    auto_flag = 1;                      
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
        token = strtok(NULL, "\t");             //Get the next element
    }
    //move_stop();
    auto_flag = 0;
    return ESP_OK;
}

/*HTML Code which displays different text depending on local_flag and contains only a single button for returning to home page "/"*/
char* get_home(int local_flag)
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    if(local_flag == 1)
        strcat(ptr, "<h3>Maximum number of connections reached</h3>\n");
    else if(local_flag == 2)
        strcat(ptr, "<h3>Deleted Successfully</h3>\n");
    else if(local_flag == 3)
        strcat(ptr, "<h3>Saved Successfully</h3>\n");
    else if(local_flag == 4)
        strcat(ptr, "<h3>Added Successfully</h3>\n");
    strcat(ptr, "<p>Press to return to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr; 
}

/*HTML Code for displaying "/manual" or "/pause" page (same for both)*/
char* manual_mode()
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    char str[20];
    sprintf(str, "%d", (total_paths+1));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    if(total_paths == PATH_NUM){ //Check if maximum number of valid paths have been reached
        strcat(ptr, "<h3>Maximum number of stored paths reached. Please delete saved paths to store new paths</h3>\n");
        strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
        strcat(ptr, "</body>\n");
        strcat(ptr, "</html>\n");
        return ptr;
    }

    if(record_flag == 0){ //Check if path recording has started
        strcat(ptr, "<h3>Path recording not yet started</h3>\n");
        strcat(ptr, "<p>Click to Start</p><a class=\"button button-on\" href=\"/start\">START</a>\n"); //Button for staring since path recording has not yet started
    }
    else{
        strcat(ptr, "<h3>Now recording Path: ");
        strcat(ptr, str);
        strcat(ptr, "</h3>\n");
        strcat(ptr, "<p>Click to Stop</p><a class=\"button button-on\" href=\"/stop\">STOP</a>\n");//Button for stopping the path recording
    }
    strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");
    strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");
    strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");
    strcat(ptr, "<p>Back: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n");
    if(record_flag == 0) //Option to go to home page "/" is disable while recording is going on
        strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
/*    strcat(ptr, "<html><head></head><style>");
    strcat(ptr, "body {background-color: lightyellow}");
    strcat(ptr, "h1 {color:blue}");
    strcat(ptr, "p {color: blue}");
    strcat(ptr, "button {color: blue;background:lightblue;border: 1px solid #000;border-radius: 8px;position: center;}");
    strcat(ptr, "</style><body>");
    strcat(ptr, "<div style=\"text-align:center\">");
    strcat(ptr, "<h1>Direction Controller</h1><br><br>");
    strcat(ptr, "<button style=\"height: 50px; width: 100px; font-size: 18px\">Start</button>");
    strcat(ptr, "<img hspace=\"20\" style=\"padding-left: 200px\">");
    strcat(ptr, "<button style=\"height: 50px; width: 100px; font-size: 18px\">Stop</button><br><br>");
    strcat(ptr, "<span style=\"display:inline-block;padding:5px;border:1px solid #ff0000; font-size: 140%;font-weight:bold;\">");
    strcat(ptr, "<br><button style=\"height: 70px; width: 80px; font-size: 18px\">Forward</button><br><br><br><br>");
    strcat(ptr, "<img hspace=\"10\" style=\"padding-left: 5px\">");
    strcat(ptr, "<button style=\"height: 70px; width: 80px; font-size: 18px\">Left</button>");
    strcat(ptr, "<img hspace=\"20\" style=\"padding-left: 10px\">");
    strcat(ptr, "<button style=\"height: 70px; width: 80px; font-size: 18px\">Stop</button>");
    strcat(ptr, "<img hspace=\"20\" style=\"padding-left: 10px\">");
    strcat(ptr, "<button style=\"height: 70px; width: 80px; font-size: 18px\">Right</button>");
    strcat(ptr, "<img hspace=\"10\" style=\"padding-left: 5px\"><br><br><br><br>");
    strcat(ptr, "<button style=\"height: 70px; width: 80px; font-size: 18px\">Back</button><br><br><br>");
    strcat(ptr, "<p>Additional Options</p>");
    strcat(ptr, "<img hspace=\"10\" style=\"padding-left: 5px\">");
    strcat(ptr, "<button style=\"height: 50px; width: 100px; font-size: 18px\">HOME</button>");
    strcat(ptr, "<img hspace=\"10\" style=\"padding-left: 5px\">");
    strcat(ptr, "<br><br></span></div></body></html>");*/
    return ptr;
}

/*HTML Code for displaying "/forward" or "/left" or "/right" or "/back" depending on local_flag*/
char* SendHTML(uint8_t local_flag)
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    char str[20];
    sprintf(str, "%d", (total_paths+1));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    if(record_flag == 0)
        strcat(ptr, "<p>Click to Start</p><a class=\"button button-on\" href=\"/start\">START</a>\n");
    else{
        strcat(ptr, "<h3>Now recording Path: ");
        strcat(ptr, str);
        strcat(ptr, "</h3>\n");
        strcat(ptr, "<p>Click to Stop</p><a class=\"button button-on\" href=\"/stop\">STOP</a>\n");
    }

    if(local_flag==0)
    {strcat(ptr, "<p>Forward: ON (Press to Pause)</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");}

    if(local_flag==1)
    {strcat(ptr, "<p>Left: ON (Press to Pause)</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");}

    if(local_flag==2)
    {strcat(ptr, "<p>Right: ON (Press to Pause)</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");}

    if(local_flag==3)
    {strcat(ptr, "<p>Backwards: ON (Press to Pause)</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Backwards: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n");}

    if(record_flag == 0)
        strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

/*HTML Code for displaying "/stop"*/
char* get_stop()
{
    char* ptr = (char*)calloc(2048, sizeof(char));
    strcat(ptr, "<!DOCTYPE html> <html>\n");
    strcat(ptr, "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");
    strcat(ptr, "<title>Choose Direction</title>\n");
    strcat(ptr, "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");
    strcat(ptr, "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");
    strcat(ptr, ".button {display: block;width: 100px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
    strcat(ptr, ".button-on {background-color: #3498db;}\n");
    strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
    strcat(ptr, ".button-off {background-color: #34495e;}\n");
    strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
    strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
    strcat(ptr, "</style>\n");
    strcat(ptr, "</head>\n");
    strcat(ptr, "<body>\n");
    strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<p>Press to discard and store new path</p><a class=\"button button-on\" href=\"/manual\">NEW</a>\n");
    strcat(ptr, "<p>Press to save</p><a class=\"button button-on\" href=\"/save\">SAVE</a>\n");

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

/*In all of the callback functions below, the HTML Code for displaying
  the webpage is passed using the 'resp' string variable*/

/*Callback function whenever "/" is accessed*/
esp_err_t handle_OnConnect(httpd_req_t *req)
{
    flag = -1;
    char* resp = default_page(); //Get the HTML Code
    httpd_resp_send(req, resp, strlen(resp));  //Send the HTML Code to display
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /");
    ESP_LOGI(TAG, "Callback Function called: handle_OnConnect()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: default_page()");
    return ESP_OK;
}

/*Callback function whenever "/reset" is accessed*/
esp_err_t handle_reset(httpd_req_t *req)
{
    remove("/spiffs/wifi_conf.txt"); //delete the files for fresh reboot
    remove("/spiffs/paths.txt");
    httpd_resp_send(req, "Device will restart now", strlen("Device will restart now"));//just send a line for displaying it in the webpage
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /reset");
    ESP_LOGI(TAG, "Callback Function called: handle_reset()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart(); //restart the ESP
    return ESP_OK;
}

/*Callback function whenever "/start" is accessed*/
esp_err_t handle_start(httpd_req_t *req)
{
    record_flag = 1; //record_flag is changed to 1 to denote that recording has started
    flag = -1;       //-1 denotes stop
/*    FILE* f = fopen("/spiffs/paths.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "File opened for writing");
    fclose(f);*/
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    char* resp = manual_mode();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /start");
    ESP_LOGI(TAG, "Callback Function called: handle_start()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: manual_mode()");
    return ESP_OK;
}

/*Callback function whenever "/path1" is accessed*/
esp_err_t handle_path1(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(1)); //Execute Path 1
    char* resp = get_home(0);   //Get the HTML Code to display
    httpd_resp_send(req, resp, strlen(resp));   //Display the HTML Code
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path1");
    ESP_LOGI(TAG, "Callback Function called: handle_path1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(0)");
    return ESP_OK;
}

/*Callback function whenever "/path2" is accessed*/
esp_err_t handle_path2(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(2)); //Execute Path 2
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
	ESP_LOGI(TAG, "Now displaying /path2");
    ESP_LOGI(TAG, "Callback Function called: handle_path2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(0)");
    return ESP_OK;
}

/*Callback function whenever "/path3" is accessed*/
esp_err_t handle_path3(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(3)); //Execute Path 3
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path3");
    ESP_LOGI(TAG, "Callback Function called: handle_path3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(0)");
    return ESP_OK;
}

/*Callback function whenever "/path4" is accessed*/
esp_err_t handle_path4(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(4)); //Execute Path 4
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path4");
    ESP_LOGI(TAG, "Callback Function called: handle_path4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(0)");
    return ESP_OK;
}

/*Callback function whenever "/path5" is accessed*/
esp_err_t handle_path5(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(5)); //Execute PAth 5
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path5");
    ESP_LOGI(TAG, "Callback Function called: handle_path5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(0)");
    return ESP_OK;
}

/*Callback function whenever "/manual" is accessed*/
esp_err_t handle_manual(httpd_req_t *req)
{
    record_flag = 0; //recording has not yet started
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    char det = determine(flag);	//the mode it was in earlier
    curr_mili = esp_timer_get_time();	//get the current time
    time_duration = (curr_mili - prev_mili)/1000;	//get the time period for which it was in the previous mode
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = -1;
    delete_paths(total_paths+1); //Ensure that the correct number of valid paths is stored in the file, this is done in case the path is not saved in the previous try
    char* resp = manual_mode();
    prev_mili = esp_timer_get_time();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /manual");
    ESP_LOGI(TAG, "Callback Function called: handle_manual()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: manual_mode()");
    return ESP_OK;
}

/*Callback function whenever "/path_details1" is accessed
  All of the handle_specific_path() functions below are similar, only difference is the values passed to the HTML generator function*/
esp_err_t handle_specific_path1(httpd_req_t *req)
{
    char* resp = get_path_specific(1);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path_details1");
    ESP_LOGI(TAG, "Callback Function called: handle_specific_path1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_path_specific(1)");
    return ESP_OK;
}

/*Callback function whenever "/path_details2" is accessed*/
esp_err_t handle_specific_path2(httpd_req_t *req)
{
    char* resp = get_path_specific(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path_details2");
    ESP_LOGI(TAG, "Callback Function called: handle_specific_path2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_path_specific(2)");
    return ESP_OK;
}

/*Callback function whenever "/path_details3" is accessed*/
esp_err_t handle_specific_path3(httpd_req_t *req)
{
    char* resp = get_path_specific(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path_details3");
    ESP_LOGI(TAG, "Callback Function called: handle_specific_path3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_path_specific(3)");
    return ESP_OK;
}

/*Callback function whenever "/path_details4" is accessed*/
esp_err_t handle_specific_path4(httpd_req_t *req)
{
    char* resp = get_path_specific(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path_details4");
    ESP_LOGI(TAG, "Callback Function called: handle_specific_path4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_path_specific(4)");
    return ESP_OK;
}

/*Callback function whenever "/path_details5" is accessed*/
esp_err_t handle_specific_path5(httpd_req_t *req)
{
    char* resp = get_path_specific(5);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /path_details5");
    ESP_LOGI(TAG, "Callback Function called: handle_specific_path5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_path_specific(5)");
    return ESP_OK;
}

/*Callback function whenever "/delete_path1" is accessed
  All of the handle_delete_path() functions below are similar, only difference is the values passed to the functions inside them*/
esp_err_t handle_delete_path1(httpd_req_t *req)
{
    ESP_ERROR_CHECK(delete_specific_path(1)); //delete 1st path from the file paths.txt
    ESP_ERROR_CHECK(update_number(-1)); //total number of valid paths has decreased by 1
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /delete_path1");
    ESP_LOGI(TAG, "Callback Function called: handle_delete_path1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/delete_path2" is accessed*/
esp_err_t handle_delete_path2(httpd_req_t *req)
{
    ESP_ERROR_CHECK(delete_specific_path(2));   //delete 2nd path from the file paths.txt
    ESP_ERROR_CHECK(update_number(-1)); //total number of valid paths has decreased by 1
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /delete_path2");
    ESP_LOGI(TAG, "Callback Function called: handle_delete_path2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/delete_path3" is accessed*/
esp_err_t handle_delete_path3(httpd_req_t *req)
{
    ESP_ERROR_CHECK(delete_specific_path(3));   //delete 3rd path from the file paths.txt
    ESP_ERROR_CHECK(update_number(-1)); //total number of valid paths has decreased by 1
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /delete_path3");
    ESP_LOGI(TAG, "Callback Function called: handle_delete_path3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/delete_path4" is accessed*/
esp_err_t handle_delete_path4(httpd_req_t *req)
{
    ESP_ERROR_CHECK(delete_specific_path(4));   //delete 4th path from the file paths.txt
    ESP_ERROR_CHECK(update_number(-1)); //total number of valid paths has decreased by 1
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /delete_path4");
    ESP_LOGI(TAG, "Callback Function called: handle_delete_path4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/delete_path5" is accessed*/
esp_err_t handle_delete_path5(httpd_req_t *req)
{
    ESP_ERROR_CHECK(delete_specific_path(5));   //delete 5th path from the file paths.txt
    ESP_ERROR_CHECK(update_number(-1)); //total number of valid paths has decreased by 1
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /delete_path5");
    ESP_LOGI(TAG, "Callback Function called: handle_delete_path5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(5)");
    return ESP_OK;
}

/*Callback function whenever "/auto" is accessed*/
esp_err_t handle_auto(httpd_req_t *req)
{
    flag = 4; //denotes auto mode
    char* resp = get_auto();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /auto");
    ESP_LOGI(TAG, "Callback Function called: handle_auto()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_auto()");
    return ESP_OK;
}

/*Callback function whenever "/forward" is accessed*/
esp_err_t handle_forward(httpd_req_t *req)
{
    char det = determine(flag);     //determine the direction it was going earlier
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000;;      //the time for which it was going in the previous direction(in ms)
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 0;                       //denotes that is now going in forward direction
    char* resp = SendHTML(flag);	//the HTML code for displaying the webpage
    httpd_resp_send(req, resp, strlen(resp));	//display the webpage
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)        //if the path is recording
    {
        //move_forward();
        FILE* f = fopen("/spiffs/paths.txt", "a");		//Open the File for writing the value
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);  //for storing the previous direction
        fprintf(f, "%.3f", time_duration);  //for storing the previous time duration
        fputc('\t', f); //seperate from next entry by '\t'
        fclose(f);
    }
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /forward");
    ESP_LOGI(TAG, "Callback Function called: handle_forward()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: SendHTML(%d)",flag);
    return ESP_OK;
}

/*Callback function whenever "/left" is accessed
	handle_left, handle_right, handle_back is very similar, only the update value of flag variable is different*/
esp_err_t handle_left(httpd_req_t *req)
{
    char det = determine(flag);     //determine the direction it was going earlier
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;     //the time for which it was going in the previous direction(in ms)
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 1;                       //denotes that is now going in left direction
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)        //if the path is recording
    {
        //move_left();
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);  //for storing the previous direction
        fprintf(f, "%.3f", time_duration); //for storing the previous time duration
        fputc('\t', f); //seperate from next entry by '\t'
        fclose(f);
    }
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /left");
    ESP_LOGI(TAG, "Callback Function called: handle_left()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: SendHTML(%d)",flag);
    return ESP_OK;
}

/*Callback function whenever "/right" is accessed*/
esp_err_t handle_right(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 2;               //Now going in right direction, everything else is same as previous
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
        //move_right();
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%.3f", time_duration);
        fputc('\t', f);
        fclose(f);
    }
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /right");
    ESP_LOGI(TAG, "Callback Function called: handle_right()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: SendHTML(%d)",flag);
    return ESP_OK;
}

/*Callback function whenever "/back is accessed"*/
esp_err_t handle_back(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 3;       //Now going in back direction, everything else same as forward and left and right
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
        //move_back();
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%.3f", time_duration);
        fputc('\t', f);
        fclose(f);
    }
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /back");
    ESP_LOGI(TAG, "Callback Function called: handle_back()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: SendHTML(%d)",flag);
    return ESP_OK;
}

/*Callback function whenever "/stop is accessed"*/
esp_err_t handle_stop(httpd_req_t *req)
{
    char det = determine(flag);         //Note that total_paths is not updated here, it is only updated in "/save", because if the author chooses to discard this path then "/manual" will automatically delete this path
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    ESP_LOGI(TAG, "Reading values");
    if(record_flag == 1)
    {
        //move_stop();
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%.3f", time_duration);
        fputc('\n', f); //Since this is the last entry, instead of putting a '\t', we put a '\n' so that the next path will be saved in the next line
        //fputc('\0', f);
        fclose(f);
    }
    char* resp = get_stop();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    record_flag = 0;    //stop recording
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /stop");
    ESP_LOGI(TAG, "Callback Function called: handle_stop()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_stop()");
    return ESP_OK;
}

/*Callback function whenever "/pause" is accessed*/
esp_err_t handle_pause(httpd_req_t *req)
{
    char det = determine(flag); //Get which direction it was travelling earlier
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = -1;
    char* resp = manual_mode();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1) //If recording is ongoing, then store the direction in which it was travelling and the time duration for whcih it had travelled in that direction
    {
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f); //Store the character denoting direction
        fprintf(f, "%.3f", time_duration); //Store the time duration
        fputc('\t', f); //Put a "\t" to seperate the next entry
        fclose(f);
    }
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /pause");
    ESP_LOGI(TAG, "Callback Function called: handle_pause()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: manual_mode()");
    return ESP_OK;  
}

/*Callback function whenever "/save is accessed"*/
esp_err_t handle_save(httpd_req_t *req)
{
    update_number(1); //total_paths is updated in paths.txt and the global variable is also updated
    ESP_ERROR_CHECK(convert_paths(total_paths+1));  //convert the saved path into co-ordinate based representation, you can comment this part out
    char* resp = get_home(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /save");
    ESP_LOGI(TAG, "Callback Function called: handle_save()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(3)");
    return ESP_OK;
}

/*Callback function whenever "/choose is accessed"*/
esp_err_t handle_choose(httpd_req_t *req)
{
    char* resp = choose_page();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /choose");
    ESP_LOGI(TAG, "Callback Function called: handle_choose()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: choose_page()");
    return ESP_OK;
}

/*Callback function whenever "/sap is accessed"*/
esp_err_t handle_sap(httpd_req_t *req)
{
    httpd_resp_send(req, "Device will restart using SAP mode", strlen("Device will restart using SAP mode"));
    ESP_ERROR_CHECK(replace_wifi("SAP", 1));    //The first line is rewritten with SAP so that on next reboot it starts in SAP mode
    ESP_LOGI(TAG, "SAP Data Written");
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sap");
    ESP_LOGI(TAG, "Callback Function called: handle_sap()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

/*Callback function whenever "/sta is accessed"*/
esp_err_t handle_sta(httpd_req_t *req)
{
    char* resp = get_sta();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta");
    ESP_LOGI(TAG, "Callback Function called: handle_sta()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_sta()");
    return ESP_OK;
}

/*Callback function whenever "/new is accessed"*/
esp_err_t handle_new(httpd_req_t *req)
{
    if(total == WIFI_NUM)   //If maximum number of wifi credentials has been reached
    {
        char* resp = get_home(1);
        httpd_resp_send(req, resp, strlen(resp));
        free(resp);
        ESP_LOGI(TAG, "Now displaying /new");
    	ESP_LOGI(TAG, "Callback Function called: handle_new()");
    	ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(1)");
    }
    else
    {
        total = total + 1;  //total number of valid wifi networks is updated
        char str[20];
        sprintf(str, "%d", total);
        replace_wifi(str, WIFI_NUM+3); //updated value is stored in the file
        char* resp = get_form(total);   //form for getting the SSID and password
        httpd_resp_send(req, resp, strlen(resp));	//show the webpage
        free(resp);
        ESP_LOGI(TAG, "Now displaying /new");
    	ESP_LOGI(TAG, "Callback Function called: handle_new()");
    	ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_form(%d)",total);
    }
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    return ESP_OK;
}

/*Callback function whenever "/sta1 is accessed"*/
esp_err_t handle_sta_data1(httpd_req_t *req)
{
    char* resp = get_sta_data(1);				//get the HTML Code, 1 is passed for Network 1
    httpd_resp_send(req, resp, strlen(resp));	//Display the HTML Code
    free(resp);									//free up the space
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta1");
    ESP_LOGI(TAG, "Callback Function called: handle_sta_data1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_sta_data(1)");
    return ESP_OK;
}

/*Callback function whenever "/sta_mod_1 is accessed"*/
esp_err_t handle_modify1(httpd_req_t *req)
{
    char* resp = get_form(1); //Only form for getting values, total is not updated because the number of valid wifi credentials remian unchanged, 1 is passed as argument because it is for network 1
    httpd_resp_send(req, resp, strlen(resp));	//Display the webpage
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_mod_1");
    ESP_LOGI(TAG, "Callback Function called: handle_modify1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_form(1)");
    return ESP_OK;
}

/*Callback function whenever "/sta_delete_1 is accessed"*/
esp_err_t handle_delete1(httpd_req_t *req)
{
    char* resp = get_home(2);	//Get the HTML Code, this 2 is not related to the Network number	
    httpd_resp_send(req, resp, strlen(resp));	//Display the webpage
    free(resp);
    total = total - 1; //total number of valid wifi credentials has decreased by 1
    char str[20];
    sprintf(str, "%d", total);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+3)); //update the value stored in the file
    ESP_ERROR_CHECK(delete(2));                     //delete the wifi details stored in the file
    ESP_ERROR_CHECK(update_wifi());                 //update the SSID and pass matrix variables
    if(total == 0)									//if total becomes 0, then there are no valid STA networks to connect to
        ESP_ERROR_CHECK(handle_sap(req));			//handle_sap function restarts the ESP in SAP mode and makes the necessary changes to wifi_conf.txt
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_delete_1");
    ESP_LOGI(TAG, "Callback Function called: handle_delete1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_choose_1 is accessed"*/
esp_err_t handle_choose1(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 1); //convert integer 1 to string
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2)); //update which wifi network to use in wifi_conf.txt 
    ESP_ERROR_CHECK(replace_wifi("STA", 1));        //update the 1st line to show it will operate in STA mode
    ESP_LOGI(TAG, "STA Wifi 1");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_choose_1");
    ESP_LOGI(TAG, "Callback Function called: handle_choose1()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart();          //restart esp
    return ESP_OK;
}

/*  The Following functions are same as handle_sta_data1, handle_modify1, handle_delete1, handle_choose1.
    Only the arguments passsed in the functions called within them are different
*/

/*Callback function whenever "/sta2 is accessed"*/
esp_err_t handle_sta_data2(httpd_req_t *req)
{
    char* resp = get_sta_data(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta2");
    ESP_LOGI(TAG, "Callback Function called: handle_sta_data2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_sta_data(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_mod_2 is accessed"*/
esp_err_t handle_modify2(httpd_req_t *req)
{
    char* resp = get_form(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_mod_2");
    ESP_LOGI(TAG, "Callback Function called: handle_modify2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_form(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_delete_2 is accessed"*/
esp_err_t handle_delete2(httpd_req_t *req)
{
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    total = total - 1;
    char str[20];
    sprintf(str, "%d", total);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+3));
    ESP_ERROR_CHECK(delete(3));
    ESP_ERROR_CHECK(update_wifi());
    if(total == 0)
        ESP_ERROR_CHECK(handle_sap(req));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_delete_2");
    ESP_LOGI(TAG, "Callback Function called: handle_delete2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_choose_2 is accessed"*/
esp_err_t handle_choose2(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 2);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 2");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_choose_2");
    ESP_LOGI(TAG, "Callback Function called: handle_choose2()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

/*Callback function whenever "/sta3 is accessed"*/
esp_err_t handle_sta_data3(httpd_req_t *req)
{
    char* resp = get_sta_data(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta3");
    ESP_LOGI(TAG, "Callback Function called: handle_sta_data3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_sta_data(3)");
    return ESP_OK;
}

/*Callback function whenever "/sta_mod_3 is accessed"*/
esp_err_t handle_modify3(httpd_req_t *req)
{
    char* resp = get_form(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_mod_3");
    ESP_LOGI(TAG, "Callback Function called: handle_modify3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_form(3)");
    return ESP_OK;
}

/*Callback function whenever "/sta_delete_3 is accessed"*/
esp_err_t handle_delete3(httpd_req_t *req)
{
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    total = total - 1;
    char str[20];
    sprintf(str, "%d", total);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+3));
    ESP_ERROR_CHECK(delete(4));
    ESP_ERROR_CHECK(update_wifi());
    if(total == 0)
        ESP_ERROR_CHECK(handle_sap(req));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_delete_3");
    ESP_LOGI(TAG, "Callback Function called: handle_delete3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_choose_3 is accessed"*/
esp_err_t handle_choose3(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 3);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA WIFI 3");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_choose_3");
    ESP_LOGI(TAG, "Callback Function called: handle_choose3()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

/*Callback function whenever "/sta4 is accessed"*/
esp_err_t handle_sta_data4(httpd_req_t *req)
{
    char* resp = get_sta_data(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta4");
    ESP_LOGI(TAG, "Callback Function called: handle_sta_data4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_sta_data(4)");
    return ESP_OK;
}

/*Callback function whenever "/sta_mod_4 is accessed"*/
esp_err_t handle_modify4(httpd_req_t *req)
{
    char* resp = get_form(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_mod_4");
    ESP_LOGI(TAG, "Callback Function called: handle_modify4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_form(4)");
    return ESP_OK;
}

/*Callback function whenever "/sta_delete_4 is accessed"*/
esp_err_t handle_delete4(httpd_req_t *req)
{
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    total = total - 1;
    char str[20];
    sprintf(str, "%d", total);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+3));
    ESP_ERROR_CHECK(delete(5));
    ESP_ERROR_CHECK(update_wifi());
    if(total == 0)
        ESP_ERROR_CHECK(handle_sap(req));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_delete_4");
    ESP_LOGI(TAG, "Callback Function called: handle_delete4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_choose_4 is accessed"*/
esp_err_t handle_choose4(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 4);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 4");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_choose_4");
    ESP_LOGI(TAG, "Callback Function called: handle_choose4()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

/*Callback function whenever "/sta5 is accessed"*/
esp_err_t handle_sta_data5(httpd_req_t *req)
{
    char* resp = get_sta_data(5);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta5");
    ESP_LOGI(TAG, "Callback Function called: handle_sta_data5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_sta_data(5)");
    return ESP_OK;
}

/*Callback function whenever "/sta_mod_5 is accessed"*/
esp_err_t handle_modify5(httpd_req_t *req)
{
    char* resp = get_form(5);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_mod_5");
    ESP_LOGI(TAG, "Callback Function called: handle_modify5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_form(5)");
    return ESP_OK;
}

/*Callback function whenever "/sta_delete_5 is accessed"*/
esp_err_t handle_delete5(httpd_req_t *req)
{
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    total = total - 1;
    char str[20];
    sprintf(str, "%d", total);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+3));
    ESP_ERROR_CHECK(delete(6));
    ESP_ERROR_CHECK(update_wifi());
    if(total == 0)
        ESP_ERROR_CHECK(handle_sap(req));
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /sta_delete_5");
    ESP_LOGI(TAG, "Callback Function called: handle_delete5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(2)");
    return ESP_OK;
}

/*Callback function whenever "/sta_choose_5 is accessed"*/
esp_err_t handle_choose5(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 5);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 5");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    ESP_LOGI(TAG, "Now displaying /sta_choose_5");
    ESP_LOGI(TAG, "Callback Function called: handle_choose5()");
    ESP_LOGI(TAG, "HTML Code is not used. Just Displaying a single line on the webpage");
    vTaskDelay(100);
    esp_restart();
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    return ESP_OK;
}

/*Callback function whenever "/data_1" is accessed, i.e, after data has been entered through form for wifi network 1 and needs to be saved"*/
esp_err_t handle_data_1(httpd_req_t *req)
{
    strcpy(line_str, "");   //Initialize it to empty string
    int len = req->content_len;     //Get the length of the header
    int ret, remaining = req->content_len;      //Get how much length is left to be read
    while (remaining > 0) {									//this method of taking the data was taken from online
        if ((ret = httpd_req_recv(req, buf, len)) <= 0) {   //Store len lngth of data from the header in the string variable "buf" 
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[len] = '\0';                        //Put terminating character in proper place
    ESP_LOGI(TAG, "Buffer: %s", buf);       //Display buffer. It will be in the format: "SSID=[ssid]&PASS=[password]"
    strcpy(copy, buf);                      //Copy the contents of buffer into copy variable
    ESP_LOGI(TAG, "Copy: %s", copy);
    char* token = strtok(buf, "&");         //token now contains "SSID=[ssid]"
    char* ssid_data = strtok(token, "=");   //ssid_data now contains "SSID"
    ssid_data = strtok(NULL, "=");          //ssid_data now contains "[ssid]"
    char* new_token = strtok(copy, "&");    //new_token now contains "SSID=[ssid]"
    new_token = strtok(NULL, "&");          //new_token now contains "PASS=[password]"
    char* pwd = strtok(new_token, "=");     //pwd now contains "PASS"
    pwd = strtok(NULL, "=");                //pwd not contains "[password]"
    ESP_LOGI(TAG, "SSID: %s", ssid_data);   //Show the extracted SSID
    ESP_LOGI(TAG, "PASS: %s", pwd);         //Show the extracted Password
    strcat(line_str, ssid_data);            //line_str now contains "[ssid]"
    strcat(line_str, " ");                  //line_str now contains "[ssid] "
    strcat(line_str, pwd);                  //line_str now contains "[ssid] [password]"
    ESP_ERROR_CHECK(replace_wifi(line_str, 2)); //Update the data stored in wifi_conf.txt (1st wifi details are stored in line 2)
    ESP_ERROR_CHECK(update_wifi());             //Update the SSID and PASS matrices
    char* resp = get_home(4);					//Get the HTML code
    httpd_resp_send(req, resp, strlen(resp));	//Display the HTML webpage
    free(resp);									//free up the pointer
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /data_1");
    ESP_LOGI(TAG, "Callback Function called: handle_data_1()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(4)");
    return ESP_OK;
}

/*Callback function whenever "/data_2" is accessed, i.e, after data has been entered through form for wifi network 2 and needs to be saved. The logic is same as the previous function*/
esp_err_t handle_data_2(httpd_req_t *req)
{
    strcpy(line_str, "");
    int len = req->content_len;
    int ret, remaining = req->content_len;
    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, len)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG, "Buffer: %s", buf);
    strcpy(copy, buf);
    ESP_LOGI(TAG, "Copy: %s", copy);
    char* token = strtok(buf, "&");
    char* ssid_data = strtok(token, "=");
    ssid_data = strtok(NULL, "=");
    char* new_token = strtok(copy, "&");
    new_token = strtok(NULL, "&");
    char* pwd = strtok(new_token, "=");
    pwd = strtok(NULL, "=");
    ESP_LOGI(TAG, "SSID: %s", ssid_data);
    ESP_LOGI(TAG, "PASS: %s", pwd);
    strcat(line_str, ssid_data);
    strcat(line_str, " ");
    strcat(line_str, pwd);
    ESP_ERROR_CHECK(replace_wifi(line_str, 3)); //Update the data stored in wifi_conf.txt (2nd wifi details are stored in line 3)
    ESP_ERROR_CHECK(update_wifi());             //Update the SSID and PASS matrices
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //SP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /data_2");
    ESP_LOGI(TAG, "Callback Function called: handle_data_2()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(4)");
    return ESP_OK;
}

/*Callback function whenever "/data_3" is accessed, i.e, after data has been entered through form for wifi network 3 and needs to be saved. The logic is same as the previous function*/
esp_err_t handle_data_3(httpd_req_t *req)
{
    strcpy(line_str, "");
    int len = req->content_len;
    int ret, remaining = req->content_len;
    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, len)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG, "Buffer: %s", buf);
    strcpy(copy, buf);
    ESP_LOGI(TAG, "Copy: %s", copy);
    char* token = strtok(buf, "&");
    char* ssid_data = strtok(token, "=");
    ssid_data = strtok(NULL, "=");
    char* new_token = strtok(copy, "&");
    new_token = strtok(NULL, "&");
    char* pwd = strtok(new_token, "=");
    pwd = strtok(NULL, "=");
    ESP_LOGI(TAG, "SSID: %s", ssid_data);
    ESP_LOGI(TAG, "PASS: %s", pwd);
    strcat(line_str, ssid_data);
    strcat(line_str, " ");
    strcat(line_str, pwd);
    ESP_ERROR_CHECK(replace_wifi(line_str, 4)); //Update the data stored in wifi_conf.txt (3rd wifi details are stored in line 4)
    ESP_ERROR_CHECK(update_wifi());             //Update the SSID and PASS matrices
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /data_3");
    ESP_LOGI(TAG, "Callback Function called: handle_data_3()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(4)");
    return ESP_OK;
}

/*Callback function whenever "/data_4" is accessed, i.e, after data has been entered through form for wifi network 4 and needs to be saved. The logic is same as the previous function*/
esp_err_t handle_data_4(httpd_req_t *req)
{
    strcpy(line_str, "");
    int len = req->content_len;
    int ret, remaining = req->content_len;
    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, len)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG, "Buffer: %s", buf);
    strcpy(copy, buf);
    ESP_LOGI(TAG, "Copy: %s", copy);
    char* token = strtok(buf, "&");
    char* ssid_data = strtok(token, "=");
    ssid_data = strtok(NULL, "=");
    char* new_token = strtok(copy, "&");
    new_token = strtok(NULL, "&");
    char* pwd = strtok(new_token, "=");
    pwd = strtok(NULL, "=");
    ESP_LOGI(TAG, "SSID: %s", ssid_data);
    ESP_LOGI(TAG, "PASS: %s", pwd);
    strcat(line_str, ssid_data);
    strcat(line_str, " ");
    strcat(line_str, pwd);
    ESP_ERROR_CHECK(replace_wifi(line_str, 5)); //Update the data stored in wifi_conf.txt (4th wifi details are stored in line 5)
    ESP_ERROR_CHECK(update_wifi());             //Update the SSID and PASS matrices
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /data_4");
    ESP_LOGI(TAG, "Callback Function called: handle_data_4()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(4)");
    return ESP_OK;
}

/*Callback function whenever "/data_5" is accessed, i.e, after data has been entered through form for wifi network 5 and needs to be saved. The logic is same as the previous function*/
esp_err_t handle_data_5(httpd_req_t *req)
{
    strcpy(line_str, "");
    int len = req->content_len;
    int ret, remaining = req->content_len;
    while (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, len)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        remaining -= ret;
    }
    buf[len] = '\0';
    ESP_LOGI(TAG, "Buffer: %s", buf);
    strcpy(copy, buf);
    ESP_LOGI(TAG, "Copy: %s", copy);
    char* token = strtok(buf, "&");
    char* ssid_data = strtok(token, "=");
    ssid_data = strtok(NULL, "=");
    char* new_token = strtok(copy, "&");
    new_token = strtok(NULL, "&");
    char* pwd = strtok(new_token, "=");
    pwd = strtok(NULL, "=");
    ESP_LOGI(TAG, "SSID: %s", ssid_data);
    ESP_LOGI(TAG, "PASS: %s", pwd);
    strcat(line_str, ssid_data);
    strcat(line_str, " ");
    strcat(line_str, pwd);
    ESP_ERROR_CHECK(replace_wifi(line_str, 6)); //Update the data stored in wifi_conf.txt (5th wifi details are stored in line 6)
    ESP_ERROR_CHECK(update_wifi());             //Update the SSID and PASS matrices
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    //ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Now displaying /data_5");
    ESP_LOGI(TAG, "Callback Function called: handle_data_5()");
    ESP_LOGI(TAG, "Webpage displayed using HTML Code returned by: get_home(4)");
    return ESP_OK;
}

/*The following are structures linking each web address with their corresponding callback functions
  Each structure contains the web address, the corresponding callback function, the HTTP method (HTTP_GET or HTTP_POST) and any user context(NULL for all the structures)*/
httpd_uri_t uri_reset = {
    .uri      = "/reset",		//the URL Address
    .method   = HTTP_GET,
    .handler  = handle_reset,	//The callback function that gets called when the URL is accessed by the user
    .user_ctx = NULL
};

httpd_uri_t uri_home = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = handle_OnConnect,
    .user_ctx = NULL
};

httpd_uri_t uri_manual = {
    .uri      = "/manual",
    .method   = HTTP_GET,
    .handler  = handle_manual,
    .user_ctx = NULL
};

httpd_uri_t uri_pause = {
    .uri      = "/pause",
    .method   = HTTP_GET,
    .handler  = handle_pause,
    .user_ctx = NULL
};

httpd_uri_t uri_auto = {
    .uri      = "/auto",
    .method   = HTTP_GET,
    .handler  = handle_auto,
    .user_ctx = NULL
};

httpd_uri_t uri_specific_path1 = {
    .uri      = "/path_details1",
    .method   = HTTP_GET,
    .handler  = handle_specific_path1,
    .user_ctx = NULL
};

httpd_uri_t uri_specific_path2 = {
    .uri      = "/path_details2",
    .method   = HTTP_GET,
    .handler  = handle_specific_path2,
    .user_ctx = NULL
};

httpd_uri_t uri_specific_path3 = {
    .uri      = "/path_details3",
    .method   = HTTP_GET,
    .handler  = handle_specific_path3,
    .user_ctx = NULL
};

httpd_uri_t uri_specific_path4 = {
    .uri      = "/path_details4",
    .method   = HTTP_GET,
    .handler  = handle_specific_path4,
    .user_ctx = NULL
};

httpd_uri_t uri_specific_path5 = {
    .uri      = "/path_details5",
    .method   = HTTP_GET,
    .handler  = handle_specific_path5,
    .user_ctx = NULL
};

httpd_uri_t uri_delete_path1 = {
    .uri      = "/delete_path1",
    .method   = HTTP_GET,
    .handler  = handle_delete_path1,
    .user_ctx = NULL
};

httpd_uri_t uri_delete_path2 = {
    .uri      = "/delete_path2",
    .method   = HTTP_GET,
    .handler  = handle_delete_path2,
    .user_ctx = NULL
};

httpd_uri_t uri_delete_path3 = {
    .uri      = "/delete_path3",
    .method   = HTTP_GET,
    .handler  = handle_delete_path3,
    .user_ctx = NULL
};

httpd_uri_t uri_delete_path4 = {
    .uri      = "/delete_path4",
    .method   = HTTP_GET,
    .handler  = handle_delete_path4,
    .user_ctx = NULL
};

httpd_uri_t uri_delete_path5 = {
    .uri      = "/delete_path5",
    .method   = HTTP_GET,
    .handler  = handle_delete_path5,
    .user_ctx = NULL
};

httpd_uri_t uri_forward = {
    .uri      = "/forward",
    .method   = HTTP_GET,
    .handler  = handle_forward,
    .user_ctx = NULL
};

httpd_uri_t uri_left = {
    .uri      = "/left",
    .method   = HTTP_GET,
    .handler  = handle_left,
    .user_ctx = NULL
};

httpd_uri_t uri_right = {
    .uri      = "/right",
    .method   = HTTP_GET,
    .handler  = handle_right,
    .user_ctx = NULL
};

httpd_uri_t uri_back = {
    .uri      = "/back",
    .method   = HTTP_GET,
    .handler  = handle_back,
    .user_ctx = NULL
};

httpd_uri_t uri_start = {
    .uri      = "/start",
    .method   = HTTP_GET,
    .handler  = handle_start,
    .user_ctx = NULL
};

httpd_uri_t uri_stop = {
    .uri      = "/stop",
    .method   = HTTP_GET,
    .handler  = handle_stop,
    .user_ctx = NULL
};

httpd_uri_t uri_save = {
    .uri      = "/save",
    .method   = HTTP_GET,
    .handler  = handle_save,
    .user_ctx = NULL
};

httpd_uri_t uri_path1 = {
    .uri      = "/path1",
    .method   = HTTP_GET,
    .handler  = handle_path1,
    .user_ctx = NULL
};

httpd_uri_t uri_path2 = {
    .uri      = "/path2",
    .method   = HTTP_GET,
    .handler  = handle_path2,
    .user_ctx = NULL
};

httpd_uri_t uri_path3 = {
    .uri      = "/path3",
    .method   = HTTP_GET,
    .handler  = handle_path3,
    .user_ctx = NULL
};

httpd_uri_t uri_path4 = {
    .uri      = "/path4",
    .method   = HTTP_GET,
    .handler  = handle_path4,
    .user_ctx = NULL
};

httpd_uri_t uri_path5 = {
    .uri      = "/path5",
    .method   = HTTP_GET,
    .handler  = handle_path5,
    .user_ctx = NULL
};

httpd_uri_t uri_choose = {
    .uri      = "/choose",
    .method   = HTTP_GET,
    .handler  = handle_choose,
    .user_ctx = NULL
};

httpd_uri_t uri_sap = {
    .uri      = "/sap",
    .method   = HTTP_GET,
    .handler  = handle_sap,
    .user_ctx = NULL
};

httpd_uri_t uri_sta = {
    .uri      = "/sta",
    .method   = HTTP_GET,
    .handler  = handle_sta,
    .user_ctx = NULL
};

httpd_uri_t uri_new = {
    .uri      = "/new",
    .method   = HTTP_GET,
    .handler  = handle_new,
    .user_ctx = NULL
};

httpd_uri_t uri_sta1 = {
    .uri      = "/sta1",
    .method   = HTTP_GET,
    .handler  = handle_sta_data1,
    .user_ctx = NULL
};

httpd_uri_t uri_sta2 = {
    .uri      = "/sta2",
    .method   = HTTP_GET,
    .handler  = handle_sta_data2,
    .user_ctx = NULL
};

httpd_uri_t uri_sta3 = {
    .uri      = "/sta3",
    .method   = HTTP_GET,
    .handler  = handle_sta_data3,
    .user_ctx = NULL
};

httpd_uri_t uri_sta4 = {
    .uri      = "/sta4",
    .method   = HTTP_GET,
    .handler  = handle_sta_data4,
    .user_ctx = NULL
};

httpd_uri_t uri_sta5 = {
    .uri      = "/sta5",
    .method   = HTTP_GET,
    .handler  = handle_sta_data5,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_mod1 = {
    .uri      = "/sta_mod_1",
    .method   = HTTP_GET,
    .handler  = handle_modify1,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_mod2 = {
    .uri      = "/sta_mod_2",
    .method   = HTTP_GET,
    .handler  = handle_modify2,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_mod3 = {
    .uri      = "/sta_mod_3",
    .method   = HTTP_GET,
    .handler  = handle_modify3,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_mod4 = {
    .uri      = "/sta_mod_4",
    .method   = HTTP_GET,
    .handler  = handle_modify4,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_mod5 = {
    .uri      = "/sta_mod_5",
    .method   = HTTP_GET,
    .handler  = handle_modify5,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_delete1 = {
    .uri      = "/sta_delete_1",
    .method   = HTTP_GET,
    .handler  = handle_delete1,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_delete2 = {
    .uri      = "/sta_delete_2",
    .method   = HTTP_GET,
    .handler  = handle_delete2,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_delete3 = {
    .uri      = "/sta_delete_3",
    .method   = HTTP_GET,
    .handler  = handle_delete3,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_delete4 = {
    .uri      = "/sta_delete_4",
    .method   = HTTP_GET,
    .handler  = handle_delete4,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_delete5 = {
    .uri      = "/sta_delete_5",
    .method   = HTTP_GET,
    .handler  = handle_delete5,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_choose1 = {
    .uri      = "/sta_choose_1",
    .method   = HTTP_GET,
    .handler  = handle_choose1,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_choose2 = {
    .uri      = "/sta_choose_2",
    .method   = HTTP_GET,
    .handler  = handle_choose2,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_choose3 = {
    .uri      = "/sta_choose_3",
    .method   = HTTP_GET,
    .handler  = handle_choose3,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_choose4 = {
    .uri      = "/sta_choose_4",
    .method   = HTTP_GET,
    .handler  = handle_choose4,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_choose5 = {
    .uri      = "/sta_choose_5",
    .method   = HTTP_GET,
    .handler  = handle_choose5,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data1 = {
    .uri      = "/data_1",
    .method   = HTTP_POST,      //POST is used for extra security since wifi credentials are passed through the form
    .handler  = handle_data_1,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data2 = {
    .uri      = "/data_2",
    .method   = HTTP_POST,      //POST is used for extra security since wifi credentials are passed through the form
    .handler  = handle_data_2,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data3 = {
    .uri      = "/data_3",
    .method   = HTTP_POST,      //POST is used for extra security since wifi credentials are passed through the form
    .handler  = handle_data_3,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data4 = {
    .uri      = "/data_4",
    .method   = HTTP_POST,      //POST is used for extra security since wifi credentials are passed through the form
    .handler  = handle_data_4,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data5 = {
    .uri      = "/data_5",
    .method   = HTTP_POST,      //POST is used for extra security since wifi credentials are passed through the form
    .handler  = handle_data_5,
    .user_ctx = NULL
};

/*Starts the webserver that is hosted on the ESP-32*/
httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG(); //default configuration
    httpd_handle_t server = NULL;                   //intialize NULL Server
    config.max_uri_handlers = 60;                   //set it to 60 to accomodate all our pre defined structures
    if (httpd_start(&server, &config) == ESP_OK) {  //Now assosciate all the structures defined earlier to the web server
        httpd_register_uri_handler(server, &uri_reset);
        httpd_register_uri_handler(server, &uri_home);
        httpd_register_uri_handler(server, &uri_manual);
        httpd_register_uri_handler(server, &uri_pause);
        httpd_register_uri_handler(server, &uri_auto);
        httpd_register_uri_handler(server, &uri_forward);
        httpd_register_uri_handler(server, &uri_left);
        httpd_register_uri_handler(server, &uri_right);
        httpd_register_uri_handler(server, &uri_back);
        httpd_register_uri_handler(server, &uri_choose);
        httpd_register_uri_handler(server, &uri_sap);
        httpd_register_uri_handler(server, &uri_sta);
        httpd_register_uri_handler(server, &uri_start);
        httpd_register_uri_handler(server, &uri_stop);
        httpd_register_uri_handler(server, &uri_save);
        httpd_register_uri_handler(server, &uri_new);
        httpd_register_uri_handler(server, &uri_specific_path1);
        httpd_register_uri_handler(server, &uri_specific_path2);
        httpd_register_uri_handler(server, &uri_specific_path3);
        httpd_register_uri_handler(server, &uri_specific_path4);
        httpd_register_uri_handler(server, &uri_specific_path5);
        httpd_register_uri_handler(server, &uri_delete_path1);
        httpd_register_uri_handler(server, &uri_delete_path2);
        httpd_register_uri_handler(server, &uri_delete_path3);
        httpd_register_uri_handler(server, &uri_delete_path4);
        httpd_register_uri_handler(server, &uri_delete_path5);
        httpd_register_uri_handler(server, &uri_path1);
        httpd_register_uri_handler(server, &uri_path2);
        httpd_register_uri_handler(server, &uri_path3);
        httpd_register_uri_handler(server, &uri_path4);
        httpd_register_uri_handler(server, &uri_path5);
        httpd_register_uri_handler(server, &uri_sta1);
        httpd_register_uri_handler(server, &uri_sta2);
        httpd_register_uri_handler(server, &uri_sta3);
        httpd_register_uri_handler(server, &uri_sta4);
        httpd_register_uri_handler(server, &uri_sta5);
        httpd_register_uri_handler(server, &uri_sta_mod1);
        httpd_register_uri_handler(server, &uri_sta_mod2);
        httpd_register_uri_handler(server, &uri_sta_mod3);
        httpd_register_uri_handler(server, &uri_sta_mod4);
        httpd_register_uri_handler(server, &uri_sta_mod5);
        httpd_register_uri_handler(server, &uri_sta_delete1);
        httpd_register_uri_handler(server, &uri_sta_delete2);
        httpd_register_uri_handler(server, &uri_sta_delete3);
        httpd_register_uri_handler(server, &uri_sta_delete4);
        httpd_register_uri_handler(server, &uri_sta_delete5);
        httpd_register_uri_handler(server, &uri_sta_choose1);
        httpd_register_uri_handler(server, &uri_sta_choose2);
        httpd_register_uri_handler(server, &uri_sta_choose3);
        httpd_register_uri_handler(server, &uri_sta_choose4);
        httpd_register_uri_handler(server, &uri_sta_choose5);
        httpd_register_uri_handler(server, &uri_sta_data1);
        httpd_register_uri_handler(server, &uri_sta_data2);
        httpd_register_uri_handler(server, &uri_sta_data3);
        httpd_register_uri_handler(server, &uri_sta_data4);
        httpd_register_uri_handler(server, &uri_sta_data5);
    }
    ESP_LOGI(TAG, "On core %d", xPortGetCoreID());
    return server;
}

/*Stops the webserver (Ref: Official Github Repo)*/
void stop_webserver(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
    }
}

/*Not really required(Ref: Official Github Repo)*/
/*static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}
*/
/*Not really required(Ref: Official Github Repo)*/
/*static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}*/

/*Not really required(Ref: Official Github Repo)*/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

/*Used for setting up SAP mode (Ref: Official Github Repo). Recommended not to change*/
void init_sap()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = DEFAULT_SSID,				//these are defined at the very beginning of the code
            .ssid_len = strlen(DEFAULT_SSID),
            .password = DEFAULT_PASS,
            .max_connection = MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s Password:%s", DEFAULT_SSID, DEFAULT_PASS);
}

/*Not really required((Ref: Official Github Repo))*/
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/*Used for setting up STA mode (Ref: Official Github Repo). Recommended not to change*/
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {       //Initialize with dummy values
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strcpy((char *)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID);        //Put the correct Wifi SSID that the ESP needs to connect to
    strcpy((char *)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS);    //Put the correct Wifi Password required
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        ESP_ERROR_CHECK(replace_wifi("SAP", 1));
        ESP_LOGI(TAG, "SAP Data Written");
        vTaskDelay(100);
        esp_restart();
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

/*Used for setting up PWM Channel (Ref: Official Github Repo)*/
void init_pwm()
{
    ledc_timer_config_t led_timer = {
            .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty, means PWM value can go from 0 to (2^13-1)
            .freq_hz = 5000,                      // frequency of PWM signal
            .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
            .timer_num = LEDC_TIMER_0,            // timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
    ledc_timer_config(&led_timer);
    led_channel.channel = LEDC_CHANNEL_0;
    led_channel.duty = 0;						//the duty value of the PWM signal
    led_channel.gpio_num = 2;					//the GPIO pin to which the the PWM signal will be supplied
    led_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    led_channel.hpoint = 0;
    led_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&led_channel);
}

/*Function that will run parallely on Core 1*/
void Task1code( void * pvParameters ){
	
    while(1){   //Put code here. This is like void loop() in arduino
    	//ESP_LOGI(TAG, "Infinite Loop running On core %d", xPortGetCoreID());
        if(record_flag == 1){					//record_flag, flag, auto_flag are updated in other portions of the code 
            if(flag == 0) move_forward();
            else if(flag == 1) move_left();
            else if(flag == 2) move_right();
            else if(flag == 3) move_back();
            else move_stop();
        }
        else if(auto_flag == 1){
            if(flag == 0) move_forward();
            else if(flag == 1) move_left();
            else if(flag == 2) move_right();
            else if(flag == 3) move_back();
            else move_stop();
        }
        else
            move_stop();
    }
}

/*Main function that gets called once at the start when ESP boots*/
void app_main(void)
{
    static httpd_handle_t server = NULL;
    init_spiffs();  //Initialize SPIFFS File System
    init_pwm();     //Initialize PWM channel
    conn_flag = main_update();  //Update the necessary variables
    if(conn_flag == 0)          //Check the returned value         
        init_sap();				//Start SAP mode
    else
        wifi_init_sta();		//Start STA mode
    server = start_webserver(); //Start the web server
    xTaskCreatePinnedToCore(    //Pinning a task in core 1
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */                  
  	//delay(500);
    ESP_ERROR_CHECK(update_paths());    //Update the variables related to paths.txt
    ESP_ERROR_CHECK(mdns_init());       //Initialize MDNS Service
    ESP_ERROR_CHECK(mdns_hostname_set("esp32"));    //Set hostname to esp32. Now you can either type the IP Address or "esp32.local" for a device which has MDNS
    ESP_LOGI(TAG, "mdns hostname set to: [esp32]");
    ESP_ERROR_CHECK(mdns_instance_name_set("web_server"));
    /*ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));*/
    //server = start_webserver();
}