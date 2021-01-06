#include "server.h"

static ledc_channel_config_t led_channel[2];       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output. Will have to create another for controlling 2 motors
#define rightm 13
#define right_dir 12
#define rpwm 14
#define leftm 27
#define left_dir 26
#define lpwm 25
struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};
int var=0;
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
    for( var = 0;var < 2; var++) {
        ledc_channel_config(&led_channel[var]);
    }
}

/*The following are dummy functions for movement of the bot*/
void move_forward()
{   gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 1);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 1);
    //Assuming that motors shows ideal responce when same pwm is given 
    for (var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,5000);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }    
}                                                                       //Similar functions below with different duty cycles

void move_left()
{      gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 0);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 1);
    for (var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,5000);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }  
}

void move_right()
{   gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 1);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 0);
    for (var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,5000);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }   
}

void move_back()
{   gpio_set_level(rightm, 1);
    gpio_set_level(right_dir, 0);
    gpio_set_level(leftm, 1);
    gpio_set_level(left_dir, 0);
    for (var = 0; var < 2; var++)
    {
        ledc_set_duty(led_channel[var].speed_mode, led_channel[var].channel,5000);
        ledc_update_duty(led_channel[var].speed_mode, led_channel[var].channel);
    }     
}

void move_stop()
{   gpio_set_level(rightm, 0);
    gpio_set_level(leftm, 0);

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
