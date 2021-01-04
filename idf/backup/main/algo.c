#include "server.h"

static ledc_channel_config_t led_channel;       //Data Structure having various fields specifying the PWM details for a single channel. A single channel for a single PWM output. Will have to create another for controlling 2 motors

struct point{                                   //Data Structure for storing points
    double x;
    double y;
    double theta;
};

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
