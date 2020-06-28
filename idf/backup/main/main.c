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

#define EXAMPLE_ESP_MAXIMUM_RETRY 5             //maximum number of times the esp will try to connect to a network in STA mode
#define DEFAULT_SSID "myssid"                   //default used in SAP and STA mode
#define DEFAULT_PASS "qwerty1234"               //default used in SAP and STA mode
#define MAX_CONN 5                              //maximum number of connections in SAP mode
#define BASE_PATH "/spiffs"                     //base path of the partition
#define MAX_FILES 5                             //maximum number of files to be stored in the /spiffs file system
#define RESET_FLAG true                         //whether partition should be formatted if mount fails
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_NUM 5                              //number of wifi credentials to be stored
#define DATA_LEN 108                            //108 = 4 + 1 + 32 + 1 + 4 + 1 + 64 + 1
#define LINE_LEN 98                             //98 = 32 + 1 + 64 + 1
#define SSID_LEN 32                             //the maximum length of ssid that can be used by the esp is 32
#define PASS_LEN 64                             //the maximum length of pass that can be used by the esp is 64
#define PATH_NUM 5                              //the maximum number of paths that can be stored

static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static int flag = -1;                           //used for determining which direction it is travelling
static int s_retry_num = 0;                     //number of times the esp has tried to connect to ssid
static int64_t prev_mili = 0;                   //used for determining the time duration between 2 commands
static int64_t curr_mili = 0;                   //used for determining the time duration between 2 commands
static float time_duration = 0;                 //used for storing the time duration between 2 commands
static char buf[DATA_LEN];                      //used for extracting ssid from POST data
static char copy[DATA_LEN];                     //used for extracting password from POST data
static int conn_flag = 0;                       //denote which mode connected: 0=SAP 1:WIFI_NUM = STA wifi index
static int record_flag = 0;                     //denote whether path is being recorded or not
static char ssid[WIFI_NUM][SSID_LEN];           //store all the network ssids
static char pass[WIFI_NUM][PASS_LEN];           //store all the network passwords
static char EXAMPLE_ESP_WIFI_SSID[SSID_LEN];    //store the network ssid that the esp is using currently in STA mode
static char EXAMPLE_ESP_WIFI_PASS[PASS_LEN];    //store the network password that the esp is using currently in STA mode
static int total = 0;                           //total number of SSID's stored till now
static char line_str[LINE_LEN];
static int total_paths = 0;                     //total number of paths stored till now
static ledc_channel_config_t led_channel;
/*Replaces the nth line in the file /wifi_conf.txt with the line supplied in the argument
  Note: *line should not end with \n*/

void move_forward()
{
    ledc_set_duty(led_channel.speed_mode, led_channel.channel, 8192);
    ledc_update_duty(led_channel.speed_mode, led_channel.channel);
}

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

esp_err_t replace_wifi(char* line, int n)
{
    char str[LINE_LEN];
    int linectr = 0;
    FILE* f_r = fopen("/spiffs/wifi_conf.txt", "r");
    FILE* f_w = fopen("/spiffs/temp.txt", "w");
    if(f_r == NULL){
        printf("Error opening file wifi_conf.txt\n");
        return ESP_FAIL;
    }
    if(f_w == NULL){
        printf("Error opening file temp.txt\n");
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
                fprintf(f_w, "%s", line);
                fprintf(f_w, "%s", "\n");
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

esp_err_t update_number(int n){
    char line[2];
    char str[LINE_LEN];
    int linectr = 0;
    total_paths = total_paths + n;
    sprintf(line, "%d", total_paths);
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
        strcpy(str, "\0");
        fgets(str, LINE_LEN, f_r);
        if(!feof(f_r))
        {
            linectr++;
            if(linectr == 1){
                fprintf(f_w, "%s", line);
                fprintf(f_w, "%s", "\n");
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

/*Used for "deleting" network credentials. All network credentials below it are moved up and the last line is replaced by example example*/
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
            if(linectr == (n+1)){
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
        strcpy(str, "\0");
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

/*Initialize SPIFFS file system. Needs to be called on every reboot*/
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

int main_update()
{
    int i, conn_flag_local = 0;
    struct stat st;
    struct stat st1;
    char line[LINE_LEN];
    char* pos;
    ESP_LOGI(TAG, "Checking connection type");
    if (stat("/spiffs/wifi_conf.txt", &st) != 0) {
        FILE* f = fopen("/spiffs/wifi_conf.txt", "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return flag;
        }
        fprintf(f, "SAP\n");
        for(i = 0; i < WIFI_NUM; i++)
            fprintf(f, "example example\n");
        fprintf(f, "0\n0\n");
        fclose(f);
        ESP_LOGI(TAG, "File written");
    }
    else{
        FILE* f = fopen("/spiffs/wifi_conf.txt", "r");
        if(f == NULL){
            printf("Failed to open file\n");
            return 0;
        }
        fgets(line, sizeof(line), f);
        pos = strchr(line, '\n');
        if(pos){
            *pos = '\0';
        }
        if(strcmp(line, "SAP") == 0){
            char* token;
            for(i = 0; i <= WIFI_NUM+1; i++){
                fgets(line, sizeof(line), f);
                pos = strchr(line, '\n');
                if(pos){
                *pos = '\0';
                }
                if(i == WIFI_NUM)
                    continue;
                if(i == WIFI_NUM+1){
                    total = atoi(line); //denote total number of saved networks
                    continue;
                }
                ESP_LOGI(TAG, "%s", line);
                token = strtok(line, " ");
                strcpy(ssid[i], token);
                token = strtok(NULL, " ");
                strcpy(pass[i], token);
            }
            ESP_LOGI(TAG, "STA Mode");
            for(i = 0; i < WIFI_NUM; i++){
                ESP_LOGI(TAG, "%dth Network SSID: %s\n", (i+1), ssid[i]);
                ESP_LOGI(TAG, "%dth Network Password: %s\n", (i+1), pass[i]);
            }
            fclose(f);
        }
        else{
            char* token;
            for(i = 0; i <= WIFI_NUM+1; i++){
                fgets(line, sizeof(line), f);
                pos = strchr(line, '\n');
                if(pos){
                *pos = '\0';
                }
                if(i == WIFI_NUM){
                    conn_flag_local = atoi(line);   //denote which network to connect 1 to NUM
                    continue;
                }
                if(i == WIFI_NUM+1){
                    total = atoi(line); //denote total number of saved networks
                    continue;
                }
                token = strtok(line, " ");
                strcpy(ssid[i], token);
                token = strtok(NULL, " ");
                strcpy(pass[i], token);
            }
            ESP_LOGI(TAG, "STA Mode");
            for(i = 0; i < WIFI_NUM; i++){
                if((i+1) == conn_flag_local) //(i+1) is done because i is from 0:WIFI_NUM while conn_flag_local is from 1:WIFI_NUM
                {
                    strcpy(EXAMPLE_ESP_WIFI_SSID, ssid[i]);
                    strcpy(EXAMPLE_ESP_WIFI_PASS, pass[i]);
                }
                ESP_LOGI(TAG, "%dth Network SSID: %s\n", (i+1), ssid[i]);
                ESP_LOGI(TAG, "%dth Network Password: %s\n", (i+1), pass[i]);
            }
            fclose(f);
        }
    }
    if (stat("/spiffs/paths.txt", &st1) != 0){
        FILE* f = fopen("/spiffs/paths.txt", "w");
        if(f == NULL)
            ESP_LOGE(TAG, "Failed to open paths.txt for writing");
        fprintf(f, "0\n");
        fclose(f);
        total_paths = 0;
    }
    else{
        FILE* f = fopen("/spiffs/paths.txt", "r");
        if(f == NULL)
            ESP_LOGE(TAG, "Failed to open path.txt for reading");
        pos = strchr(line, '\n');
        if(pos){
            *pos = '\0';
        }
        total_paths = atoi(line);
        fclose(f);
    }
    return conn_flag_local;
}

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

esp_err_t update_paths()
{
    char line[LINE_LEN], *pos;
    FILE* f = fopen("/spiffs/paths.txt", "r");
    if(f == NULL){
        ESP_LOGE(TAG, "Failed to open path.txt");
        return ESP_FAIL;
    }
    fgets(line, sizeof(line), f);
    pos = strchr(line, '\n');
    if(pos){
        *pos = '\0';
    }
    total_paths = atoi(line);
    fclose(f);
    return ESP_OK;
}


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
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");
    strcat(ptr, "<p>Press to select Manual mode</p><a class=\"button button-on\" href=\"/manual\">MANUAL</a>\n");
    strcat(ptr, "<p>Press to select Auto mode</p><a class=\"button button-on\" href=\"/auto\">AUTO</a>\n");
    strcat(ptr, "<p>Press to choose Connection Mode</p><a class=\"button button-on\" href=\"/choose\">SAP/STA</a>\n");
    strcat(ptr, "<p>Press to reset the ESP</p><a class=\"button button-on\" href=\"/reset\">FACTORY\nRESET</a>\n");

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

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
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<p>Press to select SAP mode</p><a class=\"button button-on\" href=\"/sap\">SAP</a>\n");
    strcat(ptr, "<p>Press to select STA mode</p><a class=\"button button-on\" href=\"/sta\">STA</a>\n");
    strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

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
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<p>Press to store New network credentials</p><a class=\"button button-on\" href=\"/new\">NEW</a>\n");
    for(i = 0; i < total; i++)
    {
        sprintf(str, "%d", i+1);
        strcat(ptr, "<p>Press for more options</p><a class=\"button button-on\" href=\"/sta");
        strcat(ptr, str);
        strcat(ptr, "\">Network ");
        strcat(ptr, str);
        strcat(ptr, "</a>\n");
    }
    strcat(ptr, "<p>Press to go back</p><a class=\"button button-on\" href=\"/choose\">BACK</a>\n");
    return ptr;
}

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
    if(conn_flag == 0)
        strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");
    else
        strcat(ptr, "<h3>Using Station(STA) Mode</h3>\n");

    strcat(ptr, "<h3>Network SSID: ");
    strcat(ptr, ssid[local_flag-1]);
    strcat(ptr, "</h3>\n");
    char str[2];
    sprintf(str, "%d", local_flag);
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

char* get_form(int local_flag)  //local_flag min value is 1. Note: Need to set maximum character lengths
{
    char str[2];
    sprintf(str, "%d", local_flag);
    char* ptr = (char *)calloc(2048, sizeof(char));
    strcat(ptr, "<form action=\"/data_");
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
    for(i = 1; i <= total_paths; i++)
    {
        sprintf(str, "%d", i);
        strcat(ptr, "<p>Press for more details</p><a class=\"button button-on\" href=\"/path_details");
        strcat(ptr, str);
        strcat(ptr, "\">Path ");
        strcat(ptr, str);
        strcat(ptr, "</a>\n");
    }    
    strcat(ptr, "<p>Press to return to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

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
    strcat(ptr, str);
    strcat(ptr, "\">Execute</a>\n");
    strcat(ptr, "<p>Press to delete this path</p><a class=\"button button-on\" href=\"/delete_path");
    strcat(ptr, str);
    strcat(ptr, "\">Delete</a>\n");
    strcat(ptr, "<p>Press to return to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

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
            if(linectr == (local_flag+1))
                break;
        }
    }
    ESP_LOGI(TAG, "%s", str);
    fclose(f_r);
    char* token = strtok(str, "\t");
    while(token!=NULL)
    {
        char ch = token[0];
        switch(ch){
            case 'f':move_forward();break;
            case 'l':move_left();break;
            case 'r':move_right();break;
            case 'b':move_back();break;
            default:move_stop();break;
        }
        ESP_LOGI(TAG, "Direction: %c", ch);
        token++;
        float time = atof(token);
        ESP_LOGI(TAG, "Time: %f", time);
        vTaskDelay(time/portTICK_PERIOD_MS);
        token = strtok(NULL, "\t");
    }
    move_stop();
    return ESP_OK;
}

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

    if(total_paths == PATH_NUM){
        strcat(ptr, "<h3>Maximum number of stored paths reached. Please delete saved paths to store new paths</h3>\n");
        strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
        strcat(ptr, "</body>\n");
        strcat(ptr, "</html>\n");
        return ptr;
    }

    if(record_flag == 0){
        strcat(ptr, "<h3>Path recording not yet started</h3>\n");
        strcat(ptr, "<p>Click to Start</p><a class=\"button button-on\" href=\"/start\">START</a>\n");
    }
    else{
        strcat(ptr, "<h3>Now recording Path: ");
        strcat(ptr, str);
        strcat(ptr, "</h3>\n");
        strcat(ptr, "<p>Click to Stop</p><a class=\"button button-on\" href=\"/stop\">STOP</a>\n");
    }
    strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");
    strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");
    strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");
    strcat(ptr, "<p>Back: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n");
    if(record_flag == 0)
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
    {strcat(ptr, "<p>Forward: ON</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");}

    if(local_flag==1)
    {strcat(ptr, "<p>Left: ON</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");}

    if(local_flag==2)
    {strcat(ptr, "<p>Right: ON</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");}

    if(local_flag==3)
    {strcat(ptr, "<p>Backwards: ON</p><a class=\"button button-off\" href=\"/pause\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Backwards: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n");}

    if(record_flag == 0)
        strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

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

esp_err_t handle_OnConnect(httpd_req_t *req)
{
    flag = -1;
    char* resp = default_page();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_reset(httpd_req_t *req)
{
    remove("/spiffs/wifi_conf.txt");
    remove("/spiffs/paths.txt");
    httpd_resp_send(req, "Device will restart now", strlen("Device will restart now"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_start(httpd_req_t *req)
{
    record_flag = 1;
    flag = -1;
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
    return ESP_OK;
}

esp_err_t handle_path1(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(1));
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_path2(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(2));
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_path3(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(3));
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_path4(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(4));
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_path5(httpd_req_t *req)
{
    ESP_ERROR_CHECK(get_path(5));
    char* resp = get_home(0);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_manual(httpd_req_t *req)
{
    record_flag = 0;
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000;;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = -1;
    delete_paths(total_paths+1);
    char* resp = manual_mode();
    prev_mili = esp_timer_get_time();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_pause(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = -1;
    char* resp = manual_mode();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
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
    return ESP_OK;  
}

esp_err_t handle_specific_path1(httpd_req_t *req)//to change
{
    char* resp = get_path_specific(1);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_specific_path2(httpd_req_t *req)//to change
{
    char* resp = get_path_specific(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_specific_path3(httpd_req_t *req)//to change
{
    char* resp = get_path_specific(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_specific_path4(httpd_req_t *req)//to change
{
    char* resp = get_path_specific(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_specific_path5(httpd_req_t *req)//to change
{
    char* resp = get_path_specific(5);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_delete_path1(httpd_req_t *req)//to change
{
    ESP_ERROR_CHECK(delete_specific_path(1));
    ESP_ERROR_CHECK(update_number(-1));
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_delete_path2(httpd_req_t *req)//to change
{
    ESP_ERROR_CHECK(delete_specific_path(2));
    ESP_ERROR_CHECK(update_number(-1));
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_delete_path3(httpd_req_t *req)//to change
{
    ESP_ERROR_CHECK(delete_specific_path(3));
    ESP_ERROR_CHECK(update_number(-1));
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_delete_path4(httpd_req_t *req)//to change
{
    ESP_ERROR_CHECK(delete_specific_path(4));
    ESP_ERROR_CHECK(update_number(-1));
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_delete_path5(httpd_req_t *req)//to change
{
    ESP_ERROR_CHECK(delete_specific_path(5));
    ESP_ERROR_CHECK(update_number(-1));
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_auto(httpd_req_t *req)//to change
{
    flag = 4;
    char* resp = get_auto();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_forward(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000;;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 0;
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
        move_forward();
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
    return ESP_OK;
}

esp_err_t handle_left(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 1;
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
        move_left();
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
    return ESP_OK;
}

esp_err_t handle_right(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 2;
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
        move_right();
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
    return ESP_OK;
}

esp_err_t handle_back(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    flag = 3;
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1)
    {
        move_back();
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
    return ESP_OK;
}

esp_err_t handle_stop(httpd_req_t *req)
{
    char det = determine(flag);
    curr_mili = esp_timer_get_time();
    time_duration = (curr_mili - prev_mili)/1000.0;
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    ESP_LOGI(TAG, "Reading values");
    if(record_flag == 1)
    {
        move_stop();
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%.3f", time_duration);
        fputc('\n', f);
        //fputc('\0', f);
        fclose(f);
    }
    char* resp = get_stop();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_save(httpd_req_t *req)
{
    update_number(1);
    char* resp = get_home(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_choose(httpd_req_t *req)
{
    char* resp = choose_page();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_sap(httpd_req_t *req)
{
    httpd_resp_send(req, "Device will restart using SAP mode", strlen("Device will restart using SAP mode"));
    ESP_ERROR_CHECK(replace_wifi("SAP", 1));
    ESP_LOGI(TAG, "SAP Data Written");
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_sta(httpd_req_t *req)
{
    char* resp = get_sta();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_new(httpd_req_t *req)
{
    if(total == WIFI_NUM)
    {
        char* resp = get_home(1);
        httpd_resp_send(req, resp, strlen(resp));
        free(resp);
        return ESP_OK;
    }
    else
    {
        total = total + 1;
        char str[20];
        sprintf(str, "%d", total);
        replace_wifi(str, WIFI_NUM+3);
        char* resp = get_form(total);
        httpd_resp_send(req, resp, strlen(resp));
        free(resp);
        return ESP_OK;
    }
    return ESP_OK;
}

esp_err_t handle_sta_data1(httpd_req_t *req)
{
    char* resp = get_sta_data(1);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_modify1(httpd_req_t *req)
{
    char* resp = get_form(1);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_delete1(httpd_req_t *req)
{
    char* resp = get_home(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    total = total - 1;
    char str[20];
    sprintf(str, "%d", total);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+3));
    ESP_ERROR_CHECK(delete(2));
    ESP_ERROR_CHECK(update_wifi());
    if(total == 0)
        ESP_ERROR_CHECK(handle_sap(req));
    return ESP_OK;
}

esp_err_t handle_choose1(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 1);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 1");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_sta_data2(httpd_req_t *req)
{
    char* resp = get_sta_data(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_modify2(httpd_req_t *req)
{
    char* resp = get_form(2);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    return ESP_OK;
}

esp_err_t handle_choose2(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 2);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 2");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_sta_data3(httpd_req_t *req)
{
    char* resp = get_sta_data(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_modify3(httpd_req_t *req)
{
    char* resp = get_form(3);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    return ESP_OK;
}

esp_err_t handle_choose3(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 3);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA WIFI 3");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_sta_data4(httpd_req_t *req)
{
    char* resp = get_sta_data(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_modify4(httpd_req_t *req)
{
    char* resp = get_form(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    return ESP_OK;
}

esp_err_t handle_choose4(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 4);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 4");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_sta_data5(httpd_req_t *req)
{
    char* resp = get_sta_data(5);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_modify5(httpd_req_t *req)
{
    char* resp = get_form(5);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    return ESP_OK;
}

esp_err_t handle_choose5(httpd_req_t *req)
{
    char str[20];
    sprintf(str, "%d", 5);
    ESP_ERROR_CHECK(replace_wifi(str, WIFI_NUM+2));
    ESP_ERROR_CHECK(replace_wifi("STA", 1));
    ESP_LOGI(TAG, "STA Wifi 5");
    httpd_resp_send(req, "Device will restart now and connect to wifi network", strlen("Device will restart now and connect to wifi network"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_data_1(httpd_req_t *req)
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
    ESP_ERROR_CHECK(replace_wifi(line_str, 2));
    ESP_ERROR_CHECK(update_wifi());
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    ESP_ERROR_CHECK(replace_wifi(line_str, 3));
    ESP_ERROR_CHECK(update_wifi());
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    ESP_ERROR_CHECK(replace_wifi(line_str, 4));
    ESP_ERROR_CHECK(update_wifi());
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    ESP_ERROR_CHECK(replace_wifi(line_str, 5));
    ESP_ERROR_CHECK(update_wifi());
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

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
    ESP_ERROR_CHECK(replace_wifi(line_str, 6));
    ESP_ERROR_CHECK(update_wifi());
    char* resp = get_home(4);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

httpd_uri_t uri_reset = {
    .uri      = "/reset",
    .method   = HTTP_GET,
    .handler  = handle_reset,
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
    .method   = HTTP_POST,
    .handler  = handle_data_1,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data2 = {
    .uri      = "/data_2",
    .method   = HTTP_POST,
    .handler  = handle_data_2,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data3 = {
    .uri      = "/data_3",
    .method   = HTTP_POST,
    .handler  = handle_data_3,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data4 = {
    .uri      = "/data_4",
    .method   = HTTP_POST,
    .handler  = handle_data_4,
    .user_ctx = NULL
};

httpd_uri_t uri_sta_data5 = {
    .uri      = "/data_5",
    .method   = HTTP_POST,
    .handler  = handle_data_5,
    .user_ctx = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.max_uri_handlers = 60;
    if (httpd_start(&server, &config) == ESP_OK) {
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
    return server;
}

void stop_webserver(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
    }
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}
static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

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
            .ssid = DEFAULT_SSID,
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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strcpy((char *)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS);
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

void init_pwm()
{
    ledc_timer_config_t led_timer = {
            .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
            .freq_hz = 5000,                      // frequency of PWM signal
            .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
            .timer_num = LEDC_TIMER_0,            // timer index
            .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
        };
    ledc_timer_config(&led_timer);
    led_channel.channel = LEDC_CHANNEL_0;
    led_channel.duty = 0;
    led_channel.gpio_num = 2;
    led_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    led_channel.hpoint = 0;
    led_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&led_channel);
}

void app_main(void)
{
    static httpd_handle_t server = NULL;
    init_spiffs();
    init_pwm();
    conn_flag = main_update();
    if(conn_flag == 0)
        init_sap();
    else
        wifi_init_sta();
    ESP_ERROR_CHECK(update_paths());
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp32"));
    ESP_LOGI(TAG, "mdns hostname set to: [esp32]");
    ESP_ERROR_CHECK(mdns_instance_name_set("web_server"));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();
}
