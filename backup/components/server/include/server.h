#ifndef SERVER_H
#define SERVER_H

/*The following header files are included*/
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
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

/*The variables that keep track of various things while the code is running*/
extern const char *TAG;                     //Name used for ESP_LOGI statements. Feel free to set it to whatever you want
extern int flag;                           //Used for determining in which direction it is travelling 
extern int record_flag;                     //Denote whether path is being recorded or not
extern int conn_flag;                      //Denote which Wifi mode the ESP is used : 0 = SAP, 1 to WIFI_NUM = STA wifi index
extern int auto_flag;                       //Denote whether auto mode is on or off
extern int manual_flag;
extern int total ;                           //Total number of network credentials for STA mode stored till now
extern int total_paths;                     //Total number of paths stored till now
extern int64_t prev_mili;                   //Used for determining the time duration between 2 commands while controlling the bot in manual mode
extern int64_t curr_mili;                   //Used for determining the time duration between 2 commands while controlling the bot in manual mode
extern float time_duration;                 //Used for storing the time duration between 2 commands while controlling the bot in manual mode
char buf[DATA_LEN];                      //Used for extracting ssid from POST header data whenever User stores new Network Credentials
char copy[DATA_LEN];                     //Used for extracting password from POST header data whenevr User stores new Network Credentials
char line_str[LINE_LEN];                 //Used for extracting lines from stored files
char ssid[WIFI_NUM][SSID_LEN];           //Store all the network ssids that the ESP can use in STA mode
char pass[WIFI_NUM][PASS_LEN];           //Store all the network passwords that the ESP can use in STA mode

struct httpd_uri_t {                     //structure intialization for all the structure pointer
    const char       *uri;               //linking web addresses and corresponding call back functions
    httpd_method_t    method; 
    esp_err_t (*handler)(httpd_req_t *r);
    void *user_ctx;
};

esp_err_t handle_OnConnect(httpd_req_t *req);
esp_err_t handle_reset(httpd_req_t *req);
esp_err_t handle_start(httpd_req_t *req);
esp_err_t handle_path1(httpd_req_t *req);
esp_err_t handle_path2(httpd_req_t *req);
esp_err_t handle_path3(httpd_req_t *req);
esp_err_t handle_path4(httpd_req_t *req);
esp_err_t handle_path5(httpd_req_t *req);
esp_err_t handle_manual(httpd_req_t *req);
esp_err_t handle_specific_path1(httpd_req_t *req);
esp_err_t handle_specific_path2(httpd_req_t *req);
esp_err_t handle_specific_path3(httpd_req_t *req);
esp_err_t handle_specific_path4(httpd_req_t *req);
esp_err_t handle_specific_path5(httpd_req_t *req);
esp_err_t handle_delete_path1(httpd_req_t *req);
esp_err_t handle_delete_path2(httpd_req_t *req);
esp_err_t handle_delete_path3(httpd_req_t *req);
esp_err_t handle_delete_path4(httpd_req_t *req);
esp_err_t handle_delete_path5(httpd_req_t *req);
esp_err_t handle_auto(httpd_req_t *req);
esp_err_t handle_forward(httpd_req_t *req);
esp_err_t handle_left(httpd_req_t *req);
esp_err_t handle_right(httpd_req_t *req);
esp_err_t handle_back(httpd_req_t *req);
esp_err_t handle_stop(httpd_req_t *req);
esp_err_t handle_pause(httpd_req_t *req);
esp_err_t handle_save(httpd_req_t *req);
esp_err_t handle_choose(httpd_req_t *req);
esp_err_t handle_sap(httpd_req_t *req);
esp_err_t handle_sta(httpd_req_t *req);
esp_err_t handle_new(httpd_req_t *req);
esp_err_t handle_sta_data1(httpd_req_t *req);
esp_err_t handle_modify1(httpd_req_t *req);
esp_err_t handle_delete1(httpd_req_t *req);
esp_err_t handle_choose1(httpd_req_t *req);
esp_err_t handle_sta_data2(httpd_req_t *req);
esp_err_t handle_modify2(httpd_req_t *req);
esp_err_t handle_delete2(httpd_req_t *req);
esp_err_t handle_choose2(httpd_req_t *req);
esp_err_t handle_sta_data3(httpd_req_t *req);
esp_err_t handle_modify3(httpd_req_t *req);
esp_err_t handle_delete3(httpd_req_t *req);
esp_err_t handle_choose3(httpd_req_t *req);
esp_err_t handle_sta_data4(httpd_req_t *req);
esp_err_t handle_modify4(httpd_req_t *req);
esp_err_t handle_delete4(httpd_req_t *req);
esp_err_t handle_choose4(httpd_req_t *req);
esp_err_t handle_sta_data5(httpd_req_t *req);
esp_err_t handle_modify5(httpd_req_t *req);
esp_err_t handle_delete5(httpd_req_t *req);
esp_err_t handle_choose5(httpd_req_t *req);
esp_err_t handle_data_1(httpd_req_t *req);
esp_err_t handle_data_2(httpd_req_t *req);
esp_err_t handle_data_3(httpd_req_t *req);
esp_err_t handle_data_4(httpd_req_t *req);
esp_err_t handle_data_5(httpd_req_t *req);

char* default_page();
char* choose_page();
char* get_sta();
char* get_sta_data(int local_flag);
char* get_form(int local_flag);  
char* get_auto();
char* get_path_specific(int local_flag);
char* get_home(int local_flag);
char* manual_mode();
char* SendHTML(uint8_t local_flag);
char* get_stop();

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);
/* All the above functions defined in "server.c" file */

esp_err_t replace_wifi(char* line, int n);
esp_err_t update_number(int n);
esp_err_t delete(int n);
esp_err_t delete_specific_path(int n);
esp_err_t delete_paths(int n);
esp_err_t update_wifi();
esp_err_t update_paths();

#endif
// WE should only declare the vars in .h and define it in any one c file