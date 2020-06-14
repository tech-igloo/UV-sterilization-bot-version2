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

#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define DEFAULT_SSID "myssid"
#define DEFAULT_PASS "qwerty1234"
#define MAX_CONN 5
#define BASE_PATH "/spiffs"
#define MAX_FILES 5
#define RESET_FLAG true
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static int flag = -1;
static int s_retry_num = 0;
static int64_t prev_mili = 0;
static int64_t curr_mili = 0;
static float time_duration = 0;
static char buf[107];
static char copy[107];
static int conn_flag = 0;
static int record_flag = 0;
static char EXAMPLE_ESP_WIFI_SSID[32];
static char EXAMPLE_ESP_WIFI_PASS[64];

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

int conn_type()
{
    int flag = 0;
    struct stat st;
    char line[102];
    ESP_LOGI(TAG, "Checking connection type");
    if (stat("/spiffs/wifi_conf.txt", &st) != 0) {
        FILE* f = fopen("/spiffs/wifi_conf.txt", "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return flag;
        }
        fprintf(f, "SAP\n");
        fclose(f);
        ESP_LOGI(TAG, "File written");
        return flag;
    }
    else{
        FILE* f = fopen("/spiffs/wifi_conf.txt", "r");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for reading");
            return flag;
        }
        fgets(line, sizeof(line), f);
        fclose(f);
        char* pos = strchr(line, '\n');
        if (pos) {
            *pos = '\0';
        }
        char* token = strtok(line, " ");
        if(strcmp(token, "SAP") == 0){
            ESP_LOGI(TAG, "SAP mode read");
            return flag;
        }
        else if(strcmp(token, "STA") == 0){
            flag = 1;
            ESP_LOGI(TAG, "STA mode read");
            token = strtok(NULL, " ");
            strcpy(EXAMPLE_ESP_WIFI_SSID, token);
            token = strtok(NULL, " ");
            strcpy(EXAMPLE_ESP_WIFI_PASS, token);
            return flag;
        }
        return flag;
    }
}

char determine(int flag)
{
    char c;
    switch(flag){
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

/*    strcat(ptr, "<p>Press to choose connection mode</p><a class=\"button button-on\" href=\"/choose\">CHOOSE</a>\n");
    strcat(ptr, "<p>Press to select manual mode</p><a class=\"button button-on\" href=\"/manual\">MANUAL</a>\n");*/
    strcat(ptr, "<p>Press to select Manual mode</p><a class=\"button button-on\" href=\"/manual\">MANUAL</a>\n");
    strcat(ptr, "<p>Press to select Auto mode</p><a class=\"button button-on\" href=\"/auto\">AUTO</a>\n");
    strcat(ptr, "<p>Press to choose Connection Mode</p><a class=\"button button-on\" href=\"/choose\">SAP/STA</a>\n");

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
    strcat(ptr, "<p>Click to return to home page</p><a class=\"button button-on\" href=\"/\">ON</a>\n");

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

char* get_form()
{
    char* ptr = (char *)calloc(2048, sizeof(char));
    strcat(ptr, "<form action=\"/data\" method = \"post\">\n");
    strcat(ptr, "<label for=\"ssid\">SSID:</label><br>\n");
    strcat(ptr, "<input type=\"text\" id=\"ssid\" name=\"ssid\"><br>\n");
    strcat(ptr, "<label for=\"pwd\">Password:</label><br>\n");
    strcat(ptr, "<input type=\"password\" id=\"pwd\" name=\"pwd\"><br><br>\n");
    strcat(ptr, "<input type=\"submit\" value=\"Submit\">\n");
    strcat(ptr, "<input type=\"reset\">\n");
    strcat(ptr, "</form>");
    return ptr;
}

char* auto_mode()
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
    strcat(ptr, "<p>Press to return to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");
    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
}

char* manual_mode()
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

    if(record_flag == 0)
        strcat(ptr, "<p>Click to Start</p><a class=\"button button-on\" href=\"/start\">START</a>\n");
    else
        strcat(ptr, "<p>Click to Stop</p><a class=\"button button-on\" href=\"/stop\">STOP</a>\n");
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

char* SendHTML(uint8_t flag)
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

    if(record_flag == 0)
        strcat(ptr, "<p>Click to Start</p><a class=\"button button-on\" href=\"/start\">START</a>\n");
    else
        strcat(ptr, "<p>Click to Stop</p><a class=\"button button-on\" href=\"/stop\">STOP</a>\n");

    if(flag==0)
    {strcat(ptr, "<p>Forward: ON</p><a class=\"button button-off\" href=\"/manual\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");}

    if(flag==1)
    {strcat(ptr, "<p>Left: ON</p><a class=\"button button-off\" href=\"/manual\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");}

    if(flag==2)
    {strcat(ptr, "<p>Right: ON</p><a class=\"button button-off\" href=\"/manual\">OFF</a>\n");}
    else
    {strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");}

    if(flag==3)
    {strcat(ptr, "<p>Backwards: ON</p><a class=\"button button-off\" href=\"/manual\">OFF</a>\n");}
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
    strcat(ptr, "<p>Press to continue to home</p><a class=\"button button-on\" href=\"/\">HOME</a>\n");

    strcat(ptr, "</body>\n");
    strcat(ptr, "</html>\n");
    return ptr;
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
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t handle_OnConnect(httpd_req_t *req)
{
    flag = -1;
    char* resp = default_page();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_start(httpd_req_t *req)
{
    record_flag = 1;
    flag = -1;
    FILE* f = fopen("/spiffs/path.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "File opened for writing");
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    fclose(f);
    char* resp = manual_mode();
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
    char* resp = manual_mode();
    prev_mili = esp_timer_get_time();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_auto(httpd_req_t *req)
{
    flag = 4;
    char* resp = auto_mode();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    char ch;
    FILE* f = fopen("/spiffs/path.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    do 
    {
        ch = fgetc(f);
        ESP_LOGI(TAG, "%c", ch);

    }while(ch != '\0');
    fclose(f);
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
        FILE* f = fopen("/spiffs/path.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%f", time_duration);
        fputc('\n', f);
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
        FILE* f = fopen("/spiffs/path.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%f", time_duration);
        fputc('\n', f);
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
        FILE* f = fopen("/spiffs/path.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%f", time_duration);
        fputc('\n', f);
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
        FILE* f = fopen("/spiffs/path.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%f", time_duration);
        fputc('\n', f);
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
        FILE* f = fopen("/spiffs/path.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);
        fprintf(f, "%f", time_duration);
        fputc('\n', f);
        fputc('\0', f);
        fclose(f);
    }
    char* resp = get_stop();
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
    FILE* fp = fopen("/spiffs/wifi_conf.txt", "w");
    if (fp == NULL) {
            ESP_LOGE(TAG, "Failed to handle sap for reading");
            esp_restart();
            return flag;
        }
    fputs("SAP\n", fp);
    fclose(fp);
    ESP_LOGI(TAG, "SAP Data Written");
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

esp_err_t handle_sta(httpd_req_t *req)
{
    char* resp = get_form();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    return ESP_OK;
}

esp_err_t handle_data(httpd_req_t *req)
{
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
    //char* copy = (char *)calloc(strlen(buf), sizeof(char));
    ESP_LOGI(TAG, "Buffer: %s", buf);
    strcpy(copy, buf);
    ESP_LOGI(TAG, "Copy: %s", copy);
    char* token = strtok(buf, "&");
    char* ssid = strtok(token, "=");
    ssid = strtok(NULL, "=");
    char* new_token = strtok(copy, "&");
    new_token = strtok(NULL, "&");
    char* pwd = strtok(new_token, "=");
    pwd = strtok(NULL, "=");
    FILE* fp = fopen("/spiffs/wifi_conf.txt", "w");
    if (fp == NULL) {
            ESP_LOGE(TAG, "Failed to handle sta for reading");
            esp_restart();
            return flag;
        }
    fputs("STA ", fp);
    fputs(ssid, fp);
    fputc(' ', fp);
    fputs(pwd, fp);
    fputc('\n', fp);
    fclose(fp);
    ESP_LOGI(TAG, "STA Data Written");
    httpd_resp_send(req, "Device will restart using STA mode", strlen("Device will restart using STA mode"));
    vTaskDelay(100);
    esp_restart();
    return ESP_OK;
}

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

httpd_uri_t uri_auto = {
    .uri      = "/auto",
    .method   = HTTP_GET,
    .handler  = handle_auto,
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

httpd_uri_t uri_data = {
    .uri      = "/data",
    .method   = HTTP_POST,
    .handler  = handle_data,
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

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.max_uri_handlers = 20;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_home);
        httpd_register_uri_handler(server, &uri_manual);
        httpd_register_uri_handler(server, &uri_auto);
        httpd_register_uri_handler(server, &uri_forward);
        httpd_register_uri_handler(server, &uri_left);
        httpd_register_uri_handler(server, &uri_right);
        httpd_register_uri_handler(server, &uri_back);
        httpd_register_uri_handler(server, &uri_choose);
        httpd_register_uri_handler(server, &uri_sap);
        httpd_register_uri_handler(server, &uri_sta);
        httpd_register_uri_handler(server, &uri_data);
        httpd_register_uri_handler(server, &uri_start);
        httpd_register_uri_handler(server, &uri_stop);
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

void app_main(void)
{
    static httpd_handle_t server = NULL;
    init_spiffs();
    conn_flag = conn_type();
    if(conn_flag == 0)
        init_sap();
    else
        wifi_init_sta();
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp32"));
    ESP_LOGI(TAG, "mdns hostname set to: [esp32]");
    ESP_ERROR_CHECK(mdns_instance_name_set("web_server"));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();
}