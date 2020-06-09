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
#define EXAMPLE_ESP_WIFI_SSID      "AndroidAP1"
#define EXAMPLE_ESP_WIFI_PASS      "ltgj0084"
#define EXAMPLE_ESP_MAXIMUM_RETRY  3

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";
static int flag = -1;
static int s_retry_num = 0;
static int64_t prev_mili = 0;
static int64_t curr_mili = 0;
static float time_duration = 0;

char determine(int flag)
{
	char c;
	switch(flag){
		case -1: c = 's';break;
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
	strcat(ptr, ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
	strcat(ptr, ".button-on {background-color: #3498db;}\n");
	strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
	strcat(ptr, ".button-off {background-color: #34495e;}\n");
	strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
	strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
	strcat(ptr, "</style>\n");
	strcat(ptr, "</head>\n");
	strcat(ptr, "<body>\n");
	strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
	strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");

	strcat(ptr, "<p>Press to select manual mode</p><a class=\"button button-on\" href=\"/memorize_path\">MANUAL</a>\n");
	strcat(ptr, "<p>Press to select auto mode</p><a class=\"button button-on\" href=\"/\">AUTO</a>\n");

	strcat(ptr, "</body>\n");
	strcat(ptr, "</html>\n");
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
	strcat(ptr, ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
	strcat(ptr, ".button-on {background-color: #3498db;}\n");
	strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
	strcat(ptr, ".button-off {background-color: #34495e;}\n");
	strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
	strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
	strcat(ptr, "</style>\n");
	strcat(ptr, "</head>\n");
	strcat(ptr, "<body>\n");
	strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
	strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");

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
	strcat(ptr, ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
	strcat(ptr, ".button-on {background-color: #3498db;}\n");
	strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
	strcat(ptr, ".button-off {background-color: #34495e;}\n");
	strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
	strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
	strcat(ptr, "</style>\n");
	strcat(ptr, "</head>\n");
	strcat(ptr, "<body>\n");
	strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
	strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");

	strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");
	strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");
	strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");
	strcat(ptr, "<p>Back: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n");

	strcat(ptr, "</body>\n");
	strcat(ptr, "</html>\n");
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
	strcat(ptr, ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n");
	strcat(ptr, ".button-on {background-color: #3498db;}\n");
	strcat(ptr, ".button-on:active {background-color: #2980b9;}\n");
	strcat(ptr, ".button-off {background-color: #34495e;}\n");
	strcat(ptr, ".button-off:active {background-color: #2c3e50;}\n");
	strcat(ptr, "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");
	strcat(ptr, "</style>\n");
	strcat(ptr, "</head>\n");
	strcat(ptr, "<body>\n");
	strcat(ptr, "<h1>ESP32 Web Server</h1>\n");
	strcat(ptr, "<h3>Using Access Point(AP) Mode</h3>\n");

	if(flag==0)
	{strcat(ptr, "<p>Forward: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n");}
	else
	{strcat(ptr, "<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n");}

	if(flag==1)
	{strcat(ptr, "<p>Left: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n");}
	else
	{strcat(ptr, "<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n");}

	if(flag==2)
	{strcat(ptr, "<p>Right: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n");}
	else
	{strcat(ptr, "<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n");}

	if(flag==3)
	{strcat(ptr, "<p>Backwards: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n");}
	else
	{strcat(ptr, "<p>Backwards: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n");}


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
            .ssid = "myssid",
            .ssid_len = strlen("myssid"),
            .password = "qwerty1234",
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:myssid password:qwerty1234");
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
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t handle_OnConnect(httpd_req_t *req)
{
	flag = -1;
	char* resp = default_page();
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

esp_err_t handle_manual(httpd_req_t *req)
{
	flag = -1;
	prev_mili = esp_timer_get_time();
	curr_mili = esp_timer_get_time();
	char* resp = manual_mode();
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

esp_err_t handle_auto(httpd_req_t *req)
{
	flag = 4;
	char* resp = auto_mode();
    httpd_resp_send(req, resp, strlen(resp));
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
    return ESP_OK;
}

httpd_uri_t uri_home = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = handle_OnConnect,
    .user_ctx = NULL
};

httpd_uri_t uri_manual = {
    .uri      = "/memorize_path",
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

httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_home);
        httpd_register_uri_handler(server, &uri_manual);
        httpd_register_uri_handler(server, &uri_auto);
        httpd_register_uri_handler(server, &uri_forward);
        httpd_register_uri_handler(server, &uri_left);
        httpd_register_uri_handler(server, &uri_right);
        httpd_register_uri_handler(server, &uri_back);
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
    ESP_ERROR_CHECK(nvs_flash_init());
	wifi_init_sta();
//	init_sap();
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();
}