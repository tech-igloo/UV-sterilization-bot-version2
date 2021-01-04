#include "server.h"
// /*The variables that keep track of various things while the code is running*/
static EventGroupHandle_t s_wifi_event_group;   //Used for Wifi connections during SAP and STA mode
const char *TAG = "wifi station";        //Name used for ESP_LOGI statements. Feel free to set it to whatever you want
int flag = -1;                           //Used for determining in which direction it is travelling
static int s_retry_num = 0;                     //Number of times the esp has tried to connect to a network in STA mode
int64_t prev_mili = 0;                   //Used for determining the time duration between 2 commands while controlling the bot in manual mode
int64_t curr_mili = 0;                   //Used for determining the time duration between 2 commands while controlling the bot in manual mode
float time_duration = 0;                 //Used for storing the time duration between 2 commands while controlling the bot in manual mode
// static char buf[DATA_LEN];                      //Used for extracting ssid from POST header data whenever User stores new Network Credentials
// static char copy[DATA_LEN];                     //Used for extracting password from POST header data whenevr User stores new Network Credentials
int conn_flag = 0;                       //Denote which Wifi mode the ESP is used : 0 = SAP, 1 to WIFI_NUM = STA wifi index
int record_flag = 0;                     //Denote whether path is being recorded or not
// static char ssid[WIFI_NUM][SSID_LEN];           //Store all the network ssids that the ESP can use in STA mode
// static char pass[WIFI_NUM][PASS_LEN];           //Store all the network passwords that the ESP can use in STA mode
static char EXAMPLE_ESP_WIFI_SSID[SSID_LEN];    //Store the network ssid that the esp is using currently in STA mode
static char EXAMPLE_ESP_WIFI_PASS[PASS_LEN];    //Store the network password that the esp is using currently in STA mode
int total = 0;                           //Total number of network credentials for STA mode stored till now
// static char line_str[LINE_LEN];                 //Used for extracting lines from stored files
int total_paths = 0;                     //Total number of paths stored till now
int auto_flag = 0;                       //Denote whether auto mode is on or off
static TaskHandle_t Task1;                      //Task handle to keep track of created task running on Core 1

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

