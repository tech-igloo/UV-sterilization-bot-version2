#include "server.h"
#include "algo.h"

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

/*In all of the callback functions below, the HTML Code for displaying
  the webpage is passed using the 'resp' string variable*/

/*Callback function whenever "/" is accessed*/
esp_err_t handle_OnConnect(httpd_req_t *req)
{
    manual_flag = 0;
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
    manual_flag = 0;
    flag = -1;       //-1 denotes stop

    gpio_intr_enable(LEFT_ENCODERA);    //Enabled when recording start
    gpio_intr_enable(RIGHT_ENCODERA);

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
    manual_flag = 1; //only when in manual mode, set to 0 again if start(recording) is pressed
    record_flag = 0; //recording has not yet started
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    
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
    flag = 4;
    //auto_flag = 1; //activates only when execute is pressed
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
    
    /*Code when interrupts are implemented*/
    /* 
    if(flag == 0 || flag == 3){                      //for forward and backward
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead as well
    }
    else if(flag == 1 || flag == 2){                  //to calculated angle
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead here as well, this gives us the arc length
        time_duration = (time_duration*2)/wheelbase;  //angle= arc/radius gives angle in radians
        time_duration = time_duration%(2*M_2_PI)      //if more than 2pi then take the remainder
    }
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    */
    init_pid();
    flag = 0;                       //denotes that is now going in forward direction

    char* resp = SendHTML(flag);	//the HTML code for displaying the webpage
    httpd_resp_send(req, resp, strlen(resp));	//display the webpage
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1 && det != 's')        //if the path is recording
    {
        //move_forward();
        FILE* f = fopen("/spiffs/paths.txt", "a");		//Open the File for writing the value
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f);  //for storing the previous direction
        fprintf(f, "%.3f", time_duration);  //for storing the previous time duration -- this should altered according to the encoder data
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
    /*Code when interrupts are implemented*/
    /* 
    if(flag == 0 || flag == 3){                      //for forward and backward
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead as well
    }
    else if(flag == 1 || flag == 2){                  //to calculated angle
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead here as well, this gives us the arc length
        time_duration = (time_duration*2)/wheelbase;  //angle= arc/radius gives angle in radians
        time_duration = time_duration%(2*M_2_PI)
    }
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    */
    init_pid();
    flag = 1;                       //denotes that is now going in left direction

    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1 && det != 's')        //if the path is recording
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
    /*Code when interrupts are implemented*/
    /* 
    if(flag == 0 || flag == 3){                      //for forward and backward
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead as well
    }
    else if(flag == 1 || flag == 2){                  //to calculated angle
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead here as well, this gives us the arc length
        time_duration = (time_duration*2)/wheelbase;  //angle= arc/radius gives angle in radians
        time_duration = time_duration%(2*M_2_PI)
    }
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    */    
    init_pid();
    flag = 2;               //Now going in right direction, everything else is same as previous
    
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1 && det != 's')
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
    curr_mili = esp_timer_get_time();  //In micro seconds
    time_duration = (curr_mili - prev_mili)/1000.0;      //Converting in milli
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    prev_mili = esp_timer_get_time();
    /*Code when interrupts are implemented*/
    /* 
    if(flag == 0 || flag == 3){                      //for forward and backward
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead as well
    }
    else if(flag == 1 || flag == 2){                  //to calculated angle
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead here as well, this gives us the arc length
        time_duration = (time_duration*2)/wheelbase;  //angle= arc/radius gives angle in radians
        time_duration = time_duration%(2*M_2_PI)
    }
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    */    
    init_pid();
    flag = 3;       //Now going in back direction, everything else same as forward and left and right
 
    char* resp = SendHTML(flag);
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1 && det != 's')
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
    /*Code when interrupts are implemented*/
    /* 
    if(flag == 0 || flag == 3){                      //for forward and backward
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead as well
    }
    else if(flag == 1 || flag == 2){                  //to calculated angle
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead here as well, this gives us the arc length
        time_duration = (time_duration*2)/wheelbase;  //angle= arc/radius gives angle in radians
        time_duration = time_duration%(2*M_2_PI)
    }
    else
        time_duration = 0;                          //special case for stop
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    */
    
    flag = -1;   //Stop the bot as well
    init_pid();

    gpio_intr_disable(LEFT_ENCODERA);  //Disabled interrupt once recorded  
    gpio_intr_disable(RIGHT_ENCODERA);

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
    /*Code when interrupts are implemented*/
    /* 
    if(flag == 0 || flag == 3){                      //for forward and backward
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead as well
    }
    else if(flag == 1 || flag == 2){                  //to calculated angle
        time_duration = (abs(leftRot)*ENCODERresolution + abs(leftTicks))*wheeldist_perTick;  //Could have checked right instead here as well, this gives us the arc length
        time_duration = (time_duration*2)/wheelbase;  //angle= arc/radius gives angle in radians
        time_duration = time_duration%(2*M_2_PI)
    }
    ESP_LOGI(TAG,"%c%f",det,time_duration);
    leftRot = 0;     //Advantage of incremental encoders
    leftTicks = 0;
    rightRot = 0;
    rightTicks = 0;
    */
    
    flag = -1;
    init_pid();

    char* resp = manual_mode();
    httpd_resp_send(req, resp, strlen(resp));
    free(resp);
    ESP_LOGI(TAG, "Record Flag: %d", record_flag);
    if(record_flag == 1 && det != 's') //If recording is ongoing, then store the direction in which it was travelling and the time duration for whcih it had travelled in that direction
    {
        FILE* f = fopen("/spiffs/paths.txt", "a");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return ESP_FAIL;
        }
        fputc(det, f); //Store the character denoting direction
        fprintf(f, "%.3f", time_duration); //Store the time duration //with encoders it'll be distance/angle
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
    ESP_LOGI(TAG, "Now total paths---------------- %d", total_paths);
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
