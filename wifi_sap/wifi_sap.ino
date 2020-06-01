#include <WiFi.h>
#include <WebServer.h>

/* Put your SSID & Password */
const char* ssid = "ESP32";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,43,36);
IPAddress gateway(192,168,43,36);
IPAddress subnet(255,255,255,0);

WebServer server(80);

uint8_t LED1pin = 2;
int flag = -1;
int prev_flag = -1;
float t, curr, prev;
char c;

void setup() {
  Serial.begin(115200);
  pinMode(LED1pin, OUTPUT);

  WiFi.softAP(ssid, password);
  delay(1000);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  
  server.on("/", handle_OnConnect);
  server.on("/memorize_path", handle_manual);
  server.on("/auto", handle_auto);
  server.on("/forward", handle_forward);
  server.on("/left", handle_left);
  server.on("/right", handle_right);
  server.on("/back", handle_back);
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  server.handleClient();
  if(flag==0){
    if(prev_flag!=flag)
    {
      if(prev_flag==-1)
        prev = 0;
      curr = millis();
      t = curr - prev;
      switch(prev_flag){
      case 0: c = 'F';break;
      case 1: c = 'L';break;
      case 2: c = 'R';break;
      case 3: c = 'B';break;
      default: c = 's';break;}
      Serial.println(c+String(t));
      prev = curr;
      prev_flag = flag;
    }
   }
  else if(flag==1){
    if(prev_flag!=flag)
    {
      if(prev_flag==-1)
        prev = 0;
      curr = millis();
      t = curr - prev;
      switch(prev_flag){
      case 0: c = 'F';break;
      case 1: c = 'L';break;
      case 2: c = 'R';break;
      case 3: c = 'B';break;
      default: c = 's';break;}
      Serial.println(c+String(t));
      prev = curr;
      prev_flag = flag;
    }
  }
  else if(flag==2){
    if(prev_flag!=flag)
    {
      if(prev_flag==-1)
        prev = 0;
      curr = millis();
      t = curr - prev;
      switch(prev_flag){
      case 0: c = 'F';break;
      case 1: c = 'L';break;
      case 2: c = 'R';break;
      case 3: c = 'B';break;
      default: c = 's';break;}
      Serial.println(c+String(t));
      prev = curr;
      prev_flag = flag;
    }
  }
  else if(flag==3){
    if(prev_flag!=flag)
    {
      if(prev_flag==-1)
        prev = 0;
      curr = millis();
      t = curr - prev;
      switch(prev_flag){
      case 0: c = 'F';break;
      case 1: c = 'L';break;
      case 2: c = 'R';break;
      case 3: c = 'B';break;
      default: c = 's';break;}
      Serial.println(c+String(t));
      prev = curr;
      prev_flag = flag;
    }
  }
  else if(flag==4)
    Serial.println("A");
}

void handle_OnConnect() {
  flag = -1;
  Serial.println("HomePage");
  server.send(200, "text/html", default_page()); 
}

void handle_manual() {
  flag = -1;
  Serial.println("manual homepage");
  server.send(200, "text/html", manual_mode()); 
}

void handle_forward() {
  flag = 0;
  Serial.println("moving forward");
  server.send(200, "text/html", SendHTML(flag)); 
}

void handle_left() {
  flag = 1;
  Serial.println("turning left");
  server.send(200, "text/html", SendHTML(flag)); 
}

void handle_right() {
  flag = 2;
  Serial.println("turning right");
  server.send(200, "text/html", SendHTML(flag)); 
}

void handle_back() {
  flag = 3;
  Serial.println("moving backwards");
  server.send(200, "text/html", SendHTML(flag)); 
}

void handle_auto() {
  flag = 4;
  Serial.println("GPIO4 Status: OFF");
  server.send(200, "text/html", auto_mode()); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String default_page(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>Choose Direction</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
  ptr +="<p>Press to select manual mode</p><a class=\"button button-on\" href=\"/memorize_path\">MANUAL</a>\n";
  ptr +="<p>Press to select auto mode</p><a class=\"button button-on\" href=\"/\">AUTO</a>\n";
  
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

String auto_mode(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>Choose Direction</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

String manual_mode(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>Choose Direction</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
  ptr +="<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n";
  ptr +="<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n";
  ptr +="<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n";
  ptr +="<p>Back: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n";

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

String SendHTML(uint8_t flag){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>Choose Direction</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-on {background-color: #3498db;}\n";
  ptr +=".button-on:active {background-color: #2980b9;}\n";
  ptr +=".button-off {background-color: #34495e;}\n";
  ptr +=".button-off:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>ESP32 Web Server</h1>\n";
  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
  
  if(flag==0)
  {ptr +="<p>Forward: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n";}
  else
  {ptr +="<p>Forward: OFF</p><a class=\"button button-on\" href=\"/forward\">ON</a>\n";}

  if(flag==1)
  {ptr +="<p>Left: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n";}
  else
  {ptr +="<p>Left: OFF</p><a class=\"button button-on\" href=\"/left\">ON</a>\n";}

  if(flag==2)
  {ptr +="<p>Right: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n";}
  else
  {ptr +="<p>Right: OFF</p><a class=\"button button-on\" href=\"/right\">ON</a>\n";}

  if(flag==3)
  {ptr +="<p>Backwards: ON</p><a class=\"button button-off\" href=\"/memorize_path\">OFF</a>\n";}
  else
  {ptr +="<p>Backwards: OFF</p><a class=\"button button-on\" href=\"/back\">ON</a>\n";}


  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}
