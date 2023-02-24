#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h> 
#include <AccelStepper.h>
#include <AsyncElegantOTA.h>
#include <TMCSTepper.h>

const double degrees_per_step = 1.8;
const double microsteps       = 32.;
const double pitch            = 0.792;
const int dir_pin1            = 23;
const int step_pin1           = 22; 
const int dir_pin2            = 5;
const int step_pin2           = 18; 
const int endstop1            = 27;
const int endstop2            = 26;
const char* ssid              = "electrospinning";
const char* password          = "electrospinning";

AccelStepper stepper(AccelStepper::DRIVER, step_pin1, dir_pin1);
AsyncWebServer server(80);

double diameter = 15; 

double calcVolume(long steps) {
  double micro_steps  = double(steps) / double(microsteps); // ul
  double revs   = micro_steps * degrees_per_step/360.; // ul
  double length = revs * pitch; // mm
  double area   = 3.14159265359*diameter*diameter/4.; //mm^2
  double volume = length * area / 1e2;

  return volume;
}

double calcSpeed(long rate) {
  double speed = calcVolume(rate);

  return speed;
}

int calcSteps(double volume) {
  double area   = 3.14159265359*diameter*diameter/4.; //mm^2
  double length = 1e2*volume / area; // mm
  double revs   = length / pitch; // ul
  double steps  = (revs * 360.0) / degrees_per_step; // ul
  
  int microsteps_request = ceil(steps*microsteps);
  return microsteps_request;
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() { 
  pinMode(step_pin1, OUTPUT); //Step pin as output
  pinMode(dir_pin1,  OUTPUT); //Direcction pin as output
  pinMode(endstop1,  INPUT); //Direcction pin as input
  pinMode(endstop2,  INPUT); //Direcction pin as input


  stepper.setMaxSpeed(1000000.0);
  stepper.setAcceleration(100000.0);
  stepper.setCurrentPosition(0);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  Serial.begin(9600);
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Print ESP32Local IP Address
  Serial.println(WiFi.localIP());

  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Return motor position when requested
  server.on("/motor_pos", HTTP_GET, [](AsyncWebServerRequest *request){
    long steps = -stepper.currentPosition();
    long rate  = stepper.speed();
    double volume = calcVolume(steps)*1000.;
    double speed  = calcSpeed(rate)*3600.;

    request->send(200, "application/json", "{\"motorPos\": " + String(volume, 2) + ",\"speed\": " + String(speed, 2) + "}"); 
  });

  // Speed / volume control
  server.on("/run", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String volRequestString   = request->getParam("volume")->value();
    String timeRequestString  = request->getParam("time")->value();
    String dirRequestString   = request->getParam("direction")->value();
    String diameterString     = request->getParam("diameter")->value();

    double volRequest  = volRequestString.toDouble();
    double timeRequest = timeRequestString.toDouble();
    diameter = diameterString.toDouble();

    int microsteps_request = calcSteps(volRequest);
    double speed = microsteps_request / timeRequest;

    Serial.println(microsteps_request);
    Serial.println(speed);
    
    if (dirRequestString.compareTo("pull")) {
      microsteps_request =- microsteps_request;
    }

    stepper.move(microsteps_request);
    stepper.setSpeed(speed);

    request->send(SPIFFS, "/index.html", String(), false);
  });

  // stop the syringe
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper.stop();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // reset volume
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper.setCurrentPosition(0);
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // JS Scripts
  server.on("/js/jquery.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/jquery.js", "text/javascript");
  });

  server.on("/js/jquery-ui.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery-ui.js", "text/javascript");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

}

void loop() {
  stepper.runSpeedToPosition();
  // Serial.println(digitalRead(endstop1));
  // Serial.println(digitalRead(endstop2));
  // Serial.println();
}