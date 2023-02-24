#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h> 
#include <AccelStepper.h>
#include <AsyncElegantOTA.h>
#include <TMCSTepper.h>
#include <HardwareSerial.h>

const double degrees_per_step = 1.8;
const double microsteps       = 256.;
const double pitch            = 0.792;
const int dir_pin1            = 23;
const int step_pin1           = 22; 
const int en_pin1             = 2;
const int dir_pin2            = 21;
const int step_pin2           = 19; 
const int en_pin2             = 18;
const int endstop1            = 27;
const int endstop2            = 26;
const char* ssid              = "electrospinning";
const char* password          = "electrospinning";

#define SERIAL_PORT2 Serial2
#define SERIAL_PORT1 Serial
#define R_SENSE 0.11f

TMC2208Stepper driver1(&SERIAL_PORT1, R_SENSE);   
TMC2208Stepper driver2(&SERIAL_PORT2, R_SENSE);   
AccelStepper stepper2(AccelStepper::DRIVER, step_pin2, dir_pin2);
AccelStepper stepper1(AccelStepper::DRIVER, step_pin1, dir_pin1);
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

void setupDriver(TMC2208Stepper& driver, AccelStepper& stepper, int EN_PIN) {
  Serial2.begin(115200);     // SW UART drivers
  driver.begin();             // Initiate pins and registeries
  driver.toff(5);
  driver.rms_current(1200);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autoscale(true);
  driver.microsteps(256);

  stepper.setMaxSpeed(10000); // 100mm/s @ 80 steps/mm
  stepper.setAcceleration(10000); // 2000mm/s^2
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, false);
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
}

void setup() { 
  pinMode(step_pin1, OUTPUT); //Step pin as output
  pinMode(dir_pin1,  OUTPUT); //Direcction pin as output
  pinMode(step_pin2, OUTPUT); //Step pin as output
  pinMode(dir_pin2,  OUTPUT); //Direcction pin as output
  pinMode(endstop1,  INPUT); //Direcction pin as input
  pinMode(endstop2,  INPUT); //Direcction pin as input

  setupDriver(driver1, stepper1, en_pin1);
  setupDriver(driver2, stepper2, en_pin2);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  Serial1.begin(9600);
  Serial1.print("AP IP address: ");
  Serial1.println(IP);

  // Print ESP32Local IP Address
  Serial1.println(WiFi.localIP());

  if(!SPIFFS.begin()){
    Serial1.println("An Error has occurred while mounting SPIFFS");
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
    long steps = -stepper1.currentPosition();
    long rate  = stepper1.speed();
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

    Serial1.println(microsteps_request);
    Serial1.println(speed);
    
    if (dirRequestString.compareTo("pull")) {
      microsteps_request =- microsteps_request;
    }

    stepper1.move(microsteps_request);
    stepper1.setSpeed(speed);

    request->send(SPIFFS, "/index.html", String(), false);
  });

  // stop the syringe
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper1.stop();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // reset volume
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper1.setCurrentPosition(0);
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

bool shaft = false;
void loop() {
  stepper1.runSpeedToPosition();
  stepper2.runSpeedToPosition();

  if (stepper2.distanceToGo() == 0) {
        delay(100);
        stepper2.move(100*1000); // Move 100mm
        shaft = !shaft;
        driver2.shaft(shaft);
    }
    stepper2.run();
}