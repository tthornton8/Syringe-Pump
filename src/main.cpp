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
#define Serial_debug Serial1
#define R_SENSE 0.11f

TMC2208Stepper driver1(&SERIAL_PORT1, R_SENSE);   
TMC2208Stepper driver2(&SERIAL_PORT2, R_SENSE);   
AccelStepper stepper2(AccelStepper::DRIVER, step_pin2, dir_pin2);
AccelStepper stepper1(AccelStepper::DRIVER, step_pin1, dir_pin1);
AsyncWebServer server(80);

// AccelStepper* stepper[] = {&stepper1, &stepper2};
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
  stepper.setPinsInverted(true, false, true);
  stepper.enableOutputs();
  stepper.setCurrentPosition(0);
  stepper.setSpeed(0);
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

  Serial_debug.begin(9600);
  Serial_debug.print("AP IP address: ");
  Serial_debug.println(IP);

  // Print ESP32Local IP Address
  Serial.println(WiFi.localIP());

  if(!SPIFFS.begin()){
    Serial_debug.println("An Error has occurred while mounting SPIFFS");
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
    long steps1 = -stepper1.currentPosition();
    long rate1  = stepper1.speed();
    long steps2 = -stepper2.currentPosition();
    long rate2  = stepper2.speed();

    double volume1 = calcVolume(steps1)*1000.;
    double volume2 = calcVolume(steps2)*1000.;
    double speed1  = calcSpeed(rate1)*3600.;
    double speed2  = calcSpeed(rate2)*3600.;

    request->send(200, "application/json", "{\"motorPos1\": " + String(volume1, 2) + ",\"speed1\": " + String(speed1, 2) + ",\"motorPos2\": " + String(volume2, 2) + ",\"speed2\": " + String(speed2, 2) + "}"); 
  });

  // Speed / volume control
  server.on("/run", HTTP_GET, [] (AsyncWebServerRequest *request) {
    for (int i = 1; i <= 2; i++) {
      AccelStepper* stepper;

      if (i==1) {
        stepper = &stepper1;
      } else {
        stepper = &stepper2;
      }

      String volRequestString   = request->getParam("volume" + String(i))->value();
      String timeRequestString  = request->getParam("time" + String(i))->value();
      String dirRequestString   = request->getParam("direction" + String(i))->value();
      String diameterString     = request->getParam("diameter" + String(i))->value();

      Serial_debug.println(volRequestString);
      Serial_debug.println(timeRequestString);
      Serial_debug.println(dirRequestString);
      Serial_debug.println(diameterString);

      double volRequest  = volRequestString.toDouble();
      double timeRequest = timeRequestString.toDouble();
      diameter = diameterString.toDouble();

      Serial_debug.println(volRequest);
      Serial_debug.println(timeRequest);
      Serial_debug.println(diameter);

      if (volRequest*timeRequest*diameter == 0.) {
        continue;
      }

      int microsteps_request = calcSteps(volRequest);
      double speed = microsteps_request / timeRequest;

      Serial_debug.println(microsteps_request);
      Serial_debug.println(speed);
      Serial_debug.println();
      
      if (dirRequestString.compareTo("pull")) {
        microsteps_request =- microsteps_request;
      }

      stepper->move(microsteps_request);
      stepper->setSpeed(speed);

    }
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // stop the syringe
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper1.stop();
    stepper2.stop();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // reset volume
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
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
  stepper1.runSpeedToPosition();
  stepper2.runSpeedToPosition();

  if (stepper1.distanceToGo() == 0) {
    stepper1.setSpeed(0);
  }

  if (stepper2.distanceToGo() == 0) {
    stepper2.setSpeed(0);
  }
}