#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h> 
#include <AccelStepper.h>
#include <AsyncElegantOTA.h>
#include <TMCSTepper.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>

#define RXD1 12
#define TXD1 13
#define RXD2 16
#define TXD2 15

HardwareSerial SerialPort2 (2);   // This is the key line missing.
// HardwareSerial SerialPort2 (2);   // This is the key line missing.


const double degrees_per_step = 1.8;
const int microsteps          = 256;
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

#define SERIAL_PORT2 SerialPort2
#define SERIAL_PORT1 Serial1
#define Serial_debug Serial
#define R_SENSE 0.11f

TMC2208Stepper driver1(&SERIAL_PORT1, R_SENSE);   
TMC2208Stepper driver2(&SERIAL_PORT2, R_SENSE);   
AccelStepper stepper2(AccelStepper::DRIVER, step_pin2, dir_pin2);
AccelStepper stepper1(AccelStepper::DRIVER, step_pin1, dir_pin1);
AsyncWebServer server(80);    // SW UART drivers
Scheduler ts;

void resetMS() {
  driver1.microsteps(microsteps);
  driver2.microsteps(microsteps);
}

Task resetDriver (1000*TASK_MILLISECOND, TASK_FOREVER, &resetMS, &ts);

// AccelStepper* stepper[] = {&stepper1, &stepper2};
double diameter = 15; 
byte resetFlag = false;

double calcVolume(double steps) {
  double micro_steps  = double(steps) / double(microsteps); // ul
  double revs   = micro_steps * degrees_per_step/360.; // ul
  double length = revs * pitch; // mm
  double area   = 3.14159265359*diameter*diameter/4.; //mm^2
  double volume = length * area / 1e2;

  return volume;
}

double calcSpeed(double rate) {
  double speed = calcVolume(rate);

  return speed;
}

int calcSteps(double volume) {
  double area   = 3.14159265359*diameter*diameter/4.; //mm^2
  double length = 1e2*volume / area; // mm
  double revs   = length / pitch; // ul
  double steps  = (revs * 360.0) / degrees_per_step; // ul
  
  int microsteps_request = ceil(steps*double(microsteps));
  return microsteps_request;
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setupDriver(TMC2208Stepper& driver, AccelStepper& stepper, int EN_PIN, bool dir_invert) {
  driver.begin();             // Initiate pins and registeries
  driver.toff(5);
  driver.rms_current(1200);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver.pwm_autoscale(true);
  driver.microsteps(microsteps);

  stepper.setMaxSpeed(500000); 
  stepper.setAcceleration(10000); 
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(dir_invert, false, true);
  stepper.enableOutputs();
  stepper.setSpeed(0);
}

void setup() { 
  pinMode(step_pin1, OUTPUT); //Step pin as output
  pinMode(dir_pin1,  OUTPUT); //Direcction pin as output
  pinMode(step_pin2, OUTPUT); //Step pin as output
  pinMode(dir_pin2,  OUTPUT); //Direcction pin as output
  pinMode(endstop1,  INPUT); //Direcction pin as input
  pinMode(endstop2,  INPUT); //Direcction pin as input

  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  SerialPort2.begin(115200, SERIAL_8N1, 16, 17);
  
  setupDriver(driver1, stepper1, en_pin1, false);
  setupDriver(driver2, stepper2, en_pin2, false);
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  Serial_debug.begin(9600);
  Serial_debug.print("AP IP address: ");
  Serial_debug.println(IP);

  // Print ESP32Local IP Address
  Serial.println(WiFi.localIP());

  ts.startNow();

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
    char ms1[80];
    char ms2[80];
    
    long steps1   = -stepper1.currentPosition();
    double rate1  = stepper1.speed();
    long steps2   = -stepper2.currentPosition();
    double rate2  = stepper2.speed();

    double volume1 = calcVolume(steps1)*1000.;
    double volume2 = calcVolume(steps2)*1000.;
    double speed1  = calcSpeed(rate1)*3600.;
    double speed2  = calcSpeed(rate2)*3600.;

    sprintf(ms1, "%u", driver1.microsteps());
    sprintf(ms2, "%u", driver2.microsteps());

    request->send(200, "application/json", "{\"motorPos1\": " + String(volume1, 2) + 
                                            ",\"speed1\": " + String(speed1, 4) + 
                                            ",\"rate1\": " + String(rate1, 4) + 
                                            ",\"motorPos2\": " + String(volume2, 2) + 
                                            ",\"speed2\": " + String(speed2, 4) + 
                                            ",\"rate2\": " + String(rate2, 4) + 
                                            ",\"ms1\": " + String(ms1, 3) + 
                                            ",\"ms2\": " + String(ms2, 3) + 
                                            "}"); 
  });

  // Speed / volume control
  server.on("/run", HTTP_GET, [] (AsyncWebServerRequest *request) {
    setupDriver(driver1, stepper1, en_pin1, false);
    setupDriver(driver2, stepper2, en_pin2, false);
    resetFlag = false;

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

      // Serial_debug.println(volRequestString);
      // Serial_debug.println(timeRequestString);
      // Serial_debug.println(dirRequestString);
      // Serial_debug.println(diameterString);

      double volRequest  = volRequestString.toDouble();
      double timeRequest = timeRequestString.toDouble();
      diameter = diameterString.toDouble();

      // Serial_debug.println(volRequest);
      // Serial_debug.println(timeRequest);
      // Serial_debug.println(diameter);

      if (volRequest*timeRequest*diameter == 0.) {
        continue;
      }

      int microsteps_request = calcSteps(volRequest);
      double speed = microsteps_request / timeRequest;

      // Serial_debug.println(microsteps_request);
      // Serial_debug.println(speed);
      // Serial_debug.println();
      
      if (dirRequestString.compareTo("pull")) {
        microsteps_request =- microsteps_request;
      }

      stepper->enableOutputs();
      stepper->move(microsteps_request);
      stepper->setSpeed(speed);

    }
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // stop the syringe
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    resetFlag = false;

    stepper1.stop();
    stepper2.stop();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // stop the syringe
  server.on("/motorsOff", HTTP_GET, [](AsyncWebServerRequest *request){
    resetFlag = false;

    stepper1.disableOutputs();
    stepper2.disableOutputs();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // reset volume
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    if (resetFlag) {
      ESP.restart();
    } else {
      stepper1.enableOutputs();
      stepper2.enableOutputs();
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      request->send(SPIFFS, "/index.html", String(), false);

      resetFlag = true;
    }
  });

  // JS Scripts
  server.on("/jquery.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/jquery.js", "text/javascript");
  });

  server.on("/jquery-ui.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery-ui.js", "text/javascript");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
}

unsigned long last_set_ms = micros();
void loop() {
  stepper1.runSpeedToPosition();
  stepper2.runSpeedToPosition();

  if (stepper1.distanceToGo() == 0) {
    stepper1.setSpeed(0);
  }

  if (stepper2.distanceToGo() == 0) {
    stepper2.setSpeed(0);
  }

  ts.execute();
  
}