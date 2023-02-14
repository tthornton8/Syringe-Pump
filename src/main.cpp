#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LITTLEFS.h> 
#include <AccelStepper.h>

const double degrees_per_step = 1.8;
const int microsteps          = 32;
const int pitch               = 1;
const int dir_pin             = 25;
const int step_pin            = 26; 
const char* ssid              = "electrospinning";
const char* password          = "electrospinning";

AccelStepper stepper(AccelStepper::FULL4WIRE, step_pin, dir_pin);
AsyncWebServer server(80);

double diameter = 10; 

int calcSteps(double volume) {
  double area   = 3.14159265359*diameter*diameter/4; //mm^2
  double length = 1e3*volume / area; // mm
  double revs   = length / pitch; // ul
  double steps  = (revs * 360.0) / degrees_per_step; // ul
  
  int microsteps_request = ceil(steps*microsteps);
  return microsteps_request;
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String processor(const String& var) {
  if (var == "PLACEHOLDER_VOLUME") {
    double steps  = stepper.currentPosition()/microsteps; // ul
    double revs   = steps * degrees_per_step/360; // ul
    double length = revs * pitch; // mm
    double area   = 3.14159265359*diameter*diameter/4; //mm^2
    double volume = length * area / 1e3;

    return String(volume, 2);
  } else {
    return String();
  }
}

void setup() { 
  stepper.setMaxSpeed(100.0);
  stepper.setAcceleration(100.0);
  stepper.setCurrentPosition(0);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  LittleFS.begin();

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", String(), false, processor);
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

    if (dirRequestString.compareTo("pull")) {
      microsteps_request =- microsteps_request;
    }

    stepper.setSpeed(speed);
    stepper.move(microsteps_request);
  });

  // stop the syringe
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stepper.stop();
  });
}

void loop() {
  stepper.run();
}