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
#include <ESPmDNS.h>

// serial pins
#define RXD1 12
#define TXD1 13
#define RXD2 16
#define TXD2 17

// define hardware serial port
HardwareSerial SerialPort2 (2);   

TaskHandle_t ResetStepperTask;

// constants 
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

// set serial ports to use for steppers
#define SERIAL_PORT2 SerialPort2
#define SERIAL_PORT1 Serial1
#define Serial_debug Serial
#define R_SENSE 0.11f

// declare function to use it later in a task
void resetMS();

// stepper driver and motor objects
TMC2208Stepper driver1(&SERIAL_PORT1, R_SENSE);   
TMC2208Stepper driver2(&SERIAL_PORT2, R_SENSE);   
AccelStepper stepper2(AccelStepper::DRIVER, step_pin2, dir_pin2);
AccelStepper stepper1(AccelStepper::DRIVER, step_pin1, dir_pin1);

// web server
AsyncWebServer server(80);  


// global variables
volatile double diameter = 15; 
volatile byte resetFlag = false;
volatile byte motorsOff = false;

// count the number of digits in a number
int count_digit(int number) {
   return int(log10(number) + 1);
}

// calculate pumped volume from steps
double calcVolume(double steps) {
  double micro_steps  = double(steps) / double(microsteps); // ul
  double revs   = micro_steps * degrees_per_step/360.; // ul
  double length = revs * pitch; // mm
  double area   = 3.14159265359*diameter*diameter/4.; //mm^2
  double volume = length * area / 1e2;

  return volume;
}

// calculate pump rate from step rate
double calcSpeed(double rate) {
  double speed = calcVolume(rate);

  return speed;
}

// calculate required steps to pump the volume
int calcSteps(double volume) {
  double area   = 3.14159265359*diameter*diameter/4.; //mm^2
  double length = 1e2*volume / area; // mm
  double revs   = length / pitch; // ul
  double steps  = (revs * 360.0) / degrees_per_step; // ul
  
  int microsteps_request = ceil(steps*double(microsteps));
  return microsteps_request;
}

// handle webserver 404 error
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// set up the stepper motor drivers to use UART and the right microsteps/current
void setupDriver(TMC2208Stepper& driver, AccelStepper& stepper, int EN_PIN, bool dir_invert) {
  driver.begin();                 // Initiate pins and registeries
  driver.toff(5);                 // Smoothly turn drivers on/off
  driver.rms_current(1200);       // Set stepper current to 12000mA. 
  driver.pwm_autoscale(true);     // Enable PWM automatic amplitude scaling
  driver.microsteps(microsteps);  // Set to microstepping
  stepper.setMaxSpeed(500000);    // Max speed 500000 steps/s
  stepper.setAcceleration(10000); // Max accel 10000 steps/s^2

  // Setup control pins
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(dir_invert, false, true);

  // don't accdentally toggle motor state
  if (motorsOff) {
    stepper.disableOutputs();
  } else {
    stepper.enableOutputs();
  }
  // stepper.setSpeed(0);
}

// checks the driver microstep setting and resets it if nessecary (needed for HV effects).
// will stop the motors if this keeps failing
volatile bool stopflag = false;
void resetMS() {
  uint16_t ms1 = driver1.microsteps();
  uint16_t ms2 = driver2.microsteps();

  if ((ms1 != microsteps) or (ms2 != microsteps)) {
    // onyl reset driver if needed
    setupDriver(driver1, stepper1, en_pin1, false);
    setupDriver(driver2, stepper2, en_pin2, false);

    if (stopflag) {
      // stop if not set properly
      stepper1.stop();
      stepper2.stop();
    }
    stopflag = true;

  } else {
    stopflag = false;
  }
}

// run the steppers to the requested position at the requested speed
void runSteppers() {
  stepper1.runSpeedToPosition();
  stepper2.runSpeedToPosition();

  if (stepper1.distanceToGo() == 0) {
    stepper1.setSpeed(0);
  }

  if (stepper2.distanceToGo() == 0) {
    stepper2.setSpeed(0);
  }
}

// task to check the stepper drivers and reset them if needed every 100 mS
void ResetStepperTaskCode( void * pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100; //delay for mS

  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    resetMS();
  }
}

void setup() { 
  pinMode(step_pin1, OUTPUT); //Step pin as output
  pinMode(dir_pin1,  OUTPUT); //Direcction pin as output
  pinMode(step_pin2, OUTPUT); //Step pin as output
  pinMode(dir_pin2,  OUTPUT); //Direcction pin as output
  pinMode(endstop1,  INPUT); //Direcction pin as input
  pinMode(endstop2,  INPUT); //Direcction pin as input

  // make the serial ports work on the right pins
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  SerialPort2.begin(115200, SERIAL_8N1, 16, 17);
  
  // set up motors
  setupDriver(driver1, stepper1, en_pin1, false);
  setupDriver(driver2, stepper2, en_pin2, false);
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  // set up wifi
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  // Print IP address to serial port for debugging
  Serial_debug.begin(9600);
  Serial_debug.print("AP IP address: ");
  Serial_debug.println(IP);
  Serial_debug.println(WiFi.localIP());

  // start multicast DNS on domain name http://electrospinning.local
  if(!MDNS.begin("electrospinning")) {
     Serial.println("Error starting mDNS");
     return;
  }

  // start filesystem 
  if(!SPIFFS.begin()){
    Serial_debug.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // create the stepper motor reset task
  xTaskCreate(
    ResetStepperTaskCode,
    "ResetStepperTask",
    10000,
    NULL,
    5,
    &ResetStepperTask
  );

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to load favicon.ico file
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/favicon.ico", "image/ico");
  });

  // Return motor position when requested
  server.on("/motor_pos", HTTP_GET, [](AsyncWebServerRequest *request){
    char ms1[80];
    char ms2[80];
    
    long steps1   = -stepper1.currentPosition();
    double rate1  = stepper1.speed();
    long target1  = stepper1.distanceToGo();
    long steps2   = -stepper2.currentPosition();
    double rate2  = stepper2.speed();
    long target2  = stepper2.distanceToGo();
    //1164 3682

    double volume1    = calcVolume(steps1)*1000.;
    double volume2    = calcVolume(steps2)*1000.;
    double remaining1 = calcVolume(target1)*1000.;
    double remaining2 = calcVolume(target2)*1000.;
    double speed1     = calcSpeed(rate1)*3600.;
    double speed2     = calcSpeed(rate2)*3600.;

    uint16_t ms1_res = driver1.microsteps();
    uint16_t ms2_res = driver2.microsteps();

    sprintf(ms1, "%u", ms1_res);
    sprintf(ms2, "%u", ms2_res);

    String JSON = String("{\"motorPos1\": " + String(volume1, 2) + 
                          ",\"speed1\": " + String(speed1, 4) + 
                          ",\"rate1\": " + String(rate1, 4) + 
                          ",\"motorPos2\": " + String(volume2, 2) + 
                          ",\"remaining1\": " + String(remaining1, 4) + 
                          ",\"remaining2\": " + String(remaining2, 4) + 
                          ",\"speed2\": " + String(speed2, 4) + 
                          ",\"rate2\": " + String(rate2, 4) + 
                          ",\"ms1\": " + String(ms1, count_digit(ms1_res)) + 
                          ",\"ms2\": " + String(ms2, count_digit(ms2_res)) + 
                          "}");

    request->send(200, "application/json", JSON); 
    resetMS();

    });

  // Speed control
  server.on("/run", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // turn the steppers and set up the drivers
    motorsOff = false;
    resetFlag = false;
    setupDriver(driver1, stepper1, en_pin1, false);
    setupDriver(driver2, stepper2, en_pin2, false);

    // loop over each stepper
    for (int i = 1; i <= 2; i++) {
      AccelStepper* stepper;

      if (i==1) {
        stepper = &stepper1;
      } else {
        stepper = &stepper2;
      }

      // get parameters from web request
      String volRequestString   = request->getParam("volume" + String(i))->value();
      String timeRequestString  = request->getParam("time" + String(i))->value();
      String dirRequestString   = request->getParam("direction" + String(i))->value();
      String diameterString     = request->getParam("diameter" + String(i))->value();

      // turn the parameters into numbers
      double volRequest  = volRequestString.toDouble();
      double timeRequest = timeRequestString.toDouble();
      diameter = diameterString.toDouble();

      // if one of the parameters was blank, ignore that stepper motor
      if (volRequest*timeRequest*diameter == 0.) {
        continue;
      }

      // calcualte required speed and microsteps
      int microsteps_request = calcSteps(volRequest);
      double speed = microsteps_request / timeRequest;
      
      // calculate motor direction
      if (dirRequestString.compareTo("pull")) {
        microsteps_request =- microsteps_request;
      }

      // set the stepper target position and speed
      stepper->enableOutputs();
      stepper->move(microsteps_request);
      stepper->setSpeed(speed);

    }

    // send back the webpage as a response to the request
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // stop the motors
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    resetFlag = false;

    stepper1.stop();
    stepper2.stop();
    request->send(SPIFFS, "/index.html", String(), false);
  });

  // disable the motors
  server.on("/motorsOff", HTTP_GET, [](AsyncWebServerRequest *request){
    resetFlag = false;

    stepper1.disableOutputs();
    stepper2.disableOutputs();

    motorsOff = true;
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

  // JS JQuery Scripts
  server.on("/jquery.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/jquery.js", "text/javascript");
  });

  server.on("/jquery-ui.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery-ui.js", "text/javascript");
  });


  // set up the web server
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
}

// run the steppers each loop (uses default RTOS task)
void loop() {
  runSteppers();
}