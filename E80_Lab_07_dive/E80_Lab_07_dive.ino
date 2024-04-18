/********
E80 Lab 7 Dive Activity Code
Authors:
    Omar Aleman (oaleman@g.hmc.edu) '21 (contributed 2019)
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
    Caden Perry (caperry@hmc.edu) '26 (contributed in 2024)                    
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <XYStateEstimator.h>
#include <ZStateEstimator.h>
#include <ADCSampler.h>
#include <ErrorFlagSampler.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <DepthControl.h>
#define UartSerial Serial1
#include <GPSLockLED.h>
#include <BurstADCSampler.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
XYStateEstimator xy_state_estimator;
ZStateEstimator z_state_estimator;
DepthControl depth_control;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ADCSampler adc;
ErrorFlagSampler ef;
ButtonSampler button_sampler;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;
BurstADCSampler burst_adc;

// loop start recorder
int loopStartTime;
int currentTime;
volatile bool EF_States[NUM_FLAGS] = {1,1,1};

const int outputPin = 24;
const int lowblue = 27;
const int highwhite = 25;
const int max_length = 4.6952;
bool fsr_crash = false;

//blue is low - A13/pin 27
//white is high - A11/pin 25

////////////////////////* Setup *////////////////////////////////

void setup() {
  
  logger.include(&imu);
  logger.include(&gps);
  logger.include(&xy_state_estimator);
  logger.include(&z_state_estimator);
  logger.include(&depth_control);
  logger.include(&motor_driver);
  logger.include(&adc);
  logger.include(&ef);
  logger.include(&button_sampler);
  logger.init();

  printer.init();
  ef.init();
  button_sampler.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();
  burst_adc.init();

  int diveDelay = 2000; // how long robot will stay at depth waypoint before continuing (ms)

  const int num_depth_waypoints = 3;
  double depth_waypoints [] = {0.5, 2.5, max_length};  // listed as z0,z1,... etc.
  depth_control.init(num_depth_waypoints, depth_waypoints, diveDelay);
  
  xy_state_estimator.init(); 
  z_state_estimator.init();

  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
  printer.lastExecutionTime            = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET ;
  imu.lastExecutionTime                = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  adc.lastExecutionTime                = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  ef.lastExecutionTime                 = loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
  button_sampler.lastExecutionTime     = loopStartTime - LOOP_PERIOD + BUTTON_LOOP_OFFSET;
  xy_state_estimator.lastExecutionTime = loopStartTime - LOOP_PERIOD + XY_STATE_ESTIMATOR_LOOP_OFFSET;
  z_state_estimator.lastExecutionTime  = loopStartTime - LOOP_PERIOD + Z_STATE_ESTIMATOR_LOOP_OFFSET;
  depth_control.lastExecutionTime      = loopStartTime - LOOP_PERIOD + DEPTH_CONTROL_LOOP_OFFSET;
  logger.lastExecutionTime             = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;

}



//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime=millis();
    
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,adc.printSample());
    printer.printValue(1,ef.printStates());
    printer.printValue(2,logger.printState());
    printer.printValue(3,gps.printState());   
    printer.printValue(4,xy_state_estimator.printState());  
    printer.printValue(5,z_state_estimator.printState());  
    printer.printValue(6,depth_control.printWaypointUpdate());
    printer.printValue(7,depth_control.printString());
    printer.printValue(8,motor_driver.printState());
    printer.printValue(9,imu.printRollPitchHeading());        
    printer.printValue(10,imu.printAccels());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  /* ROBOT CONTROL Finite State Machine */
  if ( currentTime-depth_control.lastExecutionTime > LOOP_PERIOD ) {
    if ((analogRead(A1) * 3.3 / 1024) < 1.5){
      fsr_crash = true;
    }
    // Serial.println("At 1st if statement");
    depth_control.lastExecutionTime = currentTime;
    if ( depth_control.diveState ) {      // DIVE STATE //
      depth_control.complete = false;
      // Serial.println("At 2nd if statement");
      if ( !depth_control.atDepth && !fsr_crash) {
        depth_control.dive(&z_state_estimator.state, currentTime);
        // Serial.println("At 3rd if statement");
      }
      else {
        depth_control.diveState = false; 
        depth_control.surfaceState = true;
        // Serial.println("At 1st else statement");
      }
      //motor_driver.drive(0,0,255);
      //motor_driver.drive(0,0,depth_control.uV);
      if (depth_control.uV > 0){
        digitalWrite (highwhite , HIGH);
        digitalWrite (lowblue , LOW);      
      }

      else if (depth_control.uV < 0){
        digitalWrite (highwhite , LOW);
        digitalWrite (lowblue , HIGH);
      }

      else {
        digitalWrite (highwhite , LOW);
        digitalWrite (lowblue , LOW);
      }

      
      // Serial.println("Motor is driving");
    }
    if ( depth_control.surfaceState ) {     // SURFACE STATE //
      // Serial.println("At 4th if statement");
      if ( !depth_control.atSurface ) { 
        // Serial.println("At 5th if statement");
        depth_control.surface(&z_state_estimator.state);
      }
      else if ( depth_control.complete ) { 
        // Serial.println("At 1st else if statement");
        delete[] depth_control.wayPoints;   // destroy depth waypoint array from the Heap
      }
      //motor_driver.drive(0,0,-255);
     // motor_driver.drive(0,0,depth_control.uV);
      if (depth_control.uV > 0){
        digitalWrite (highwhite , HIGH);
        digitalWrite (lowblue , LOW);      
      }

      else if (depth_control.uV < 0){
        digitalWrite (highwhite , LOW);
        digitalWrite (lowblue , HIGH);
      }

      else {
        digitalWrite (highwhite , LOW);
        digitalWrite (lowblue , LOW);
      }
    }
  }
  
  if ( currentTime-adc.lastExecutionTime > LOOP_PERIOD ) {
    adc.lastExecutionTime = currentTime;
    adc.updateSample(); 
  }

  // Expiremental burst pin sampling
// samples at around 7400Hz every 30 seconds
// stops motors and waits 2 seconds before burst sample
  if ( currentTime-burst_adc.lastExecutionTime > 30000 ) {
    burst_adc.lastExecutionTime = currentTime;
    // motor_driver.drive(0,0,0);
    delay(500);
    Serial.print("Sampling\n");
    analogWrite(outputPin, 128); // Set the PWM duty cycle to 50% (analogWrite values range from 0 to 255)
    delayMicroseconds(125);
    analogWrite(outputPin, 0);
    burst_adc.sample();
    Serial.print("done\n");
  }

  if ( currentTime-ef.lastExecutionTime > LOOP_PERIOD ) {
    ef.lastExecutionTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
    attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
    delay(5);
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
    detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
    ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
    EF_States[0] = 1;
    EF_States[1] = 1;
    EF_States[2] = 1;
  }

  if ( currentTime-button_sampler.lastExecutionTime > LOOP_PERIOD ) {
    button_sampler.lastExecutionTime = currentTime;
    button_sampler.updateState();
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read();     // blocking I2C calls
  }
 
  gps.read(&GPS); // blocking UART calls, need to check for UART data every cycle

  if ( currentTime-xy_state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    xy_state_estimator.lastExecutionTime = currentTime;
    xy_state_estimator.updateState(&imu.state, &gps.state);
  }

  if ( currentTime-z_state_estimator.lastExecutionTime > LOOP_PERIOD ) {
    z_state_estimator.lastExecutionTime = currentTime;
    z_state_estimator.updateState(analogRead(PRESSURE_PIN));
  }
  
  if ( currentTime-led.lastExecutionTime > LOOP_PERIOD ) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if ( currentTime- logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging ) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}

void EFA_Detected(void){
  EF_States[0] = 0;
}

void EFB_Detected(void){
  EF_States[1] = 0;
}

void EFC_Detected(void){
  EF_States[2] = 0;
}