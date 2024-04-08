#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <Pinouts.h>
#include <TimingOffsets.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <XYStateEstimator.h>
#include <ADCSampler.h>
#include <ErrorFlagSampler.h>
#include <ButtonSampler.h> // A template of a data source library
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>
#include <SurfaceControl.h>
#define UartSerial Serial1
#include <GPSLockLED.h>

/* FSR testing sketch.
Connect one end of FSR to power, the other end to Analog 0. Then connect one end of a 10K resistor from Analog 0 to ground
For more information see www.ladyada.net/learn/sensors/fsr.html */
int fsrPin = 14;
int fsrReading;
int fsrVoltage;
unsigned long fsrResistance; // The voltage converted to resistance, can be very big so make "long"
motor = HIGH;
crash.state = false;
void setup(void) {
Serial.begin(9600); // We'll send debugging information via the Serial monitor
}
void loop(void) {
fsrReading = analogRead(fsrPin); Serial.print("Analog reading = "); Serial.println(fsrReading);
while (crash.state != true) {
  motor = HIGH;
  if (fsrReading > 0.1) {
    motor = LOW;
  }
} else{
  motor = LOW;
}
}

