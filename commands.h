#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>  
#include <Servo.h>

// Define the pin for the ESC signal.  Make sure this is a PWM-capable pin.
#define THRUSTER_FrntBck_PIN 6  //  Any PWM pin should work (e.g., 2, 4, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33)
#define THRUSTER_LftRght_PIN 5
#define ROTARYACTUATOR_PIN 7  //it was 9


#define S0 12  // Connect S0 pin of TCS3200 to Arduino Digital Pin 2
#define S1 11  // Connect S1 pin of TCS3200 to Arduino Digital Pin 3
#define S2 10 // Connect S2 pin of TCS3200 to Arduino Digital Pin 4
#define S3 9  // Connect S3 pin of TCS3200 to Arduino Digital Pin 5
#define OUT 8 // Connect OUT pin of TCS3200 to Arduino Digital Pin 6


void initializeThrusters(Servo, Servo);
void initializeActuator(Servo);
void handleCommand(Servo, Servo, Servo, String*);
//String getField(String, int);

void resetSystem(void);

int parseUSBLMessage(String*, String);
uint8_t computeCRC8(const char*, size_t);

void moveForward(Servo, int);
void moveBackward(Servo, int);
void moveLeft(Servo, Servo, Servo, int);
void moveRight(Servo, Servo, Servo, int);
void stopThrust(Servo, Servo, Servo);
void configureCamera(void);
int readColorFrequency(int, int);
#endif