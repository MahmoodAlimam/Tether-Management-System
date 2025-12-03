#include <CircularBuffer.hpp>
#include <TaskScheduler.h>
#include <Servo.h>
#include "commands.h"
#include "fuzzy.h"

#define BUF_SIZE 10

//Create the scheduler
Scheduler runner;

//Create a buffer for the message received from ESP32
CircularBuffer<String,BUF_SIZE> queue;

//Create servo objects
Servo thrustFrntBck;  
Servo thrustLftRght;
Servo rotary;

//Roll angle with its limit
volatile float roll= 0;
const int rollAngleLimitmin = 80;
const int rollAngleLimitmax = 100;
float xAcc, yAcc, zAcc;

void taskRunCommand_callback();
void taskRollControl_callback();

//Global parameters
static String usblStreamcommands;
static int cmdSeq = 0;


//List of tasks
Task taskRunCommand(5000, TASK_FOREVER, &taskRunCommand_callback, &runner, true);
Task taskRollControl(0,TASK_FOREVER, &taskRollControl_callback, &runner, false);

//Setup of the system
void setup() 
{
// Set delay for 1s
//Serial.begin(9600);
delay(1000);
Serial1.begin(9600);

// Attach the Nano IoT to the specified pin.
thrustFrntBck.attach(THRUSTER_FrntBck_PIN);
thrustLftRght.attach(THRUSTER_LftRght_PIN);
rotary.attach(ROTARYACTUATOR_PIN);

//Initialization
//Initialize thrusters
initializeThrusters(thrustFrntBck, thrustLftRght);
  
//Initialize rotary actuator
initializeActuator(rotary);

// Initialize the IMU (accelerometer)
  if (!IMU.begin()) 
  {
   Serial.println("Failed to initialize IMU!");
   while (1); // Halt if IMU fails
  }

Serial.print("Accelerometer sample rate = ");
Serial.print(IMU.accelerationSampleRate());
Serial.println(" Hz");  
}

void loop() 
{
  IMU.readAcceleration(xAcc, yAcc, zAcc);
  roll = atan2(yAcc, zAcc) * 180.0 / PI;
  //Serial.println(roll);
  // Decide what task to start
    //if (roll < rollAngleLimitmin || roll > rollAngleLimitmax)
    if(roll > -40 || roll < -140)  //80 and 100
    {
      taskRunCommand.disable();
      taskRollControl.enable();
    }
     else 
    {
      taskRunCommand.enable();
      taskRollControl.disable();
    }
    runner.execute();
  
}

void taskRunCommand_callback()
{
 String cmdField[2];
 int re;
    
    // Read the message from NanoESP32
    
    if(Serial1.available())
    {
      String receivedMessage = Serial1.readStringUntil('\n');  

      if (receivedMessage.length()>0)
      {
        Serial.println("receivedMessage and will be pushed to queue");
        queue.unshift(receivedMessage);
      }
    }   
  
    //Process received command
    while(!queue.isEmpty())
   {
    usblStreamcommands = queue.pop();
    Serial.print("usbl Command =");
    Serial.println(usblStreamcommands);
    re = parseUSBLMessage(cmdField, usblStreamcommands);
    handleCommand(thrustFrntBck, thrustLftRght, rotary, angle, cmdField);
   }   

}

void taskRollControl_callback()
{
    // For this example, we'll use yAcc (forward/backward tilt) as input to fuzzy logic
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    float r = atan2(yAcc, zAcc) * 180.0 / PI;
    Serial.print("START FUZZY, with roll angle =");
    Serial.println(r);
    //Orientation of the IMU is 90 degree 
    float accelInput = r;

    Serial.print("Y-Acc: ");
    Serial.print(accelInput, 3); // Print with 3 decimal places

    // --- Fuzzy Logic Processing ---
    // 1. Fuzzification: Calculate membership degrees for each fuzzy set
    float muNL = getMembership(accelInput, ACCEL_RANGE_MIN, ACCEL_NL_PEAK, ACCEL_NS_PEAK); // NL: (min, peak, next_peak)
    float muNS = getMembership(accelInput, ACCEL_NL_PEAK, ACCEL_NS_PEAK, ACCEL_ZERO_PEAK); // NS: (prev_peak, peak, next_peak)
    float muZ  = getMembership(accelInput, ACCEL_NS_PEAK, ACCEL_ZERO_PEAK, ACCEL_PS_PEAK); // Z:  (prev_peak, peak, next_peak)
    float muPS = getMembership(accelInput, ACCEL_ZERO_PEAK, ACCEL_PS_PEAK, ACCEL_PL_PEAK); // PS: (prev_peak, peak, next_peak)
    float muPL = getMembership(accelInput, ACCEL_PS_PEAK, ACCEL_PL_PEAK, ACCEL_RANGE_MAX); // PL: (prev_peak, peak, max)

    // Serial.print(" | NL:"); Serial.print(muNL, 2);
    // Serial.print(" NS:"); Serial.print(muNS, 2);
    // Serial.print(" Z:"); Serial.print(muZ, 2);
    // Serial.print(" PS:"); Serial.print(muPS, 2);
    // Serial.print(" PL:"); Serial.print(muPL, 2);

    // 2. Rule Evaluation (Simple Mamdani-style, one rule per input fuzzy set)
    // The strength of each output fuzzy set is simply the membership degree of the corresponding input fuzzy set.
    float strengthFR = muPL; // If Accel is PL, then Motor is FR
    float strengthSR = muPS; // If Accel is PS, then Motor is SR
    float strengthST = muZ;  // If Accel is Z,  then Motor is ST
    float strengthSF = muNS; // If Accel is NS, then Motor is SF
    float strengthFF = muNL; // If Accel is NL, then Motor is FF

    // 3. Defuzzification (Weighted Average / Center of Gravity approximation)
    // Calculate the crisp output motor command
    float crispMotorCommand = (strengthFF * MOTOR_FF +
                               strengthSF * MOTOR_SF +
                               strengthST * MOTOR_ST +
                               strengthSR * MOTOR_SR +
                               strengthFR * MOTOR_FR) /
                              (strengthFF + strengthSF + strengthST + strengthSR + strengthFR);

    // Handle division by zero if all strengths are zero (e.g., input outside defined range)
    if (isnan(crispMotorCommand)) 
    {
      crispMotorCommand = 0.0; // Default to stop
    }

    Serial.print(" | Motor Command: ");
    Serial.print(crispMotorCommand, 3);

    // 4. Map the crisp motor command (-1.0 to 1.0) to ESC pulse width (MIN_PULSE to MAX_PULSE)
    int targetPulse = mapFloat(crispMotorCommand, -1.0, 1.0, MIN_PULSE, MAX_PULSE);

    Serial.print(" | Target Pulse: ");
    Serial.println(targetPulse);

    // Set the motor speed
    setMotorPulse(thrustLftRght, targetPulse);
  

    delay(50); // Small delay for stable readings and motor updates
    Serial.println("Start a new task");
}

