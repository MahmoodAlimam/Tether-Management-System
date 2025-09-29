#include <TaskScheduler.h>
#include <Servo.h>
#include "commands.h"
#include "fuzzy.h"

//Create the scheduler
Scheduler runner;

//Create servo objects
Servo thrustFrntBck;  
Servo thrustLftRght;
Servo rotary;

//Roll angle with its limit
volatile int angle= 0;
const int rollAngleLimit = 20;

void taskRunCommand_callback();
void taskRollControl_callback();

//Global command message
static String usblStreamcommands[]={"$FWD,1700","$STP,1500","$BWD,1300","$STP,1500","$LFT,1700","$STP,1500","$RGT,1300","$STP,1500"};
static int cmdSeq = 0;

//List of tasks
Task taskRunCommand(8000, TASK_FOREVER, &taskRunCommand_callback, &runner, true);
Task taskRollControl(0,TASK_ONCE, &taskRollControl_callback, &runner, false);

//Setup of the system
void setup() 
{
// Set delay for 1s
Serial.begin(9600);
delay(1000);
while(!Serial);

// Attach the Nano IoT to the specified pin.
thrustFrntBck.attach(THRUSTER_FrntBck_PIN);
thrustLftRght.attach(THRUSTER_LftRght_PIN);


//Initialization
//Initialize thrusters
initializeThrusters(thrustFrntBck, thrustLftRght);
  
//Initialize rotary actuator
initializeActuator(rotary);
}

void loop() {
  // put your main code here, to run repeatedly:
    if (angle > rollAngleLimit && !taskRollControl.isEnabled())
    {
      taskRunCommand.disable();
      taskRollControl.enable();
    }
    runner.execute();
}

void taskRunCommand_callback()
{
 String cmdField[2];
 int re;
   
   while(cmdSeq < 8)
   {
    Serial.print("Command sequence = "); Serial.println(cmdSeq);
    re = parseUSBLMessage(cmdField, usblStreamcommands[cmdSeq]);
    handleCommand(thrustFrntBck, thrustLftRght, rotary, cmdField);
    cmdSeq++;
   }
   cmdSeq = 0;

}

void taskRollControl_callback()
{

if(angle < rollAngleLimit)
  {
    taskRunCommand.enable();
  }
}
