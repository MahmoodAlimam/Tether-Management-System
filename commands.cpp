#include "commands.h"
#include <Servo.h>

void configureCamera()
{
// Set the S0, S1, S2, S3 pins as OUTPUTs to control the sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Set the OUT pin as an INPUT to read the frequency from the sensor
  pinMode(OUT, INPUT);

  digitalWrite(S0, HIGH); // Set S0 HIGH
  digitalWrite(S1, LOW);  // Set S1 LOW

}
void initializeThrusters(Servo s1, Servo s2)
{
  s1.writeMicroseconds(1000);   // Send the minimum throttle signal
  s2.writeMicroseconds(1000);
  delay(4000);                               
  s1.writeMicroseconds(1500);
  s2.writeMicroseconds(1500);   // Send the neutral signal
  delay(2000);
  Serial.println("ESC armed.");
  delay(1000);
  Serial.println("Thrusters initialized");
}

void initializeActuator(Servo r)
{
  //r.attach(7);
  Serial.println("Starting actuator test...");

  // Move to 0 degrees
  r.write(0);
  Serial.println("Moved to 0°");
  delay(2000);

  // Move to 90 degrees
  r.write(90);
  Serial.println("Moved to 90°");
  delay(2000);

  // Move to 180 degrees
  r.write(180);
  Serial.println("Moved to 180°");
  delay(2000);

  // Return to center
  r.write(90);
  Serial.println("Returned to 90°");
  Serial.println("Actuator initialized");
}

void handleCommand(Servo thrust1, Servo thrust2, Servo actuate, String* msg) 
{  
  Serial.print("Command =");Serial.println(msg[0]);
  if(msg[0] == "FWD")
  {
   Serial.println("Match FWD");
   moveForward(thrust1, msg[1].toInt());
  }
  else if(msg[0]=="BWD")
  {
   Serial.println("Match BWD"); 
   moveBackward(thrust1, msg[1].toInt());
  }
  else if(msg[0]=="LFT")
  {
   Serial.println("Match LFT");
   moveLeft(thrust1, thrust2, actuate, msg[1].toInt());
  }
  else if(msg[0]=="RGT")
  {
    Serial.println("Match RGT");
    moveRight(thrust1, thrust2, actuate, msg[1].toInt());
  }
  else if(msg[0]=="STP")
  {
    Serial.println("Match STP");
    stopThrust(thrust1, thrust2, actuate);
  }
  else
  {
    Serial.println("command not recognized");
  }
  
}

// String getField(String msg, int index) {
//   int start = 0;
//   for (int i = 0; i < index; i++) 
//    {
//     start = msg.indexOf(',', start);
//     if (start == -1) return "";
//     start++;
//    }
//   int end = msg.indexOf(',', start);
//   if (end == -1) 
//    {
//      end = msg.length();
//    }
//   return msg.substring(start, end);
// }


int parseUSBLMessage(String* fields, String msg) {
  
  int fieldCount = 0;
  int startIdx = 0;
  // Remove any newline or carriage return
  Serial.println("Parsing ...");
  msg.trim();

  // Basic sanity check
  if (!msg.startsWith("$")) {
    Serial.println("Invalid message header.");
    return 1;
  }
  
  // Strip off checksum if present
  //int asteriskIdx = msg.indexOf('*');
  
  //if (asteriskIdx == -1) {
    //Serial.println("No checksum found");
    //return 1;
  //}

  //extract the message and checksum
  //String crcString = msg.substring(asteriskIdx+1, asteriskIdx+3);
  msg = msg.substring(1, msg.length());

  //Convert string checksum into integer
  //uint8_t crcInt = strtoul(crcString.c_str(), NULL, 16);

  //Compute CRC
  //uint8_t crcCalculate = computeCRC8(msg.c_str(), msg.length());
  //Serial.print("Extracted crc = "); Serial.println(crcInt);
  //Serial.print("Calculated crc = "); Serial.println(crcCalculate);
  //Coparison of CRCs
  //if (crcInt == crcCalculate) {
  //    Serial.println("✅ CRC OK — Valid command.");  
  //  } 
  //  else 
  //  {
  //    Serial.println("❌ CRC mismatch."); 
  //    return 1;
  //  }

  // Split fields by comma
  for (int i = 0; i < msg.length(); i++) {
    if (msg.charAt(i) == ',' || i == msg.length() - 1) {
      int endIdx = (i == msg.length() - 1) ? i + 1 : i;
      fields[fieldCount++] = msg.substring(startIdx, endIdx);
      startIdx = i + 1;
    }
  }

  if (fieldCount < 1) {
    //Serial.println("Incomplete message.");
    return 1;
  }

  // Extract values
  int period = fields[1].toInt();
  
  Serial.print("Command: "); Serial.print(fields[0]); Serial.println(" ");
  Serial.print("Thruster time period: "); Serial.print(period); Serial.println(" ");
  return 0;
}

void moveForward(Servo thruster, int pwmThrust)
{
  if (pwmThrust > 1700) 
  { 
    pwmThrust = 1700;  // Constrain maximum speed
  }

  thruster.writeMicroseconds(pwmThrust);
  Serial.print("Moving Forward");
  Serial.println(pwmThrust);
}

void moveBackward(Servo thruster, int pwmThrust)
{
  if (pwmThrust < 1300) 
  { 
    pwmThrust = 1300;  // Constrain minimum speed
  }
  
  thruster.writeMicroseconds(pwmThrust);
  Serial.print("Moving Backward");
  Serial.println(pwmThrust);
}

void moveLeft(Servo thruster1, Servo thruster2, Servo actuator, int pwmThrust)
{
  if (pwmThrust > 1700) 
  { 
    pwmThrust = 1700;  // Constrain minimum speed
  }
  
  // Move to 90 degrees
  // actuator.write(0);
  // delay(2000);
  // actuator.write(90);
  // Serial.println("Moving left");
  // //Serial.println("Moved to 90°");
  // delay(2000);

for (int angle = 0; angle <= 90; angle += 10) {
    actuator.write(angle);
    delay(100); } 

  //Run thrusters
  thruster1.writeMicroseconds(pwmThrust);
  thruster2.writeMicroseconds(pwmThrust);
  //Display message
  Serial.print("time period for left movement is set to: ");
  Serial.println(pwmThrust);
}

void moveRight(Servo thruster1, Servo thruster2, Servo actuator, int pwmThrust)
{
  if (pwmThrust < 1300) 
  { 
    pwmThrust = 1300;  // Constrain minimum speed
  }
  
  // Move to 90 degrees
  // actuator.write(0);
  // delay(2000);
  // actuator.write(90);
  // Serial.println("Moved Right");
  // delay(2000);  
for (int angle = 0; angle <= 90; angle += 10) {
    actuator.write(angle);
    delay(100);}

  //Run thrusters
  thruster1.writeMicroseconds(pwmThrust);
  thruster2.writeMicroseconds(pwmThrust);
  //Display message
  Serial.print("time period for movement right is set to: ");
  Serial.println(pwmThrust);
}

void stopThrust(Servo thruster1, Servo thruster2, Servo actuate)
{
  actuate.write(0);
  delay(2000);
  thruster1.writeMicroseconds(1500);
  thruster2.writeMicroseconds(1500);

}

uint8_t computeCRC8(const char* data, size_t len)
{
 uint8_t crc = 0x00;
 for(size_t i = 0;i < len; i++)
 {
  crc ^= data[i];
  for(uint8_t j=0; j<8; j++)
   {
    if(crc & 0x80)
    {
      crc = (crc <<1)^ 0x07;
    }
    else
    {
      crc <<= 1;
    }
   }
 }
   return crc;
}

int readColorFrequency(int s2State, int s3State) {
  // Set S2 and S3 pins to select the desired color filter
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);

  // Read the frequency from the OUT pin.
  // pulseIn() measures the duration of a pulse (HIGH or LOW) on a pin.
  // Since the TCS3200 outputs a square wave, measuring either HIGH or LOW pulse
  // duration and then calculating frequency from it is a common approach.
  // The frequency is 1 / (2 * pulseWidth), but often for comparative
  // readings, the raw pulse width (or period) is sufficient.
  // We're measuring the LOW pulse duration here, which is often more stable.
  // The timeout of 1000000 microseconds (1 second) prevents the sketch from
  // hanging if no pulse is detected.
  unsigned long pulseDuration = pulseIn(OUT, LOW, 1000000); // Measure duration of LOW pulse in microseconds

  // If a pulse was detected, convert pulse duration to frequency (Hz)
  // Frequency = 1 / Period. Period = 2 * pulseDuration for 50% duty cycle.
  // If no pulse (or timeout), pulseIn returns 0. Avoid division by zero.
  if (pulseDuration > 0) {
    return (int)(1000000UL / (2 * pulseDuration)); // Convert µs to Hz (1,000,000 µs in 1 second)
  } else {
    return 0; // Return 0 if no pulse was detected (e.g., very dark)
  }
}


void resetSystem() {
  //Serial.println("🔄 Resetting system...");
  // Perform a software reset or system clear
}