#include <HardwareSerial.h>
// Map Serial1 to D0/D1 (GPIO43/44) on Nano ESP32

#define RX1_PIN 9  // D9 — connected to GM65 TX - GPIO 18
#define TX1_PIN 10  // D10 — connected to GM65 RX (optional) - GPIO 21
#define RXrs 6
#define TXrs 7

HardwareSerial rs232Usbl(1), GM65Serial(2), Comm2IoT(0);  // Use UART1
byte autoScan[] = {0x7E, 0x00, 0x08,0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};

void setup() {
  
  Comm2IoT.begin(9600,SERIAL_8N1, 0,1);
  GM65Serial.begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
  GM65Serial.write(autoScan, sizeof(autoScan));
  rs232Usbl.begin(9600, SERIAL_8N1, RXrs, TXrs);
}

void loop() {
  int r;
  static String barcode = "";
  static String Data="";
  static String msgUsbl = "$FWD,1600*32";
  static String msg2IoT="";
    
    rs232Usbl.println(msgUsbl);
    Data = Scan_QR();
    
    
    
    if(rs232Usbl.available())
    {
      String c = rs232Usbl.readStringUntil('\n');
      r = handleUSBLmsg(&msg2IoT,Data,c);
      
      if(c.length()>0)
      {
        Comm2IoT.println(msg2IoT); 
      }
      

    }
 
    delay(2000);
  
}

String Scan_QR()
{
  unsigned long starttime;
  String str = "";
  GM65Serial.write(autoScan, 9);
     // Waiting for the recovery signal to end
  while (GM65Serial.read() != 0x31);
  starttime = millis();
  while (true)
  {
         // The scan code module is scanned to change the line.
    str = GM65Serial.readStringUntil('/r');
    if ((str.length() >= 7) || (millis() - starttime > 5000))
      break;
  }
  return str;
}

int handleUSBLmsg(String * msgSerial,String msgBarcode,String msgUsbl)
{
  int startIdx = 0;
  // Remove any newline or carriage return
  msgUsbl.trim();

  // Basic sanity check
  if (!msgUsbl.startsWith("$")) {
    return 1;
  }
  
  // Strip off checksum if present
  int asteriskIdx = msgUsbl.indexOf('*');
  
  if (asteriskIdx == -1) {
    return 1;
  }

  //extract the message and checksum
  msgUsbl = msgUsbl.substring(0, asteriskIdx);

  //Convert string checksum into integer
  //uint8_t crcInt = strtoul(crcString.c_str(), NULL, 16);

  //Compute CRC
  //uint8_t crcCalculate = computeCRC8(msg->c_str(), msg->length());
  
  //Coparison of CRCs
  //if (crcInt == crcCalculate) {
      //append  the reading of barcode
      // *serialMsg = *serialMsg+*barcodemsg;
    //} 
    // else 
    // {
      
    //   return 1;
    // }

  *msgSerial = msgUsbl +","+ msgBarcode;
  return 0;
}




  
  




  

  