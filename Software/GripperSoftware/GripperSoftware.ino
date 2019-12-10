/* GripperSoftware

This software runs on the OpenCM9 and controls the parallel gripper. The interface consists of a letter
which may be followed by numbers:

'pxxxx' set position (0 to approx 5000)
'sxxxx' set speed (0 to 1023)
'txxxx' set torque (0 to 1023)
'i' get one data frame of the current position consisting of (position,speed,load,volts,temp)
'c' close gripper
'o' open gripper
'l' close gripper to lemon width
'm' move gripper to mid position

Assumes the only one dynamixel is connected with ID 0x01

If you are not running it with the uninterruptable power supply circuit, comment out that section in the main loop.

Jasper Brown 2019

*/

// Set these value to calibrate the gripper
#define openPosition 800
#define closedPosition 10500
#define midPosition (closedPosition - 5000)
#define lemonPosition closedPosition - 1400

// Control Parameters
#define start_speed 1023 //max 1023
#define start_torque 1023 //max 1023
//#define start_torque 500 //max 1023

// Function Definitions
unsigned int parseUInt (byte*, byte);
void parseString();

// Global variables
unsigned int setSpeed = start_speed;
unsigned int setPos = openPosition;
unsigned int setTorque = start_torque;
int voltSensePin = 0;
int voltVal =0;

Dynamixel Dxl(3); //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

// Serial stuff
byte inString[20];
int stringLength = 0;
boolean sendStatus = false;
boolean streamData = false;

void setup() {
  Dxl.begin(1);  //57600 baud rate
  delay(1000);

  //Set initial parameters servo 1
  Dxl.writeWord(1,34, setTorque);
  Dxl.goalSpeed(1, setSpeed);
  //Dxl.goalPosition(1, setPos);
  
  //Set to multi turn mode by defining the joint limits
  Dxl.writeWord(1,6, 4095);
  Dxl.writeWord(1,8, 4095);
  
  //Set resolution divider
  Dxl.writeByte(1,22,1);
  
  SerialUSB.println("Initialised");
  SerialUSB.attachInterrupt(usbInterrupt);
}


void usbInterrupt(byte* buffer, byte nCount){
  for(unsigned int i=0; i < nCount;i++){  //printf_SerialUSB_Buffer[N]_receive_Data
    if(stringLength > 19){
      stringLength = 0;
    }
    if(buffer[i] == '\n'){
        parseString();
        stringLength = 0;
    }
    else{
      inString[stringLength] = buffer[i];
      stringLength++;
    }
  }
}

// THIS IS WHERE STUFF HAPPENS //
void parseString(){
  //Parse String
  if(inString[0] == -1){
      //Serial was not actually read
  }
  //Close gripper
  else if((char)inString[0] == 'c'){
    Dxl.goalPosition(1, closedPosition);
  }
  //Open gripper
  else if((char)inString[0] == 'o'){
    Dxl.goalPosition(1, openPosition);
  }
  //Middle pos, neither open nor closed
  else if((char)inString[0] == 'm'){
    Dxl.goalPosition(1, midPosition);
  }
  //Lemon width pos, almost closed
  else if((char)inString[0] == 'l'){
    Dxl.goalPosition(1, lemonPosition);
  }
  //Set speed
  else if((char)inString[0] == 's'){
      setSpeed = parseUInt(inString, stringLength);
      Dxl.goalSpeed(1, setSpeed);
  }
  //Set Position
  else if((char)inString[0] == 'p'){
      setPos = parseUInt(inString, stringLength);
      Dxl.goalPosition(1, setPos);
  }
  //Set Torque
  else if((char)inString[0] == 't'){
      setTorque = parseUInt(inString, stringLength);
      Dxl.writeWord(1,34, setTorque);
  }
  //Send data packet
  else if((char)inString[0] == 'i'){
      sendStatus = true;
  }
}
// END STUFF HAPPENING //

unsigned int parseUInt(byte* buffer, byte nCount){
  unsigned int number = 0;
  int tempInt = 0;
  
  for(int i = 1; i<nCount; i++){
    tempInt = (int)buffer[i] - 48;
    number = number + (unsigned int)tempInt*(pow(10,(nCount-i-1)));
  }
  return number;
}

//Main loop
void loop() {
  if(sendStatus){
    //Position
    SerialUSB.print(Dxl.readWord(1, 36));
    SerialUSB.print(",");
    //Speed
    SerialUSB.print(Dxl.readWord(1, 38));
    SerialUSB.print(",");
    //Load
    SerialUSB.print(Dxl.readWord(1, 40));
    SerialUSB.print(",");
    //Volts
    SerialUSB.print(Dxl.readByte(1, 42));
    SerialUSB.print(",");
    //Temp
    SerialUSB.println(Dxl.readByte(1, 43));
    sendStatus = false; 
  }
  
  /* Uninterruptable power supply logic, voltVal should read ~4000 when supplied by 3.3V and zero when power is lost */
  voltVal = analogRead(voltSensePin);  // read the input pin
  if(voltVal < 3000){
    SerialUSB.println("Power Lost!");
    Dxl.goalPosition(1, openPosition);
    delay(100);
  }
}
