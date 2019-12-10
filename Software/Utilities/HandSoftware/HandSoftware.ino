/* SerialReciever
 
 Receives serial commands used to control MX28 dynamixels

This is the development code, the actual deployed code is called 'GripperSoftware'

Jasper Brown 2017

This uses a CM9 controller on 485EXP board with multiple mx28AT dynamixels. It is designed to be a simple serial reciever and controller for use with the ACFR TCP approach to device control.
 */

//Calibration
#define openPosition 3000
#define closedPosition 0

//Control Parameters ---------
#define start_speed 500 //max 1024
#define start_torque 500 //max 1024




// Function Definitions ---------
void listDevices();
void listIDs();

void selfCalibrate();

unsigned int parseUInt (byte*, byte);
void parseString();

//Global variables -----------------
unsigned int setSpeed = start_speed;
unsigned int setPos = openPosition;
unsigned int openPos = 100;
unsigned int closedPos = 0;

#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL3);

byte inString[20];
int stringLength = 0;
boolean sendStatus = false;
boolean streamData = false;


void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  
  Dxl.begin(1);  //Mx28 uses 57600 by default
  delay(3000);
  
  listDevices();

  //Set initial parameters servo 1
  Dxl.writeWord(1,34, start_torque);
  Dxl.goalSpeed(1, start_speed);
  //Dxl.jointMode(1); //jointMode() is to use position mode
  //Dxl.goalPosition(1, start_pos_1);
  
  //Set to multi turn mode by defining the joint limits
  Dxl.writeWord(1,6, 4095);
  Dxl.writeWord(1,8, 4095);
  
  //Set resolution divider
  Dxl.writeByte(1,22, 1);
  
  SerialUSB.println("Initialised");
  SerialUSB.attachInterrupt(usbInterrupt);
    listDevices();
  delay(2000);
  
  //selfCalibrate();
}


void usbInterrupt(byte* buffer, byte nCount){
  for(unsigned int i=0; i < nCount;i++){  //printf_SerialUSB_Buffer[N]_receive_Data
    if(stringLength > 19){
      stringLength = 0;
    }
    if(buffer[i] == '\n'){
//        SerialUSB.println("Newline Received");
        parseString();
        stringLength = 0;
    }
    else{
//      SerialUSB.println((char)buffer[i]);
      inString[stringLength] = buffer[i];
      stringLength++;
    }
  }
}

// THIS IS WHERE STUFF HAPPENS // -------------------------------------------------------------------------------------------------------
void parseString(){

  //Parse String
  if(inString[0] == -1){
      //Serial was not actually read
      
  }
  //Status characters
  else if((char)inString[0] == 'l'){
    //List devices
    listDevices();
  }
  //Close gripper
  else if((char)inString[0] == 'c'){
    Dxl.goalPosition(1, closedPos);
  }
  //Open gripper
  else if((char)inString[0] == 'o'){
    Dxl.goalPosition(1, openPos);
  }
  //Middle pos, neither open nor closed
  else if((char)inString[0] == 'm'){
    Dxl.goalPosition(1, closedPos+1000);
  }
  else if((char)inString[0] == 's'){
      setSpeed = parseUInt(inString, stringLength);
      
      Dxl.goalSpeed(1, setSpeed);
  }
  else if((char)inString[0] == 'p'){
      setPos = parseUInt(inString, stringLength);
      Dxl.goalPosition(1, setPos);
      
  }
  else if((char)inString[0] == 't'){
      int torque = parseUInt(inString, stringLength);
         
      Dxl.writeWord(1,34, torque);
      delay(50);

      
  }

  else if((char)inString[0] == 'i'){
      sendStatus = true;
  }
  else if((char)inString[0] == 'u'){
      streamData = !streamData;
  }
}
// END STUFF HAPPENING // -----------------------------------------------------------------------------------------------------------------------------------------

unsigned int parseUInt(byte* buffer, byte nCount){
  unsigned int speed = 0;
  int tempInt = 0;
  
  for(int i = 1; i<nCount; i++){
    tempInt = (int)buffer[i] - 48;
    speed = speed + (unsigned int)tempInt*(pow(10,(nCount-i-1)));
  }
  
  return speed;
}

void loop() {
  if(sendStatus || streamData){
    
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
    delay(100);
  }
}

void listDevices(){
  //Determine number of dynamixels connected (DOES NOT WORK DURING INTTERUPT)
  int model;
  int numConnected = 0;
  
  for (int i=1; i<20; i++){
    SerialUSB.print(i);
    delay(10);

    model = Dxl.readWord(i, 0);

    if(model == 12){
      SerialUSB.println(": AX-12A");
      SerialUSB.println("Error: Non Mx28 Detected");
    }

    else if(model == 300){
      SerialUSB.println(": AX-12W");
      SerialUSB.println("Error: Non Mx28 Detected");
    }

    else if(model == 18){
      SerialUSB.println(": AX-18A");
      SerialUSB.println("Error: Non Mx28 Detected");
    }

    else if(model == 29){
      SerialUSB.println(": MX-28");
      numConnected++;
    }     

    else if(model == 54){
      SerialUSB.println(": MX-64");
      SerialUSB.println("Error: Non Mx28 Detected");
    }

    else if(model == 64){
      SerialUSB.println(": MX-106");
      SerialUSB.println("Error: Non Mx28 Detected");
    }

    else if(model == 350){
      SerialUSB.println(": XL-320"); 
      SerialUSB.println("Error: Non Mx28 Detected");
    }  

    else{
      if(model == 65535) model = 0;
      SerialUSB.print(": Empty : "); 
      SerialUSB.println(model);
    }
  }
}

void selfCalibrate(){
  int currTorque = 0;
  unsigned int currPos = 1500;
  
  /* Self calibration has never worked properly because the torque readings coming from the servo
  appear to be essentially random. */
  
  /* To relieve the motor torque it has to be reversed slightely after every move */
  
  SerialUSB.println("Now self calibrating"); 
  Dxl.goalPosition(1, 1400);
  delay(2000);
  Dxl.goalPosition(1, 1500);
  delay(200);
  Dxl.goalPosition(1, 1490);
  delay(200);
  
  currTorque = Dxl.readWord(1, 40);

  SerialUSB.print("Torque: "); 
  SerialUSB.println(currTorque);
  
  if(currTorque > 1000){
    SerialUSB.println("Error: Self calibration failed, torque at p1500 too high"); 
  }
  
  SerialUSB.print("Closing: "); 
  SerialUSB.println(currTorque);
  
  //Close until torque rises
  while(currTorque < 1000){
    currPos = currPos - 200;
    Dxl.goalPosition(1, currPos);
    delay(2000);
    Dxl.goalPosition(1, currPos+10);
    delay(500);
    
 
    
    currTorque = Dxl.readWord(1, 40);
    SerialUSB.println(currTorque);
  }
  closedPos = currPos;
  
  currPos = 1500;
  Dxl.goalPosition(1, currPos);
  delay(1000);
  while(currTorque < 1000){
    currPos = currPos + 10;
    Dxl.goalPosition(1, currPos);
    currTorque = Dxl.readWord(1, 40);
    delay(50);

  }
  openPos = currPos;
  
  SerialUSB.println("Self calibration complete"); 
}


