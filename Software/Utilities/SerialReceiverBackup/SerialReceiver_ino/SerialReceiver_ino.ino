/* SerialReciever
 
 Receives serial commands used to control MX28 dynamixels

Jasper Brown 2017

This uses a CM9 controller on 485EXP board with multiple mx28AT dynamixels. It is designed to be a simple serial reciever and controller for use with the ACFR TCP approach to device control.
 */

//Control Parameters ---------
#define start_speed_1 0
#define start_pos_1 1500 + home_offset_1
#define start_torque_limit_1 600

#define start_speed_2 0
#define start_pos_2 0 + home_offset_2
#define start_torque_limit_2 600

#define start_speed_3 0
#define start_pos_3 0 + home_offset_3
#define start_torque_limit_3 600

#define start_speed_4 0
#define start_pos_4 0 + home_offset_4
#define start_torque_limit_4 600

#define home_offset_1 0
#define home_offset_2 1180
#define home_offset_3 1180
#define home_offset_4 1395

// Function Definitions ---------
void listDevices();
void listIDs();

void readPosition();
void readSpeed();
void readTorque();
void readTemp();

unsigned int parseUInt (byte*, byte);
void parseString();

//Global variables -----------------
unsigned int setSpeed = start_speed_1;
unsigned int setPos = start_pos_1;
int currentServo = 1;

#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL3);

byte inString[20];
int stringLength = 0;
boolean sendStatus = false;


void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  
  Dxl.begin(1);  //Mx28 uses 57600 by default
  delay(3000);
  
  listDevices();

  //Set initial parameters servo 1
  Dxl.writeWord(1,34, start_torque_limit_1);
  Dxl.goalSpeed(1, start_speed_1);  //Dynamixel ID 1 Speed Control 100 setting
  Dxl.jointMode(1); //jointMode() is to use position mode
  Dxl.goalPosition(1, start_pos_1);
  
  //Set initial parameters servo 2
  Dxl.writeWord(2,34, start_torque_limit_2);
  Dxl.goalSpeed(2, start_speed_2);  //Dynamixel ID 1 Speed Control 100 setting
  Dxl.jointMode(2); //jointMode() is to use position mode
  Dxl.goalPosition(2, start_pos_2);
  
  //Set initial parameters servo 3
  Dxl.writeWord(3,34, start_torque_limit_3);
  Dxl.goalSpeed(3, start_speed_3);  //Dynamixel ID 1 Speed Control 100 setting
  Dxl.jointMode(3); //jointMode() is to use position mode
  Dxl.goalPosition(3, start_pos_3);
  
  //Set initial parameters servo 4
  Dxl.writeWord(4,34, start_torque_limit_4);
  Dxl.goalSpeed(4, start_speed_4);  //Dynamixel ID 1 Speed Control 100 setting
  Dxl.jointMode(4); //jointMode() is to use position mode
  Dxl.goalPosition(4, 2700); //Drive 4 is inverted

  
  SerialUSB.println("Initialised");
  SerialUSB.attachInterrupt(usbInterrupt);
    listDevices();
  delay(2000);
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

  SerialUSB.println("inString at zero is: ");
  SerialUSB.println((char)inString[0]);

  //Parse String
  if(inString[0] == -1){
      //Serial was not actually read
      SerialUSB.println("ReadError");
  }
  //Status characters
  else if((char)inString[0] == 'l'){
    //List devices
    listDevices();
  }
  else if((char)inString[0] == 's'){
      setSpeed = parseUInt(inString, stringLength);
      SerialUSB.println("set speed is now");
      SerialUSB.println(setSpeed);
      Dxl.goalSpeed(currentServo, setSpeed);
  }
  else if((char)inString[0] == 'p'){
      setPos = parseUInt(inString, stringLength);
      SerialUSB.println("set pos is now");
      SerialUSB.println(setPos);
      setPos = setPos;
      
      switch(currentServo){
        case 1:
          setPos = setPos + home_offset_1;
          break;
        
        case 2:
          setPos = setPos + home_offset_2;
          break;
        
        case 3:
          setPos = setPos + home_offset_3;
          break;
        
        case 4:
          setPos = setPos + home_offset_4;
          setPos = 4095 - setPos; 
          break;
          
        default :
          SerialUSB.println("Invalid Servo Number Selected");
          break;
      }
      
      Dxl.goalPosition(currentServo, setPos);
      
  }
  else if((char)inString[0] == 't'){
      int torque = parseUInt(inString, stringLength);
         
      Dxl.writeWord(currentServo,34, torque);
      delay(50);

      SerialUSB.println("set torque is now");
      SerialUSB.println(torque);
      
  }
  else if((char)inString[0] == 'a'){
      currentServo = parseUInt(inString, stringLength);
      SerialUSB.println("Active servo is now");
      SerialUSB.println(currentServo);
  }
  else if((char)inString[0] == 's'){
      sendStatus = true;
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
  if(sendStatus){
    SerialUSB.println("Status:");
    //Position
    SerialUSB.print(Dxl.readWord(currentServo, 36));
    SerialUSB.print(",");
    //Speed
    SerialUSB.print(Dxl.readWord(currentServo, 38));
    SerialUSB.print(",");
    //Load
    SerialUSB.print(Dxl.readWord(currentServo, 40));
    SerialUSB.print(",");
    //Volts
    SerialUSB.print(Dxl.readByte(currentServo, 42));
    SerialUSB.print(",");
    //Temp
    SerialUSB.println(Dxl.readByte(currentServo, 43));
  
  
    sendStatus = false; 
  }
}

void listDevices(){
  //Determine number of dynamixels connected (DOES NOT WORK DURING INTTERUPT)
  int model;
  int numConnected = 0;
  
  for (int i=1; i<10; i++){
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
void listIDs(){
  
}

void readPosition(){}
void readSpeed(){}
void readTorque(){}
void readTemp(){}



