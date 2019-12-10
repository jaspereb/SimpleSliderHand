/* SerialReciever
 
 Receives serial commands used to control MX28 dynamixels

Jasper Brown 2017

This uses a CM9 controller on 485EXP board with multiple mx28AT dynamixels. It is designed to be a simple serial reciever and controller for use with the ACFR TCP approach to device control.
 */

//Control Parameters ---------
#define start_speed 300
#define start_pos 0
#define start_torque_limit 0x3FF

// Function Definitions ---------
void listDevices();
void listIDs();

void readPosition();
void readSpeed();
void readTorque();
void readTemp();

void writeTorque(unsigned int);
void writeCLimit(int);
void writeCCLimit(int);

unsigned int getValue();
int getSignedValue();
unsigned int parseUInt (byte*, byte);
void parseString();

//Global variables -----------------
unsigned int setSpeed = start_speed;
unsigned int setPos = start_pos;
int currentServo = 1;

#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL3);

byte inString[20];
int stringLength = 0;


void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  
  Dxl.begin(1);  //Mx28 uses 57600 by default
  delay(3000);
  
  listDevices();

  //Set initial parameters
  Dxl.goalSpeed(currentServo, start_speed);  //Dynamixel ID 1 Speed Control 100 setting
  Dxl.jointMode(currentServo); //jointMode() is to use position mode
  
  Dxl.goalPosition(currentServo, start_pos);
  
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
        SerialUSB.println("Newline Received");
        parseString();
        stringLength = 0;
    }
    else{
      SerialUSB.println((char)buffer[i]);
      inString[stringLength] = buffer[i];
      stringLength++;
    }
  }
}
    
void parseString(){
//  SerialUSB.println("inString is: ");
//  for(int i = 1; i < stringLength; i++){
//    SerialUSB.print((char)inString[i]);
//  }
  
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
  }
  else if((char)inString[0] == 'p'){
      setPos = parseUInt(inString, stringLength);
      SerialUSB.println("set pos is now");
      SerialUSB.println(setPos);
  }
  else if((char)inString[0] == 't'){
      int torque = parseUInt(inString, stringLength);
         
      Dxl.writeWord(currentServo,34, torque);
      delay(50);

      SerialUSB.println("set torque is now");
      SerialUSB.println(torque);

      //flash LED
      Dxl.writeByte(currentServo,25,1);
      delay(500);
      Dxl.writeByte(currentServo,25,0);
      delay(500);
      Dxl.writeByte(currentServo,25,1);
      delay(500);
      Dxl.writeByte(currentServo,25,0);
      delay(500);
      
  }
  else if((char)inString[0] == 'a'){
      currentServo = parseUInt(inString, stringLength);
      SerialUSB.println("Active servo is now");
      SerialUSB.println(currentServo);
  }
  else if((char)inString[0] == 'j'){
    SerialUSB.println("Temperature Is: ");
      SerialUSB.println(Dxl.readByte(currentServo,43)); //Get Temp
  }
  
  //Set actual values ------------------------------------------------------------------
  Dxl.jointMode(currentServo);
  Dxl.goalSpeed(currentServo, setSpeed);
  delay(10);
  Dxl.goalPosition(currentServo, setPos);
  //------------------------------------------------------------------------------------
}

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
//  listDevices();

//  
}

unsigned int getValue(){
//   while(!SerialUSB.available()){
//        delay(5);
//   }
//   char delimeter='\n';
//   char c;
//   int i=0;
//   char intBuffer[24];
//    while (i<23&&SerialUSB.available()&&(c=SerialUSB.read()!=delimeter)) {
//        intBuffer[i++]=c;
//    }
//    intBuffer[i]=0; //end of string.
//    int value = atoi(intBuffer);
//    
//    SerialUSB.println("value is");
//        SerialUSB.println(intBuffer[0]);
//
//   String somestring = "this";
//   SerialUSB.println(value);
//   
//   unsigned int uvalue = (unsigned int) value;
//   return uvalue;
return 0;
   //Could do with having a timeout here
}

int getSignedValue(){
//   while(!SerialUSB.available()){
//        delay(5);
//   }
//   char delimeter='\n';
//   char c;
//   int i=0;
//   char intBuffer[24];
//    while (i<23&&SerialUSB.available()&&(c=SerialUSB.read()!=delimeter)) {
//        intBuffer[i++]=c;
//    }
//    intBuffer[i]=0; //end of string.
//    int value =atoi(intBuffer);
//    
//   return value;
  return 0;
   //Could do with having a timeout here
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

void writeTorque(unsigned int torque){
  
}
void writeCLimit(int limit){
  
}
void writeCCLimit(int limit){
  
}


