/* ChangeID
 
 Changes Dynamixel ID and displays register values

Jasper Brown 2017

 */


#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

int OldID = 13;
int NewID = 1;

Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  
  Dxl.begin(1);  //Mx28 uses 57600 by default
  delay(4000);
  
  for (int i=1; i<50; i++){
    SerialUSB.print(i);
    delay(10);
    
    int model = Dxl.getModelNumber(i);

    if(model == 12)
      SerialUSB.println(": AX-12A");

    else if(model == 300)
      SerialUSB.println(": AX-12W");

    else if(model == 18)
      SerialUSB.println(": AX-18A");

    else if(model == 29)
      SerialUSB.println(": MX-28");     

    else if(model == 54)
      SerialUSB.println(": MX-64");

    else if(model == 64)
      SerialUSB.println(": MX-106");

    else if(model == 350)
      SerialUSB.println(": XL-320");   

    else{
      if(model == 65535) model = 0;
      SerialUSB.print(": Unknown : "); 
      SerialUSB.println(model);
    }
    
  }

Dxl.writeByte(1,25,1);
    delay(50);

Dxl.writeByte(1,24,0);
    delay(50);

Dxl.writeWord(1,34,10);
    delay(50);
    
//Dxl.goalTorque(1,10);

//////////CAHANGE ID HERE ---------------------------------------
Dxl.writeByte(OldID,3, NewID);


  for(int i = 0; i <74; i++){
    SerialUSB.print("i is :  ");
    SerialUSB.print(i);
    SerialUSB.print("   value is :  ");
    SerialUSB.println(Dxl.readByte(NewID,i));
        delay(50);

}



}


void loop() {

//  
}

