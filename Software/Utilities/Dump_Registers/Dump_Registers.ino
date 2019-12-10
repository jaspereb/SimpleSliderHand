/* SerialReciever
 
 Receives serial commands used to control MX28 dynamixels

Jasper Brown 2017

This uses a CM9 controller on 485EXP board with multiple mx28AT dynamixels. It is designed to be a simple serial reciever and controller for use with the ACFR TCP approach to device control.
 */


#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  
  Dxl.begin(1);  //Mx28 uses 57600 by default
  delay(4000);

  for(int i = 0; i <74; i++){
    SerialUSB.print("i is :  ");
    SerialUSB.print(i);
    SerialUSB.print("   value is :  ");
    SerialUSB.println(Dxl.readByte(1,i));
    delay(50);
}

Dxl.writeByte(1,25,1);
    delay(50);

Dxl.writeByte(1,24,0);
    delay(50);

Dxl.writeWord(1,34,10);
    delay(50);
    
//Dxl.goalTorque(1,10);



  for(int i = 0; i <74; i++){
    SerialUSB.print("i is :  ");
    SerialUSB.print(i);
    SerialUSB.print("   value is :  ");
    SerialUSB.println(Dxl.readByte(1,i));
        delay(50);

}


}


void loop() {

//  
}

