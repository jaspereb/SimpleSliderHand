# openHandCM9
Serial code to run on the OpenCM9 for the control of Dynamixel servos

This code was written to control the Yale openHand using 4 dynamixel MX28AT servos along with an OpenCM9 board and 485EXP board for power.

Download the serialReceiver code onto the OpenCM9 then send it serial commands from a PC terminal. It is set up for control of 4 servos but can easily be modified for other numbers. Most of the configuration variables are in this file.

There are also a few matlab functions to do basic hand control. These just act as serial wrappers.

The most important commands are:

a%d\n - set the active servo number (based on ID) eg. 'a1\n'

p%d\n - set the position of active servo eg. 'p1500\n'

t%d\n - set the torque limit of servo eg. 't500\n'

s%d\n - set the speed of servo eg. 's1000\n'


# From my project notes:


I used MX28AT dynamixels for my gripper design and ran into a bunch of not obvious problems because the Robotis documentation is awful. So this is a very brief guide to setting up dynamixels for USB control via serial commands. Eventually I'll put the code online.
 
The openhand uses 4 dynamixel MX28AT's, with an opencm9 and 485EXP board. The former is an arduino based (but not arduino) board and the latter is just to provide power (12V used, but this is variable) and comms.
 
You can download the full IDE from github if using a USB2Dynamixel stick, or use the OpenCM IDE (which is a totally different thing) with the CM9 board. 
 
To use the CM9:
Download the IDE which looks like the Arduino one and look through some of the examples
Connect the CM9 via USB to the computer
Attempt to download the blink_LED sketch, if you get the board not responding error then you can try to unplug the board, hold down the user button and plug it back in. The green LED should stay lit. If you still have the error:
I could not get it working in ubuntu 16.04, about one in 3 times it would correctly program, the rest would give that error. So try windows
Try installing the arduino IDE to get usb drivers in windows
Remove it from the EXP board, you don't need this to run the blink_LED sketch
If it downloads properly more than once, you can attach the EXP board. Make sure it is the right way around
Attach power to the EXP board, see docs for the various power options
Plug in the servos to the EXP board
The black switch on the EXP board enables power to both servos and the CM9 if not powered over usb
 
Dynamixels have a set of control registers that can be written with writeByte(ID, REG, VALUE); or writeWord(ID, LOWREG, VALUE); for double register operations. The word write function must be used for dual register operations or they will not work. The timing is sensitive and not documented. 50ms delays after write commands seem to work. See http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-28at_ar.htm for the register list.
Be careful of timing, add delays if things don't work
Reg 25 can be used to control the LED for testing
writeByte must be used for single registers
writeWord must be used for double registers
Make sure the baud rate is set (MX28 use Dxl.begin(1) by default which is 57600 baud)
Make sure the bus is set to use the EXP485 not the CM9, this is done when creating the dynamixel object and should be "Dynamixel Dxl(3);"
The goalTorque() command never worked for me
OpenCM9 will not auto assign ID numbers, so if using more than one dynamixel you need to set them manually. If there are conflicting IDs on the network, write commands will execute on all of them (so you can use IDs to make synchronised actuator groups, not sure if this is wise/stable), read commands will not execute properly though.
Dynamixel documentation is very spread out and many of the documents are wrong. I am still not quite sure what the following means:

A good way of checking settings is to dump all of the registers to serial using:
```C
for(int i = 0; i <74; i++){
    SerialUSB.print("i is :  ");
    SerialUSB.print(i);
    SerialUSB.print("   value is :  ");
    SerialUSB.println(Dxl.readByte(1,i));
}
```
 
The OpenCM9 IDE (looks like the arduino IDE and is written in Java) is very basic, not all libraries are listed at http://support.robotis.com/en/software/robotis_opencm/api_reference.htm but some are. It does not support standard Arduino libraries or standard C libraries (eg. no Strings or easy use of header files). It also seems to use the Maple serial library at http://docs.leaflabs.com/static.leaflabs.com/pub/leaflabs/maple-docs/0.0.12/lang/api/serialusb.html.
Because of this, I have tried to make the code on the CM9 as basic and robust as possible. All control, error handling, formatting and synchronisation should be done on the host computer.
