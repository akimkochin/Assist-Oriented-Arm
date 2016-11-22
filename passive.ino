/* Dynamixel Wheel Mode Example
 
 This example shows how to use dynamixel as wheel mode
 All dynamixels are set as joint mode in factory,
 but if you want to make a wheel using dynamixel, 
 you have to change it to wheel mode by change CCW angle limit to 0
 
               Compatibility
 CM900                  O
 OpenCM9.04             O
 
                  Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****  
 
 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
 /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Dynamixel ID defines */
#define ID_NUM 2
/* Control table defines */
#define GOAL_SPEED 32
#define CCW_Angle_Limit 8
#define CONTROL_MODE 11

Dynamixel Dxl(DXL_BUS_SERIAL1);
int state=0;
int load2=0;
void setup() {
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  
  pinMode(22, INPUT_PULLDOWN);
  //AX MX RX Series
  Dxl.writeWord(ID_NUM, CCW_Angle_Limit, 0);
  Dxl.writeWord(1, CCW_Angle_Limit, 0); 
  //disable CCW Angle Limit(L) to use wheel mode
  
  //XL-320
  //Dxl.writeByte(ID_NUM, CONTROL_MODE, 1);
}

void loop() {
  //forward
  state = digitalRead(22);
  if(state == HIGH){
    Dxl.writeWord(ID_NUM, GOAL_SPEED, 400 | 0x400);
    //Dxl.writeWord(1, GOAL_SPEED, 400 | 0x400); 
    //load2 = (Dxl.readByte(ID_NUM, 41) << 8 ) + Dxl.readByte(ID_NUM, 40);
    SerialUSB.println(Dxl.readByte(ID_NUM, 41));
    delay(500);
    for(int count = 0;state == HIGH;count++){
      Dxl.writeWord(ID_NUM, GOAL_SPEED, 0);
      //Dxl.writeWord(1, GOAL_SPEED, 0);
      state = digitalRead(22);
      //SerialUSB.println(count);
    }
    // Dxl.writeWord(ID_NUM, GOAL_SPEED, 400 | 0x400);
    // Dxl.writeWord(1, GOAL_SPEED, 400 | 0x400);
    // delay(3500);
    // SerialUSB.println("reverse sucsess");
  }
  
  Dxl.writeWord(ID_NUM, GOAL_SPEED, 0);
  //Dxl.writeWord(1, GOAL_SPEED, 0);
  //reverse
  //Dxl.writeWord(ID_NUM, GOAL_SPEED, 400 | 0x400);
  //delay(1000); 
  //stop
  //Dxl.writeWord(ID_NUM, GOAL_SPEED, 0); 
  //delay(2000);
}

