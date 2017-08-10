/* Dynamixel Basic Position Control Example
 
 Turns left the dynamixel , then turn right for one second,
 repeatedly.
 
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
#include <math.h>
#define DXL_BUS_SERIAL1 1
#define DXL_BUS_SERIAL3 3

#define ID_NUM3 3
#define ID_NUM4 4
#define pi 3.14159265359
#define D2R pi/180
#define R2D 180/pi
#define Deg 0.29325513
#define Max_rad 5.2358333
#define Add_anglev 4
#define Add_angleh 4


//const int digitalPin16 = 16;
//const int digitalPin17 = 17;


// These variables will change:
      // value output to the PWM

Dynamixel Dxl(DXL_BUS_SERIAL3);

int count = 0;

double l1 = 410.0;
double l2 = 2600.0;
double q1 = 0.0*D2R;
double q2 = 0.0*D2R;
double phi1 = 0.0*D2R;
double angle = 0.0*D2R; 

double x = 0.0;
double y = 0.0;
double z = 0.0;

unsigned char sendvalue;


void kinematic();
void inputDegree();
void inverKinema();
int deg2pos(double ra);
void convCoordinate();


void setup() {

  Serial2.begin(57600);//bluetoothの通信帯域
  Dxl.begin(3);
 // pinMode(digitalPin16, OUTPUT);
  //pinMode(digitalPin17, OUTPUT);

  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  //pinMode(BOARD_LED_PIN, OUTPUT);

  Dxl.jointMode(ID_NUM3); //jointMode() is to use position mode
  Dxl.jointMode(ID_NUM4);
  inputDegree();
  kinematic();
  SerialUSB.println("setup");
  //File file = new File("C:\Users\Michiko\Documents\ROBOTIS\kinematics2\prot.csv")
}


void kinematic(){
  double s1 = sin(q1);
  double c1 = cos(q1);
  double s2 = sin(q2);
  double c2 = cos(q2);
  double temp1 = 0.0;
  double temp2 = 0.0;

  x = l1 + (l2 * c2);
  y = l2 * s2 * c1;
  z = l2 * s2 * s1;


  temp1 = (x-l1) / l2;
  phi1 = acos(temp1);

  temp2 = z / (x - l1);
  angle = asin(temp2);

  if(temp2 > 1.0 && temp2 < -1.0){
    SerialUSB.println("ERROR!!!!!!");
  }

}

void inputDegree(){
  double rad1;
  double rad2;
  delay(1000);
  Dxl.setPosition(ID_NUM3,512,100);
  int presentPos1 = 512;//(Dxl.readByte(ID_NUM3, 37) << 8 )  + Dxl.readByte(ID_NUM3, 36);
  q1 = D2R*(Deg*(presentPos1-512));


  Dxl.setPosition(ID_NUM4,512,100);
  int presentPos2 = 512;//(Dxl.readByte(ID_NUM3, 37) << 8 )  + Dxl.readByte(ID_NUM3, 36);
  q2 = D2R*(Deg*(presentPos2-512));

}

void inverKinema(){
  double theta1 = 0.0;
  double theta2 = 0.0;

  theta2 = (x - l1) / l2;
 // SerialUSB.print("theta2 = ");
 // SerialUSB.println(theta2);

  /*
  if(theta2 > 1.0){
    theta2 = 1.0;
  }else if(theta2 < -1.0){
    theta2 = -1.0;
  }
  */
  q2 = acos(theta2);
  //SerialUSB.println(q2);
 // theta1 = z / y;
/*  SerialUSB.print("z = ");
  SerialUSB.println(z);
  SerialUSB.print("y = ");
  SerialUSB.println(y);
*/
  q1 = atan2(z, y);
  //SerialUSB.print("q1 = ");
  //SerialUSB.println(q1);

}

int deg2pos(double ra){
 int ret;


 ret = ra * R2D;
 ret = (double)ret / Deg;
 ret = ret + 512;
 if(ret > 1023){
    ret = 1023;
 }
 if(ret < 0){
  ret = 0;
 }
  //SerialUSB.print(" ret = ");
  //SerialUSB.println(ret);
 return ret;
 }
void convCoordinate(){
  double tmp;

    x = l1 + (l2 * cos(phi1) * cos(angle));
    y = l2 * sin(phi1);
    z = l2 * cos(phi1) * sin(angle);
/*
    x = l1 + (l2 * sin(angle) * cos(phi1));
    y = l2 * sin(angle) * sin(phi1);
    z = l2 * cos(angle);
*/
}



void loop() {
  //sendvalue = Serial2.read();
  
  //SerialUSB.println(sendvalue);

  if(Serial2.available()){
    //sendvalue = Serial2.read();
    //SerialUSB.println(sendvalue);
    for(;count <= 100; count++){
      sendvalue = Serial2.read();
      SerialUSB.print("count = ");
      SerialUSB.println(count);
      if(sendvalue == Serial2.read()){
      }
      if(sendvalue != Serial2.read()){
        count = 0;
      }
    }
    if(sendvalue != Serial2.read()){
      count = 0;

    }
  
 
    if(sendvalue == 'a' && count >= 100){
      SerialUSB.print("d in");

      phi1 = phi1 + Add_anglev * D2R;
      //if(phi1 > 5.235){
        //phi1 = 5.235; 
      if(phi1 >= 2.617){
        phi1 = 2.617;
      }
    convCoordinate();
    }
    
    if(sendvalue == 'u' && count >= 100){
      SerialUSB.print("u in");
      phi1 = phi1 - Add_anglev*D2R;
      if(phi1 <= -2.617){
        phi1 = -2.617;
      }
    convCoordinate();
    }
    
    if(sendvalue == 'r' && count >= 100){
      SerialUSB.print("r in");
      angle = angle + Add_angleh*D2R;
      if(angle >= 2.617){
        angle = 2.617;
      }
    convCoordinate();
    }
  
    if(sendvalue == 'l' && count >= 100){
      SerialUSB.print("l in");
      angle = angle - Add_angleh*D2R;
      if(angle <= -2.617){
        angle = -2.617;
      }
    convCoordinate();
    }
  
  }
  inverKinema();
  Dxl.setPosition(ID_NUM3,deg2pos(q1),800);
  Dxl.setPosition(ID_NUM4,deg2pos(q2),800);
  
    //Dynamixelへの命令の衝突を防ぐ
  while(Dxl.readByte(ID_NUM3, 46) != 0 ||  Dxl.readByte(ID_NUM3, 46) !=0){
    delay(10);
  }
 
  
}



