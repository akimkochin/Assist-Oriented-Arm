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
#define ID_NUM3 3
#define ID_NUM4 4
#define Timer_rate 250
#define pi 3.14159265359
#define D2R pi/180
#define R2D 180/pi
#define Deg 0.29325513
#define Max_rad 5.2358333
#define Add_anglev 4
#define Add_angleh 4


const int analogPin0 = 0; // Analog input pin that the potentiometer
const int analogPin1 = 1;
const int digitalPin16 = 16;
const int digitalPin17 = 17;


typedef enum{
  CHARGE_1,
  READ_1,
  CHARGE_2,
  READ_2,
}t_paw_state;

t_paw_state pawstat = CHARGE_1;
unsigned short ain1_v0, ain2_v0;
unsigned short paw1, paw2, paw3, paw4;
// These variables will change:
      // value output to the PWM
HardwareTimer Timer(1);

Dynamixel Dxl(DXL_BUS_SERIAL1);



double l1 = 410.0;
double l2 = 2600.0;
double q1 = 0.0*D2R;
double q2 = 0.0*D2R;
double phi1 = 0.0*D2R;
double angle = 0.0*D2R; 

double x = 0.0;
double y = 0.0;
double z = 0.0;
double x_1 = 0.0;
double y_1 = 0.0;
double z_1 = 0.0;

int move = 0;

void kinematic();
void inputDegree();
void inverKinema();
int deg2pos(double ra);
void convCoordinate();


void setup() {
  pinMode(analogPin0, INPUT_ANALOG);
  pinMode(analogPin1, INPUT_ANALOG);
  pinMode(digitalPin16, OUTPUT);
  pinMode(digitalPin17, OUTPUT);
  /*Timer割り込み*/
  Timer.pause();
  Timer.setPeriod(Timer_rate);
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1,1);
  Timer.attachInterrupt(TIMER_CH1, periodicPawRead);
  Timer.refresh();
  Timer.resume();
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.jointMode(ID_NUM3); //jointMode() is to use position mode
  Dxl.jointMode(ID_NUM4);
  inputDegree();
  kinematic();

  //File file = new File("C:\Users\Michiko\Documents\ROBOTIS\kinematics2\prot.csv")
}

void periodicPawRead(){
  switch(pawstat){
    case CHARGE_1:
        ain1_v0 = analogRead(analogPin0);
        ain2_v0 = analogRead(analogPin1);
        digitalWrite(digitalPin16, HIGH);
        break;

    case READ_1:
        paw3 = analogRead(analogPin0) - ain1_v0;
        paw4 = analogRead(analogPin1) - ain2_v0;
        digitalWrite(digitalPin16, LOW);
        break;

    case CHARGE_2:
        ain1_v0 = analogRead(analogPin0);
        ain2_v0 = analogRead(analogPin1);
        digitalWrite(digitalPin17, HIGH);
        break;

    case READ_2:
        paw2 = analogRead(analogPin0) - ain1_v0;
        paw1 = analogRead(analogPin1) - ain2_v0;
        digitalWrite(digitalPin17, LOW);
        break;
  }
  switch(pawstat){
    case CHARGE_1:
        pawstat = READ_1;
        break;

    case READ_1:
        pawstat = CHARGE_2;
        break;

    case CHARGE_2:
        pawstat = READ_2;
        break;

    case READ_2:
        pawstat = CHARGE_1;
        break;
  }

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

void kinematic_tmp(){
  double s1 = sin(q1);
  double c1 = cos(q1);
  double s2 = sin(q2);
  double c2 = cos(q2);
  double temp1 = 0.0;
  double temp2 = 0.0;

  x_1 = l1 + (l2 * c2);
  y_1 = l2 * s2 * c1;
  z_1 = l2 * s2 * s1;
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





  if(paw4 < paw3 && paw4 < paw2 && paw4 < paw1){
    if(paw4 < 400 && paw3 > 700 && paw2 > 700 && paw1 > 700){
      phi1 = phi1 + Add_anglev * D2R;
      //if(phi1 > 5.235){
        //phi1 = 5.235; 
      if(phi1 >= 2.617){
        phi1 = 2.617;
      }
      convCoordinate();
  
  SerialUSB.print(x);
  SerialUSB.print(", ");
  SerialUSB.print(y);
  SerialUSB.print(", ");
  SerialUSB.print(z);
  SerialUSB.print(", ");
  move=1;
    }
  }
  if(paw1 < 400 && paw2 < 400 && paw3 < 400 && paw4 < 400){
    //if(paw2 < 1500){
      phi1 = phi1 - Add_anglev*D2R;
      if(phi1 <= -2.617){
        phi1 = -2.617;
      }
      convCoordinate();

  SerialUSB.print(x);
  SerialUSB.print(", ");
  SerialUSB.print(y);
  SerialUSB.print(", ");
  SerialUSB.print(z);
  SerialUSB.print(", ");
  move=1;  
    //}
  }
  if(paw4 < 400 && paw1 < paw3){
    if(paw1 < 500){
 
      angle = angle + Add_angleh*D2R;
      if(angle >= 2.617){
        angle = 2.617;
      }
      convCoordinate();
  
  SerialUSB.print(x);
  SerialUSB.print(", ");
  SerialUSB.print(y);
  SerialUSB.print(", ");
  SerialUSB.print(z);
  SerialUSB.print(", ");
  move=1;  
    }
  }

  if(paw4 < 400 && paw3 < paw1){
    if(paw3 < 500){
      angle = angle - Add_angleh*D2R;
      if(angle <= -2.617){
        angle = -2.617;
      }
      convCoordinate();
  
  SerialUSB.print(x);
  SerialUSB.print(", ");
  SerialUSB.print(y);
  SerialUSB.print(", ");
  SerialUSB.print(z);
  SerialUSB.print(", ");
  move=1;  
    }
  }
  if (move==1) {
    inverKinema();
    kinematic_tmp();
    Dxl.setPosition(ID_NUM3,deg2pos(q1),1023);
    Dxl.setPosition(ID_NUM4,deg2pos(q2),1023);
    while(Dxl.readByte(ID_NUM3, 46) != 0 ||  Dxl.readByte(ID_NUM3, 46) !=0){
      delay(10);
    }
    SerialUSB.print(x_1);
    SerialUSB.print(", ");
    SerialUSB.print(y_1);
    SerialUSB.print(", ");
    SerialUSB.println(z_1);

    move=0;
  }
 
  
}


