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

#define DXL_BUS_SERIAL1 1
#define ID_NUM3 3
#define ID_NUM4 4
#define Timer_rate 250
#define pi 3.14159265359
#define D2R pi/180
#define R2D 180/pi
#define Deg 0.29325513

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
  delay(1000);
  SerialUSB.print("c2 = ");
  SerialUSB.print(c2);
  SerialUSB.print("x = ");
  SerialUSB.print(x);
  SerialUSB.print("y = ");
  SerialUSB.print(y);
  SerialUSB.print("z = ");
  SerialUSB.print(z);

  temp1 = (x-l1) / l2;
  phi1 = acos(temp1);
  SerialUSB.print("temp1 = ");
  SerialUSB.print(temp1);
  SerialUSB.print(" phi = ");
  SerialUSB.println(phi1);
  temp2 = z / l2;
  if(temp2 > 1.0 && temp2 < -1.0){
    SerialUSB.println("ERROR!!!!!!");
  }
  angle = acos(temp2);
  SerialUSB.print("temp2 = ");
  SerialUSB.print(temp1);
  SerialUSB.print(" angle = ");
  SerialUSB.println(phi1);
}

void inputDegree(){
  double rad1;
  double rad2;
  delay(1000);
  Dxl.setPosition(ID_NUM3,512,100);
  int presentPos1 = 512;//(Dxl.readByte(ID_NUM3, 37) << 8 )  + Dxl.readByte(ID_NUM3, 36);
  q1 = D2R*(Deg*(presentPos1-512));
  SerialUSB.print("rad1 = ");
  SerialUSB.print(q1);

  Dxl.setPosition(ID_NUM4,512,100);
  int presentPos2 = 512;//(Dxl.readByte(ID_NUM3, 37) << 8 )  + Dxl.readByte(ID_NUM3, 36);
  q2 = D2R*(Deg*(presentPos2-512));
  SerialUSB.print("rad2 = ");
  SerialUSB.print(q2);
}

void inverKinema(){
  double theta1 = 0.0;
  double theta2 = 0.0;

  theta2 = y / (x - l1);
  q2 = asin(theta2);
  theta1 = z / y;
  q1 = atan(theta1);
  SerialUSB.print(" q1 = ");
  SerialUSB.print(q1); 
  SerialUSB.print(" q2 = ");
  SerialUSB.println(q2);
}

int deg2pos(double ra){
 int ret;
 SerialUSB.print("rad = ");
 SerialUSB.println(ra);

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

  x = l2 * cos(phi1);
  y = l2 * sin(phi1);
  z = x * cos(angle);

  SerialUSB.print("cos = ");
  tmp = cos(phi1);
  SerialUSB.print(tmp);
  SerialUSB.print("l2 = ");
  SerialUSB.print(l2);
  SerialUSB.print("phi1 = ");
  SerialUSB.print(phi1);
  SerialUSB.print("angle = ");
  SerialUSB.print(angle);
  SerialUSB.print("cnv_x = ");
  SerialUSB.print(x);
  SerialUSB.print("cnv_y = ");
  SerialUSB.print(y);
  SerialUSB.print("cnv_z = ");
  SerialUSB.print(z);
}


void loop() {
/*
  SerialUSB.print("theta1 = ");
  SerialUSB.print(deg2pos(q1));
  SerialUSB.print(" ");
  SerialUSB.print("theta2 = ");
  SerialUSB.print(deg2pos(q2));
  SerialUSB.print(" x= ");
  SerialUSB.print(x);
  SerialUSB.print(" y= ");
  SerialUSB.print(y);
  SerialUSB.print(" z= ");
  SerialUSB.println(z);
  SerialUSB.print("phi1= ");
  SerialUSB.print(phi1);
  SerialUSB.print(" theta= ");
  SerialUSB.println(angle);

  SerialUSB.print(" ch4 = ");
  SerialUSB.print(paw1);
  SerialUSB.print(", ch3 = ");
  SerialUSB.print(paw2);
  SerialUSB.print(", ch2 = ");
  SerialUSB.print(paw3);
  SerialUSB.print(", ch1 = ");
  SerialUSB.println(paw4);
*/
  if(paw4 < paw3 && paw4 < paw2 && paw4 < paw1){
    if(paw4 < 900){
      phi1 = phi1 + 5*D2R;
      SerialUSB.print(" Before phi1 no atai  = ");
      SerialUSB.println(phi1);
      convCoordinate();
    }
  }
  if(paw2 < paw4 && paw2 < paw3 && paw2 < paw1){
    if(paw2 < 900){
      phi1 = phi1 - 5*D2R;
      convCoordinate();
    }
  }
  if(paw3 < paw4 && paw3 < paw2 && paw3 < paw1){
    if(paw3 < 900){
      SerialUSB.print(" Before theta no atai  = ");
      SerialUSB.println(angle);
      angle = angle + 5*D2R;
      convCoordinate();
    }
  }
  if(paw1 < paw4 && paw1 < paw2 && paw1 < paw3){
    if(paw1 < 900){
      angle = angle - 5*D2R;
      convCoordinate();
    }
  }
  inverKinema();
  Dxl.setPosition(ID_NUM3,deg2pos(q1),100);
  Dxl.setPosition(ID_NUM4,deg2pos(q2),100);
  
}


