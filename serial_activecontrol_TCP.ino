
#include <math.h>
#define DXL_BUS_SERIAL1 1
#define DXL_BUS_SERIAL3 3

#define ID_NUM3 3
#define ID_NUM4 4
#define pi 3.14159265359
#define D2R pi/180
#define R2D 180/pi
#define Deg 0.29296875 //dynamixelの角度1はdegreeでは0.29296875
#define Max_rad 5.2358333
#define Add_anglev 4
#define Add_angleh 4

Dynamixel Dxl(DXL_BUS_SERIAL3);

int flag = 0;
int count = 0;
double theta_t_1 = 0.0;

double l1 = 42.0;
double l2 = 210.0;
double q1 = 0.0*D2R;
double q2 = 0.0*D2R;
double phi1 = 0.0*D2R;
double angle = 0.0*D2R;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double beforephi = 0.0;
double beforeangle = 0.0;

unsigned char sendvalue;

double alpha = 0.0;


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
  //q1 = D2R*(Deg*(presentPos1-512));


  Dxl.setPosition(ID_NUM4,512,100);
  int presentPos2 = 512;//(Dxl.readByte(ID_NUM3, 37) << 8 )  + Dxl.readByte(ID_NUM3, 36);
  //q2 = D2R*(Deg*(presentPos2-512));

}

void inverKinema(){
  theta_t_1 = q1;

  q1 = atan2(z, y);
  q2 = atan2(y, x - l1);

  SerialUSB.print(q1);
  SerialUSB.print(",");
  SerialUSB.println(q2);
  //姿勢制御(t-1時のtheta1とt時のtheta1を比較し、差が180だった場合t-1時のtheta角を用いる)
  double res = abs(theta_t_1 - q1);
  SerialUSB.println(res);

  if(res >= 3.14){
    SerialUSB.print("姿勢制御");
    q1 = theta_t_1;
  }
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
  return ret;
}


void convCoordinate(){
  /*double tmp = 90*D2R;
  if(abs(q2) >= tmp){
  return;
}*/
  beforephi = phi1;
  beforeangle = angle;

  x = l1 + (l2 * cos(phi1) * cos(angle));
  y = l2 * sin(phi1);
  z = l2 * cos(phi1) * sin(angle);
  inverKinema();

  double radq2pl = 90 * D2R;
  double radq2mi = -90 * D2R;
  if(q2 > radq2pl || q2 < radq2mi){
    x = l1 + (l2 * cos(phi1) * cos(angle));
    y = l2 * sin(phi1);
    z = l2 * cos(phi1) * sin(angle);
  }else{
    flag = 0;
  }

}



void loop() {

  if(Serial2.available()){
    for(;count <= 100; count++){
      sendvalue = Serial2.read();
      //SerialUSB.print("count = ");
      //SerialUSB.println(count);
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
      //SerialUSB.print("d in");

      phi1 = phi1 + Add_anglev * D2R;
      if(phi1 >= 2.617){
        phi1 = 2.617;
      }
      convCoordinate();
    }

    if(sendvalue == 'u' && count >= 100){
      //SerialUSB.print("u in");
      phi1 = phi1 - Add_anglev*D2R;
      if(phi1 <= -2.617){
        phi1 = -2.617;
      }
      convCoordinate();
    }

    if(sendvalue == 'r' && count >= 100){
      //SerialUSB.print("r in");
      angle = angle + Add_angleh*D2R;
      if(angle >= 2.617){
        angle = 2.617;
      }
      convCoordinate();
    }

    if(sendvalue == 'l' && count >= 100){
      //SerialUSB.print("l in");
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
  //Serial2.print('o');
  //Serial2.print('k');

  //Dynamixelへの命令の衝突を防ぐ
  while(Dxl.readByte(ID_NUM3, 46) != 0 ||  Dxl.readByte(ID_NUM3, 46) !=0){
  delay(10);
}


}
