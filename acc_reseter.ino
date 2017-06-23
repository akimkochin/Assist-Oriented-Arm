/* Minimum_Source*/

#define Timer_rate 250
#define analogPin0 0
#define analogPin1 1
#define analogPin2 2
#define Hori_X 319
#define Hori_Y 294
#define min_Y 147
#define min_X 171
#define onedeg_Y 1.45
#define onedeg_X 1.32
#define deg 30


int acc_X = 0;
int acc_Y = 0;
int acc_Z = 0;

int defference_Y = 0;
int defference_X = 0;

int degree_Y = 0;
int degree_X = 0;

int onetime_X = 0;
int onetime_Y = 0;

HardwareTimer Timer(1);

void setup() {
  Serial2.begin(57600);
  reset();
  checker();

  /*Timer割り込み*/

  Timer.pause();
  Timer.setPeriod(Timer_rate);
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1,1);
  Timer.attachInterrupt(TIMER_CH1, accSencer);
  Timer.refresh();
  Timer.resume();

}

void reset(){
  defference_Y = Hori_Y - analogRead(analogPin1) / 10;
  defference_X = Hori_X - analogRead(analogPin2) / 10;
}

void accSencer(){

  acc_Z = analogRead(analogPin0) / 10;//10分の1に省略する
  acc_Y = (analogRead(analogPin1) / 10) + defference_Y;
  acc_X = (analogRead(analogPin2) / 10) + defference_X;

  degree_Y = (acc_Y - min_Y) / onedeg_Y - 90;
  degree_X = (acc_X - min_X) / onedeg_X - 90;
}

void checker(){
  //average of value when read 100 times
  for(int count=0; count < 100; count++){
    acc_Y = (analogRead(analogPin1) / 10) + defference_Y;
    acc_X = (analogRead(analogPin2) / 10) + defference_X;
    onetime_Y = onetime_Y + (acc_Y - min_Y) / onedeg_Y - 90;
    onetime_X = onetime_X + (acc_X - min_X) / onedeg_X - 90;
  }
  onetime_X = onetime_X / 100;
  onetime_Y = onetime_Y / 100;
}
void loop() {
  unsigned char sendvalue1;
  unsigned char sendvalue2;
  // put your main code here, to run repeatedly:
  /*
  //Serial2.print('d');
  SerialUSB.print("x =");
  SerialUSB.print(acc_X);
  SerialUSB.print("degX =");
  SerialUSB.print(degree_X);
  SerialUSB.print("y =");
  SerialUSB.print(acc_Y);
  SerialUSB.print("degY =");
  SerialUSB.print(degree_Y);
  SerialUSB.print("z =");
  SerialUSB.println(acc_Z);
  SerialUSB.print("checker = ");
  SerialUSB.print(onetime_X);
  SerialUSB.print("checker = ");
  SerialUSB.print(onetime_Y);
  */
    //後ろ
  if((onetime_Y + 30) < degree_Y){
    Serial2.print('a');
    SerialUSB.println("aaa");
  }
  //前
  if((onetime_Y - 30) > degree_Y){
    Serial2.print('u');
    SerialUSB.println("uuu");
  }

  if((onetime_X - 30) > degree_X){
    Serial2.print('r');
    SerialUSB.println("rrr");
  }
  if((onetime_X + 30) < degree_X){
    Serial2.print('l');
    SerialUSB.println("lll");
  }
  if(Serial2.available()){
    while(1){
      sendvalue1 = Serial2.read();
      SerialUSB.print(sendvalue1);
      if(sendvalue1 == 'o'){
        break;
      }
    }
  }
}
