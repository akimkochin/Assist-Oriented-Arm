/* Minimum_Source*/

#define Timer_rate 250
#define analogPin2 2
#define analogPin3 3
#define analogPin4 4

int acc_X = 0;
int acc_Y = 0;
int acc_Z = 0;

HardwareTimer Timer(1);
unsigned char sendvalue = 0;

void setup() {
  Serial2.begin(57600); 


/*
  // put your setup code here, to run once:
  pinMode(analogPin2, INPUT_ANALOG);
  pinMode(analogPin3, INPUT_ANALOG);
  pinMode(analogPin4, INPUT_ANALOG);

  /*Timer割り込み*/
  Timer.pause();
  Timer.setPeriod(Timer_rate);
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1,1);
  Timer.attachInterrupt(TIMER_CH1, accSencer);
  Timer.refresh();
  Timer.resume();
}

void accSencer(){

  acc_X = analogRead(2);
  acc_Y = analogRead(3);
  acc_Z = analogRead(4);
}

void loop() {
  // put your main code here, to run repeatedly: 

  //Serial2.print('d');
 /* 
  SerialUSB.print("x =");
  SerialUSB.print(acc_X);
  SerialUSB.print("y =");
  SerialUSB.print(acc_Y);
  SerialUSB.print("z =");
  SerialUSB.println(acc_Z);
*/

//後ろ
  if(acc_Y < 3500 && 3400 < acc_X < 3500 && acc_Z > 4000){
    Serial2.print('a');
    SerialUSB.println("aaa");
  }
//前
  if(acc_Y == 4095 && 3400 < acc_X < 3500 && acc_Z > 4000){
    Serial2.print('u');
    SerialUSB.println("uuu");
  }

  if(acc_X > 3600 && 3700 < acc_Y < 3800 && acc_Z > 4000){
 	  Serial2.print('r');
    SerialUSB.println("rrr");
  }
  if(acc_X < 2800 && 3600 < acc_Y < 3700 && acc_Z > 4000){
  	Serial2.print('l');
    SerialUSB.println("lll");
  }
  
}

