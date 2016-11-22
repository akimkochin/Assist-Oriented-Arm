      /* Button
       
       OpenCM9.04 has built-in button which is called as user button
       User button is defined previously as BOARD_BUTTON PIN, so just call it.
       This example shows how to use user button and status LED(built-in LED)
       When button is pressed, status LED turn on, when released LED turn off
       
                        Compatibility
       CM900                  X
       OpenCM9.04             O

       created 16 Nov 2012
       by ROBOTIS CO,.LTD.
       */

#define DXL_BUS_SERIAL1 1
#define DXL_BUS_SERIAL3 3
#define GOAL_SPEED 32
#define CONTROL_MODE 11
#define CCW_Angle_Limit 8

Dynamixel Dxl(DXL_BUS_SERIAL3);
int ID[5] = {1,2,3,4,5};
        // int turnR = 0;
        // int turnL = 1;
        // int shift = 2;
int breaking = 22;
unsigned char sendvalue = 0;

  void setup(){

      // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps
    Serial2.begin(57600);
    Dxl.begin(3);
    for(int i = 2; i <= 3; i++){
      Dxl.jointMode(ID[i]);
    }
    Dxl.writeWord(5, CCW_Angle_Limit, 0);
    
    Dxl.writeByte(ID[0], CONTROL_MODE, 1);
    Dxl.writeByte(ID[1], CONTROL_MODE, 1);
    Dxl.writeWord(ID[0], CCW_Angle_Limit, 0);
    Dxl.writeWord(ID[1], CCW_Angle_Limit, 0);
           //jointMode() is to use position mode
        /*  
         BOARD_BUTTON_PIN is needed to pull-down circuit 
         for operationg as digital switch fully.
         */
        //pinMode(1,INPUT_PULLUP);
        // pinMode(turnR, INPUT_PULLDOWN); //initial  state is LOW
        // pinMode(turnL, INPUT_PULLDOWN);
        // pinMode(shift, INPUT_PULLDOWN);
    pinMode(breaking, INPUT_PULLDOWN);
  }

    void loop(){
      SerialUSB.println("loopnow");
        // read the state of the pushbutton value:
        // int turnR_state = digitalRead(turnR); 
        // int turnL_state = digitalRead(turnL);
        // int shift_state = digitalRead(shift);
        int breaking_state = digitalRead(breaking);
        int Active_num = 2;
        int start_position[3];
        int start_positionH[3];
        int j = 0;
        int flag = 0;

        for(int i=2; i<=4; i++){
          int x = (Dxl.readByte(ID[i], 37) << 8 )  + Dxl.readByte(ID[i], 36);
          if(0 <= x && x <= 1024){  
            start_position[j] = x;
            j++;
          }
        SerialUSB.println(i);
        }
  
        j = 0;

         //int value = digitalRead(1);
          //if(value == LOW) SerialUSB.println("hello");
      Dxl.writeWord(5, GOAL_SPEED, 0);
      while(1){
        // turnR_state = digitalRead(turnR);
        // turnL_state = digitalRead(turnL);
        // shift_state = digitalRead(shift);
        breaking_state = digitalRead(breaking);
        SerialUSB.println("Whilenow");
        
        Dxl.writeWord(5, GOAL_SPEED, 0);
        if(Serial2.available() > 0){
           sendvalue = Serial2.read();

          if(sendvalue == 'r'){ //if button is pushed, means 3.3V(HIGH) is connected to BOARD_BUTTON_PIN
              if(Active_num == 4){
                  Dxl.writeWord(5, GOAL_SPEED, 1020);
              }
              else{
                start_position[j] -= 20;
                Dxl.setPosition(ID[Active_num],start_position[j],100);
            
                SerialUSB.println("RIGHT_HIGH");
                SerialUSB.println(start_position[j]);
              }
            delay(100);

          }else if(sendvalue == 'l'){
            if(Active_num == 4){
                  Dxl.writeWord(5, GOAL_SPEED, 1020 | 0x400);
            }
            else{
                start_position[j] += 20;
                Dxl.setPosition(ID[Active_num],start_position[j],100);
                SerialUSB.println("LEFT_HIGH");
            }
          delay(100);

          }else if(sendvalue == 'c'){
            // for(int check = sendvalue;check != 'n';){
            //   check = Serial2.read();
            //   SerialUSB.println("bbbbbbbbbbbbbbbbbbbbb");
              
            // }
            if(j==2 && Active_num==4){
              //Active_num = 2;
              //j = 0;
              SerialUSB.println("omedeto!!!!");
              break;
            }
            Active_num++;
            j++;
            SerialUSB.println("SHIFT_HIGH");

          }
          breaking_state = digitalRead(breaking);
        
           if(breaking_state==HIGH){
             //Dxl.setPosition(ID[0],0,100);
             //Dxl.setPosition(ID[1],210,100);
             //new prototype program
             Dxl.writeWord(ID[0], GOAL_SPEED, 400);
             Dxl.writeWord(ID[1], GOAL_SPEED, 400);
             delay(3000);
             SerialUSB.println("BREAK_OFF");
             for(int count = 0;breaking_state == HIGH;count++){
               Dxl.writeWord(ID[0], GOAL_SPEED, 0);
               Dxl.writeWord(ID[1], GOAL_SPEED, 0);
               breaking_state = digitalRead(breaking);
               //SerialUSB.println(count);
             }
           Dxl.writeWord(ID[0], GOAL_SPEED, 400 | 0x400);
           Dxl.writeWord(ID[1], GOAL_SPEED, 400 | 0x400);
           delay(3500);
           Dxl.writeWord(ID[0], GOAL_SPEED, 0);
           Dxl.writeWord(ID[1], GOAL_SPEED, 0);
           SerialUSB.println("reverse sucsess");
           }
  
        }
      
        // }else if(breaking_state==LOW){
        //   Dxl.setPosition(ID[0],96,100);
        //   Dxl.setPosition(ID[1],300,100);
        //     //SerialUSB.println("BREAK_ON");

        // }else{
        //   SerialUSB.print("aaaaaa\n");
        // }
        
      }
    }



