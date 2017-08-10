/* Minimum_Source*/

#define DXL_BUS_SERIAL1 1
#define GOAL_SPEED 32
#define CONTROL_MODE 11
#define CCW_Angle_Limit 8
#define DXL_BUS_SERIAL1 1
#define P_DX1 1
#define P_DX2 2
#define breaking 22
int breaking_state;
Dynamixel Dxl(DXL_BUS_SERIAL1);
void setup() {
  // put your setup code here, to run once:
	Dxl.begin(3);
	Dxl.wheelMode(P_DX1);
	Dxl.wheelMode(P_DX2); //jointMode() is to use position mode

    pinMode(breaking, INPUT_PULLDOWN);

}

void loop() {
  // put your main code here, to run repeatedly: 

    breaking_state = digitalRead(breaking);
    if(breaking_state==HIGH){
       //Dxl.setPosition(ID[0],0,100);
       //Dxl.setPosition(ID[1],210,100);
       //new prototype program
    	Dxl.writeWord(P_DX1, GOAL_SPEED, 1023);
    	Dxl.writeWord(P_DX2, GOAL_SPEED, 1023);
        delay(2500);
        SerialUSB.println("BREAK_OFF");
		for(int count = 0;breaking_state == HIGH;count++){
            Dxl.writeWord(P_DX1, GOAL_SPEED, 0);
            Dxl.writeWord(P_DX2, GOAL_SPEED, 0);
            breaking_state = digitalRead(breaking);
            //SerialUSB.println(count);
        }
        Dxl.writeWord(P_DX1, GOAL_SPEED, 1023 | 0x400);
        Dxl.writeWord(P_DX2, GOAL_SPEED, 1023 | 0x400);
        delay(2500);
        Dxl.writeWord(P_DX1, GOAL_SPEED, 0);
        Dxl.writeWord(P_DX2, GOAL_SPEED, 0);
        SerialUSB.println("reverse sucsess");
    }
  


}

