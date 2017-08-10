/* Minimum_Source*/

#define DXL_BUS_SERIAL1 1
#define GOAL_SPEED 32
#define CONTROL_MODE 11
#define CCW_Angle_Limit 8
#define DXL_BUS_SERIAL1 1
#define P_DX1 1
#define P_DX2 2
#define breaking 16
int breaking_state;
Dynamixel Dxl(DXL_BUS_SERIAL1);
void setup() {
  // put your setup code here, to run once:
	Dxl.begin(3);
	Dxl.jointMode(P_DX1);
	Dxl.jointMode(P_DX2); //jointMode() is to use position mode

    pinMode(breaking, INPUT_PULLDOWN);

}

void loop() {
  // put your main code here, to run repeatedly: 
 breaking_state = digitalRead(breaking);

    /*Turn dynamixel ID 2 to position 0*/
 if(breaking_state==HIGH){
    Dxl.goalPosition(P_DX1, 1000); 
    Dxl.goalPosition(P_DX2, 1010); 
    // Wait for 1 second (1000 milliseconds)
    delay(500); 
 }
 if(breaking_state==LOW){
    /*Turn dynamixel ID 2 to position 300*/
    Dxl.goalPosition(P_DX1, 400); 
    Dxl.goalPosition(P_DX2, 500);
    // Wait for 1 second (1000 milliseconds)
    delay(500);   
 }
/*
    breaking_state = digitalRead(breaking);
    if(breaking_state==HIGH){
       //Dxl.setPosition(ID[0],0,100);
       //Dxl.setPosition(ID[1],210,100);
       //new prototype program
        Dxl.goalPosition(P_DX1, 1010); 
        Dxl.goalPosition(P_DX2, 1010);
       delay(1000); 
        SerialUSB.println("BREAK_OFF");
    }
    if(breaking_state==LOW){
     Dxl.goalPosition(P_DX1, 1010); 
     Dxl.goalPosition(P_DX2, 1010); 
            delay(1000); 

        SerialUSB.println("BREAK_ON");
    }
  */


}

