/* Minimum_Source*/

#define DXL_BUS_SERIAL1 1
#define GOAL_SPEED 32
#define CONTROL_MODE 11
#define CCW_Angle_Limit 8
#define DXL_BUS_SERIAL1 1
#define P_DX1 1
#define P_DX2 2
#define breaking 16
#define Torque 24
#define ON 1
#define OFF 0
#define GOAL_POSITION 30

int breaking_state;
Dynamixel Dxl(DXL_BUS_SERIAL1);
void setup() {
  // put your setup code here, to run once:
	Dxl.begin(3);
	Dxl.jointMode(P_DX1);
	Dxl.jointMode(P_DX2); //jointMode() is to use position mode
    Dxl.writeByte(P_DX1, Torque, OFF);
    Dxl.writeByte(P_DX2, Torque, OFF);
    
	attachInterrupt(breaking, Brake_ON, FALLING);
	
	
	pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(breaking, INPUT_PULLDOWN);

}

void Brake_ON(){
	
	Dxl.writeWord(P_DX1, GOAL_POSITION, 400); 
    Dxl.writeWord(P_DX2, GOAL_POSITION, 500);
	delay(500);
	
}



void Torque_OFF(){
	Dxl.writeWord(P_DX1, Torque, OFF);
    Dxl.writeWord(P_DX2, Torque, OFF);
}

void loop() {
  // put your main code here, to run repeatedly: 
  Torque_OFF();
  breaking_state = digitalRead(breaking);
  if(breaking_state==HIGH){
	Dxl.writeWord(P_DX1, GOAL_POSITION, 1000); 
    Dxl.writeWord(P_DX2, GOAL_POSITION,1010);
	delay(500);
  }


}

