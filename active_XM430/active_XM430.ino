#include "define.h"

/*coordinate of active joints*/
float coordinate_x = 0.0;
float coordinate_y = 310.0;
float coordinate_z = 0.0;
/*length of links*/
float l1 = 40.0;
float l2 = 270.0;
/*angle of joints in active*/
float theta1 = 0.0;
float theta2 = 0.0;


int act_up_flag = 0;
int act_right_flag = 0;
int act_down_flag = 0;
int act_left_flag = 0;
int grip_close_flag = 0;
int grip_open_flag = 0;
int act_wrist_right_flag = 0;
int act_wrist_left_flag = 0;

int dx1_pos = 2048;
int dx2_pos = 2048;
int dx3_pos = 2048;
int dx4_pos = 2048;

float r = 270.0;

float angle = 90.0 * D2R;
float phy = 90.0 * D2R;

float abs_x = 0.0;
float abs_y = 0.0;
float abs_z = 0.0;

float abs_theta1 = 0.0;
float abs_theta2 = 0.0;
float tempL = 0.0;




Dynamixel Dxl(DXL_BUS_SERIAL3);

void setup() {
  // put your setup code here, to run once:
	// Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
	Dxl.begin(1);
	Dxl.setPacketType(DXL_PACKET_TYPE2);
	Dxl.writeByte(ID_NUM1,64, 1);  //Dynamixel Torque be changeing on
	Dxl.writeByte(ID_NUM2,64, 1);  //Dynamixel Torque be changeing on
	Dxl.writeByte(ID_NUM3,64, 1);  //Dynamixel Torque be changeing on
	Dxl.writeByte(ID_NUM4,64, 1);  //Dynamixel Torque be changeing on
	
	Dxl.writeByte(ID_NUM1,11, 3);
	Dxl.writeByte(ID_NUM2,11, 3);
	Dxl.writeByte(ID_NUM3,11, 3);
	Dxl.writeByte(ID_NUM4,11, 3);
	
	attachInterrupt(ACTIVE_UP, Condi_Active_UP, CHANGE);
	attachInterrupt(ACTIVE_DW, Condi_Active_DW, CHANGE);
	attachInterrupt(ACTIVE_R, Condi_Active_R, CHANGE);
	attachInterrupt(ACTIVE_L, Condi_Active_L, CHANGE);
	attachInterrupt(ACTIVE_3_R, Condi_A3R, CHANGE);
	attachInterrupt(ACTIVE_3_L, Condi_A3L, CHANGE);
	attachInterrupt(ACTIVE_GRIP_OP, Condi_GripOP, CHANGE);
	attachInterrupt(ACTIVE_GRIP_CL, Condi_GripCL, CHANGE);
	
	pinMode(0, INPUT_PULLDOWN);
	pinMode(1, INPUT_PULLDOWN);
	pinMode(2, INPUT_PULLDOWN);
	pinMode(3, INPUT_PULLDOWN);
	pinMode(4, INPUT_PULLDOWN);
	pinMode(5, INPUT_PULLDOWN);
	pinMode(6, INPUT_PULLDOWN);
	pinMode(7, INPUT_PULLDOWN);

	Dxl.writeDword(ID_NUM3, 116, 2048);
	Dxl.writeDword(ID_NUM4, 116, 814);
	Dxl.writeDword(ID_NUM1, 116, 2048);
	Dxl.writeDword(ID_NUM2, 116, 2048);
	


	

}


void loop() {
  // put your main code here, to run repeatedly: 
	
	//Kinematics();
	/*
	SerialUSB.print(coordinate_x);
	SerialUSB.print(",");
	SerialUSB.print(coordinate_y);
	SerialUSB.print(",");
	SerialUSB.println(coordinate_z);
	*/
	
	
	SerialUSB.print("theta1 = ");
	SerialUSB.print(theta1);
	SerialUSB.print(", theta2 = ");
	SerialUSB.println(theta2);
	
	T_coordinate();
	Inv_Kinemtaics();
	Dxl.writeDword(ID_NUM1, 112, 200);
	Dxl.writeDword(ID_NUM2, 112, 200);
	MoveDynamixel();
	Change_rad2Dynamixel_pos();
	
	Dxl.writeDword(ID_NUM1, 116, dx1_pos);
	Dxl.writeDword(ID_NUM2, 116, dx2_pos);

}


void MoveDynamixel(){
	
	//controling of Dynamixel ID 3
	if(act_wrist_right_flag == ON){
		dx3_pos += Increase;
		Dxl.writeDword(ID_NUM3, 116, dx3_pos); 
	}
	if(act_wrist_left_flag == ON){
		dx3_pos -= Increase; 
		Dxl.writeDword(ID_NUM3, 116, dx3_pos); 
	}
	//Controling of Dynamixel ID 4 (Gripper)
	if(grip_close_flag == ON){
		dx4_pos += Increase;
		if(dx4_pos >= 2861)
			dx4_pos = 2861;
		Dxl.writeDword(ID_NUM4, 116, dx4_pos); 
	}
	if(grip_open_flag == ON){
		dx4_pos -= Increase; 
		if(dx4_pos <= 814)
			dx4_pos = 814;
		Dxl.writeDword(ID_NUM4, 116, dx4_pos); 
	}
	
	if(act_up_flag == ON){
		angle += Add_angle;
	}
	if(act_down_flag == ON){
		angle -= Add_angle;
	}
	if(act_right_flag == ON){
		phy -= Add_angle;
	}
	if(act_left_flag == ON){
		phy += Add_angle;
	}
	
}

void Change_rad2Dynamixel_pos(){
	dx1_pos = theta1 * R2Dynamixel+2048;
	dx2_pos = theta2 * R2Dynamixel+2048;
	
}

void Condi_A3L(){
	if(digitalRead(ACTIVE_3_L)==ON){
		act_wrist_left_flag = ON;
	}else{
		act_wrist_left_flag = OFF;
	}	
}
void Condi_A3R(){
	if(digitalRead(ACTIVE_3_R)==ON){
		act_wrist_right_flag = ON;
	}else{
		act_wrist_right_flag = OFF;
	}	
}

void Condi_GripOP(){
	if(digitalRead(ACTIVE_GRIP_OP)==ON){
		grip_open_flag = ON;
	}else{
		grip_open_flag = OFF;
	}	
}

void Condi_GripCL(){
	if(digitalRead(ACTIVE_GRIP_CL)==ON){
		grip_close_flag = ON;
	}else{
		grip_close_flag = OFF;
	}	
}

void Condi_Active_UP(){
	if(digitalRead(ACTIVE_UP)==ON){
		act_up_flag = ON;
	}else{
		act_up_flag = OFF;
	}	
}

void Condi_Active_DW(){
	if(digitalRead(ACTIVE_DW)==ON){
		act_down_flag = ON;
	}else{
		act_down_flag = OFF;
	}	
}

void Condi_Active_R(){
	if(digitalRead(ACTIVE_R)==ON){
		act_right_flag = ON;
	}else{
		act_right_flag = OFF;
	}	
}

void Condi_Active_L(){
	if(digitalRead(ACTIVE_L)==ON){
		act_left_flag = ON;
	}else{
		act_left_flag = OFF;
	}	
}


/*
void Kinematics(){
	
	coordinate_x = l2 * cos(theta1) * sin(theta2);
	coordinate_y = l1 + l2 * cos(theta2);
	coordinate_z = l2 * sin(theta1) * sin(theta2);
	
}*/

void Inv_Kinemtaics(){
	
	/************theta1**************/
	theta1 = atan2(coordinate_z, coordinate_x);
	
	if(coordinate_x == 0.0 && coordinate_z == 0.0){   //if x,z is 0, assigned 0 for theta1.
		theta1 = 0.0;
	}
	/*
	if(coordinate_z == 0.0){  //if x is 0, assigned 2 / pi for theta1.
		theta1 = PI / 2;
	}
	
	if(coordinate_x == 0.0){  //if z is 0, assigned 0 for theta1.
		theta1 = 0.0;
	}
	*/
	
	
	abs_theta1 = abs(theta1);
	if(theta1 >= 3.0){
		theta1 = 0.0;
	}
	
	if(theta1 < 0){
		abs_theta1 = abs(theta1);
		theta1 = abs_theta1;
	}
	
	
	/**********theta2*************/
	if(coordinate_x == 0.0){
		theta2 = atan2(coordinate_z, coordinate_y - l1);
	}else if(coordinate_z == 0.0){
		theta2 = atan2(coordinate_x, coordinate_y - l1);
	}else{
		tempL = sqrt(coordinate_x * coordinate_x + coordinate_z * coordinate_z);
		theta2 = atan2(tempL, coordinate_y - l1);
		if(coordinate_x < 0 && coordinate_z < 0){
			 
		}else{
			theta2 *= -1; 
		}
	}
	
	abs_theta2 = abs(theta2);
	if(abs_theta2 >= 1.57){
		if(theta2 > 0)
			theta2 = 1.57;
		if(theta2 < 0)
			theta2 = -1.57;
	}
	
}



void T_coordinate(){
	
	coordinate_x = r * sin(angle) * cos(phy);
	coordinate_y = r * sin(angle) * sin(phy) + l1;
	coordinate_z = r * cos(angle);
	abs_x = abs(coordinate_x);
	abs_y = abs(coordinate_y);
	abs_z = abs(coordinate_z);
	
	if(abs_x <= 0.001){
		coordinate_x = 0.0;
	}
	if(abs_y <= 0.001){
		coordinate_y = 0.0;
	}
	if(abs_z <= 0.001){
		coordinate_z = 0.0;
	}
}

