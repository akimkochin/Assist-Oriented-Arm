/* define header file*/

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define ID_NUM1 1
#define ID_NUM2 2
#define ID_NUM3 3
#define ID_NUM4 4
/* caluculation of fundamental items*/
//#define Pi 3.14159265359

#define ON 1
#define OFF 0
#define D2R PI / 180
#define R2D 180 / PI
#define R2Dynamixel 652.0

#define Add_angle 2.0 * D2R
#define Add_phy 2.0 * D2R

#define Increase 30
#define Timer_rate 100


#define ACTIVE_3_L 7
#define ACTIVE_3_R 6
#define ACTIVE_GRIP_OP 5
#define ACTIVE_GRIP_CL 4
#define ACTIVE_UP 0
#define ACTIVE_DW 2
#define ACTIVE_R 1
#define ACTIVE_L 3