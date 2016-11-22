/*SerialUSB_Serial2_Converter
 
 This example is convert from serial2 to USB.
 CM-900, OpenCM9.04 has a port(J9) connected directly to Serial2.
 If some data is coming from Serial2, it is sent to serialUSB.
 On the contrary, all data coming from serialUSB is sent to Serial2.
 
 
 You can connect the below products to J9 Connector in CM-900, OpenCM9.04
 [BT-110A] or [BT-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=875
 [ZIG-110A Set]
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=405
 [LN-101] USART communication and download tool in CM-100
 http://www.robotis-shop-kr.com/goods_detail.php?goodsIdx=348
 
 You can also find all information about ROBOTIS products
 http://support.robotis.com/
 
                   Compatibility
 CM900                  O
 OpenCM9.04             O

 created 16 Nov 2012
 by ROBOTIS CO,.LTD.
 */
 int switchL=13;
 int switchC=23;
 int switchR=18;
 int leftstate=0;
 int centerstate=0;
 int rightstate=0;

 void setup(){
 	Serial2.begin(57600);  
 	pinMode(switchL, INPUT_PULLDOWN);
 	pinMode(switchC, INPUT_PULLDOWN);
 	pinMode(switchR, INPUT_PULLDOWN);

 }

 void loop(){
 	leftstate = digitalRead(switchL);
 	centerstate = digitalRead(switchC);
 	rightstate = digitalRead(switchR);
 	if(leftstate == HIGH){
 		Serial2.print('l');
 		SerialUSB.println("send a pass-code");
 	}
 	else if(leftstate == LOW){
 		Serial2.print('e');
 		SerialUSB.println("send a end pass-code");
 	}

 	if(centerstate == HIGH){
 		Serial2.print('c');
 		SerialUSB.println("send a pass-code");
 	}
 	else if(centerstate == LOW){
 		Serial2.print('n');
 		SerialUSB.println("send a end pass-code");
 	}

 	if(rightstate == HIGH){
 		Serial2.print('r');
 		SerialUSB.println("send a pass-code");
	}
	else if(centerstate == LOW){
 		Serial2.print('d');
 		SerialUSB.println("send a end pass-code");
 	}
 	delay(100);
}
