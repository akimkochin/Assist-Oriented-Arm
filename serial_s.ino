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
 unsigned char sendvalue = 0;////unsign char

 void setup(){
 	Serial2.begin(57600);  
 	pinMode(16, OUTPUT);


 }

 void loop(){

 	if(Serial2.available() > 0){
 		sendvalue = Serial2.read();
 		//Serial2.print(sendvalue);
 		if(sendvalue == 'a'){
 			digitalWrite(16,HIGH);
 			SerialUSB.println("out put the power for LED");
 		}else if(sendvalue == 'e'){
 			digitalWrite(16,LOW);
 		}else{
 			SerialUSB.println("pass-code is different");
 		}
 	}  
 }

