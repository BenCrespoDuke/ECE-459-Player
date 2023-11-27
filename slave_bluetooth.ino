#include <ServoTimer2.h>

//slave

#include <SoftwareSerial.h> 
#define button 8
SoftwareSerial MyBlue(10, 11); // RX | TX 

ServoTimer2 servo1;

char state = 0;
//int buttonState = 0;
void setup() {
  pinMode(button, INPUT);
  Serial.begin(38400);
  MyBlue.begin(38400);
  

}

void loop() {
//  digitalWrite(button, HIGH);
//  if(MyBlue.available() > 0){
//    state = MyBlue.read();
//  }
  
  //buttonState = digitalRead(button);
//  if(buttonState == HIGH) {
//    MyBlue.write('1');
//    Serial.print('1');
//  }
//  else {
//    MyBlue.write('0');
//    Serial.print('0');
//  }

  //MyBlue.write("HELLO");

    //ask for prompt
  if(MyBlue.available() > 0){
    state = MyBlue.read();
  }
  if(state == '1'){
    //Serial.print("state 1");
    MyBlue.write('1');
    delay(100);
    MyBlue.write('E');
    delay(100);
  }
  else if(state == '2'){
    MyBlue.write('2');
    delay(100);
    MyBlue.write('E');
    delay(100);
  }
}
