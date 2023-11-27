#include<SoftwareSerial.h>
SoftwareSerial BT(10,11); //Rx/Tx
void setup() {
  // put your setup code here, to run once:
  pinMode(9,OUTPUT);
  digitalWrite(9,HIGH);
  Serial.begin(38400);
  BT.begin(38400);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  if(BT.available())
    Serial.write(BT.read());
  if(Serial.available())
    BT.write(Serial.read());
}
