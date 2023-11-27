//master
#include <SoftwareSerial.h> 
#include "ArrayList.h"
#define USE_TIMER_2 true
#define USE_TIMER_3 true
#include "TimerInterrupt.h"
#define TIMER2_INTERVAL_MS  2
#define TIMER3_INTERVAL_MS  200
#include <Servo.h>

union conv32
{
  uint32_t  u32;
  float     f32;
};

int LED = 8;
String data = "";
SoftwareSerial MyBlue(10, 11); //RX/TX
SoftwareSerial MyBlue1(12, 13); //RX/TX
boolean data1_processing = false;
boolean data2_ready = false;
ArrayList<char> data1;
ArrayList<char> data2;

ArrayList<char> buffer1;
ArrayList<char> buffer2;

ArrayList<char> slave1_queue;
ArrayList<char> slave2_queue;
int num = 0;

boolean slave1 = false;
boolean slave2 = false;

//char prev_right_1;
//char prev_left_1;

ArrayList<char> prevPacket1;

ArrayList<char> prevPacket2;

char prev_left_p1;
char prev_right_p1;

Servo right1;
Servo left1;
Servo x1;
Servo y1;

Servo right2;
Servo left2;
Servo x2;
Servo y2;


void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(38400);
  MyBlue.begin(38400);
  MyBlue1.begin(38400);
  Serial.print("STUFF");
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
  pinMode(12, INPUT);
  pinMode(13, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(21), readInData, RISING);
  ITimer2.init();
  ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, readInData);

  ITimer3.init();
  ITimer3.attachInterruptInterval(TIMER3_INTERVAL_MS, activate_slave1);
  MyBlue.listen();

  right1.attach(2);
  left1.attach(3);
  x1.attach(4);
  y1.attach(5);

  right2.attach(6);
  left2.attach(7);
  x2.attach(8);
  y2.attach(9);

  prev_left_p1 = -16;
  prev_right_p1 = -16;
  for(int i = 0; i < 15; i++){
    prevPacket1.add(-16);
  }
  for(int i = 0; i < 15; i++){
    prevPacket2.add(-16);
  }
  
}

void loop() {
  //rightPunchSlave1(1);
  if(MyBlue.overflow()){
    while(MyBlue.available()){
      MyBlue.read();
    }
  }
  //Serial.println(buffer1.size());
  if(data2_ready){
    MyBlue.listen();
    data2_ready = false;
    
    ArrayList<char> right_data;
    ArrayList<char> left_data;
    ArrayList<char> center_data;
    //convert data1 to float
    uint32_t slave2_X = 0;
    uint32_t slave2_Y = 0;
    

    for(int i = 7; i < 11; i++){
      slave2_X = (slave2_X << 8) | (data2[i] & 0xFF);
    }
    for(int i = 12; i < 16; i++){
      slave2_Y = (slave2_Y << 8) | (data2[i] & 0xFF);
    }
    
    Serial.print("R: ");
    Serial.println((int)data2[1]);
    Serial.print("L: ");
    Serial.println((int)data2[3]);
    Serial.print("C: ");
    Serial.println((int)data2[5]);


//    rightPunchSlave2(data2[1]);
//    leftPunchSlave2(data2[3]);

    union conv32 x = {.u32 = slave2_X};
    float slave2_x_float = x.f32;
    union conv32 y = {.u32 = slave2_Y};
    float slave2_y_float = y.f32;
    Serial.print("X: ");
    Serial.println(slave2_x_float);
    Serial.print("Y: ");
    Serial.println(slave2_y_float);
    
    clearArray(data2);
  }
}

void readInData() {
   if(data1_processing){
    return;
   }
   //Serial.print("x");
   while(MyBlue.available() >= 17){
      char state = MyBlue.peek();
      if(state == 'A'){
        //Serial.println("New Packet");
        for(int i = 0; i<17; i++){
        
          state = MyBlue.read();
          data1.add(state);
//          Serial.print((int)state);
//          Serial.print(" ");
//          Serial.print(state);
//          Serial.print("    ");
        }
        process_data1();
      }
      else{
        MyBlue.read();
      }
   }
   /*while(MyBlue.available()){
      //Serial.println("myblue available");
      if(!data1_processing){
          char state = MyBlue.read();
          for(int i = 0; i < 17; i++){
            data1.add(MyBlue.read());
          }
          process_data1();
      }*/
      
      //Serial.print(state);
      //if((state == 'E')){
//        //data1_ready = true;
//        slave1 = false;
//        if(!data1_processing){
//          for(int i = 0; i < 15; i++){
//            data1.add(buffer1[0]);
//            buffer1.removeAt(0);
//          }
//          process_data1();
//        }
//        
//      }
//      else if(state == 'A'){
//        slave1 = true;
//      }
//      else if(slave1){
//        buffer1.add(state); 
//      }
      
    //}
   //}
   if(MyBlue1.available()){
     // Serial.println("myblue1 available");
      char state = MyBlue1.read();
      if(state == 'E'){
        data2_ready = true; 
        slave2 = false;
      }
      else if(state == 'B'){
        slave2 = true;
      }
      else if(slave2){
        data2.add(state);
      }
    }
}

template<typename T>
void printArray(ArrayList<T> &list) {
   for (int i = 0; i < list.size(); i++) {
       //Serial.print("[" + String(i) + "]: ");
       Serial.print(list[i]);
   }
   Serial.println();
}

template<typename T>
void clearArray(ArrayList<T> &list) {
   while (list.size() > 0) {
      list.removeAt(0);
   }
}

void leftPunchSlave1(char input) {
  if(input == 15){
    left1.write(180);
    //Serial.print("pwm left slave 1");
  }
  else if(input == -16){
    left1.write(0);
    //Serial.print("pwm left slave 1");
  }
  
}

void rightPunchSlave1(char input) {
  if(input == 15){
    right1.write(180);
    Serial.println("pwm right slave 1");
  }
  else if(input == -16){
    right1.write(0);
    Serial.println("pwm right slave 1");
  }
  //processVelocity();
}

void moveSlave1(){
  Serial.print("pwm move slave 1");
  x1.write(50);
  y1.write(50);
  processVelocity();
}

void processVelocity(){
  Serial.print("processing velocity");
}


void activate_slave1(){
  if(slave1_queue.size() > 0){
    if(slave1_queue[0] == 'R'){
      rightPunchSlave1(slave1_queue[1]);
      slave1_queue.removeAt(0);
      slave1_queue.removeAt(0);
    }
    else if(slave1_queue[0] == 'L'){
      leftPunchSlave1(slave1_queue[1]);
      slave1_queue.removeAt(0);
      slave1_queue.removeAt(0);
    }
  }
}

void activate_slave2(){
  
}

void process_data1(){
  data1_processing = true;
  //MyBlue1.listen();
  
  //convert data1 to float
  uint32_t slave1_X = 0;
  uint32_t slave1_Y = 0;
  

  for(int i = 7; i < 11; i++){
    slave1_X = (slave1_X << 8) | (data1[i] & 0xFF);
  }
  for(int i = 12; i < 16; i++){
    slave1_Y = (slave1_Y << 8) | (data1[i] & 0xFF);
  }
  
  //Serial.println(data1.size());
//    Serial.print("R: ");
//    Serial.print((int)data1[2]);
//    Serial.print(data1[3]);
//    Serial.print(": ");
//    Serial.print((int)data1[4]);
//    Serial.print(" C: ");
//    Serial.println((int)data1[6]);
  
  if(((data1[2] == 15) && (prev_right_p1 == -16)) || ((data1[2] == -16) && (prev_right_p1 == 15))){
    slave1_queue.add('R');
    slave1_queue.add(data1[2]);
    Serial.print('R');
    Serial.println((int)data1[2]);
    prev_right_p1 = data1[2];
    //data1[1] = 0;
  }

  if(((data1[4] == 15) && (prev_left_p1 == -16)) || ((data1[4] == -16) && (prev_left_p1 == 15))){
    slave1_queue.add('L');
    slave1_queue.add(data1[4]);
    Serial.print('L');
    Serial.println((int)data1[4]);
    prev_left_p1 = data1[4];
    //data1[3] = 0;
  }
  
  
  union conv32 x = {.u32 = slave1_X};
  float slave1_x_float = x.f32;
  union conv32 y = {.u32 = slave1_Y};
  float slave1_y_float = y.f32;
  Serial.print("X: ");
  Serial.println(slave1_x_float);
  Serial.print("Y: ");
  Serial.println(slave1_y_float);
  
  clearArray(data1);

  data1_processing = false;
}
