#define right_hand_mux_addr 0x80
#define left_hand_mux_addr 0x10
#define button 6
#define grav 9.807
#define USE_TIMER_1     true
#include "TimerInterrupt.h"
#define TIMER1_INTERVAL_MS    100
#include <SoftwareSerial.h>
#include <Wire.h>
//#include "ArrayList.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#define print_hand_movements
#define player1
byte test_num = 0;
//#define player2

//enum hand_state{ret,ext_a,ext_d,ext,ret_1,ret_a,ret_d};
enum hand_state{ret,ext_a,ext,ret_a};
enum hand_state right_hand_state; // assuming we start with hand retracted
enum hand_state left_hand_state; // assuming we start with hand retracted


Adafruit_ICM20948 hand; //USE I2C Mux to Switch
Adafruit_ICM20948 body;
Adafruit_Sensor *icm_accel_left;
Adafruit_Sensor *icm_accel_right;
char bluetooth_data[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char prev_left = -16;
char prev_right = -16;

float body_vel_x = 0.0;
float body_vel_z = 0.0;
float body_ang = 0.0;
float body_ang_offset = 0.0;
float right_vel = 0.0;
float left_vel = 0.0;

bool punch_left = false;
bool punch_right = false;
bool update_left = true;
bool update_right = true;
bool pull_left = false;
bool pull_right = false;


float accel_z_past_raw_right[4] = {0,0,0,0};
float accel_z_past_in_right[4] = {0,0,0,0};
float accel_z_past_out_right[4] = {0,0,0,0};


float accel_z_past_raw_left[4] = {0,0,0,0};
float accel_z_past_in_left[4] = {0,0,0,0};
float accel_z_past_out_left[4] = {0,0,0,0};

float accel_x_past_raw_body[4] = {0,0,0,0};
float accel_x_past_in_body[4] = {0,0,0,0};
float accel_x_past_out_body[4] = {0,0,0,0};


float punch_iir_a[5] = {1,-3.26231,4.04702664,-2.2564272,0.4762797};
float punch_iir_b[5] = {0.00028314, 0.00113258, 0.00169887, 0.00113258, 0.00028314};


union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};


float irr_filt(float input, float past_in[], float past_out[],float a[],float b[],int order){
  float sum = b[0]*input;
  for(int i = 0; i<order; i++){
    sum = sum+b[i+1]*past_in[i]-a[i+1]*past_out[i];
  }
  
  return (sum * (1/a[0]));
}

SoftwareSerial bluetooth(3,2);
int state = 20;
int buttonState = 0;


void body_calcs(sensors_event_t accel,sensors_event_t mag,float accel_x_past_raw[], float accel_x_past_in[],float accel_x_past_out[]){
  float accel_x_avg = (accel.acceleration.x*0.2+accel_x_past_raw[0]*.2+accel_x_past_raw[1]*.2+accel_x_past_raw[2]*.2+accel_x_past_raw[3]*.2);
  float accel_x_comp = accel.acceleration.x-accel_x_avg;
  accel_x_past_raw[3] = accel_x_past_raw[2];
  accel_x_past_raw[2] = accel_x_past_raw[1];
  accel_x_past_raw[1] = accel_x_past_raw[0];
  accel_x_past_raw[0] = accel.acceleration.x;
  float vel_delta_x = irr_filt(accel_x_comp, accel_x_past_in, accel_x_past_out,punch_iir_a,punch_iir_b,4);
  body_vel_x = body_vel_x+vel_delta_x;
  body_ang = mag.magnetic.z-body_ang_offset;
 buttonState = digitalRead(button);
 /*if(buttonState == HIGH){
  Serial.print(vel_delta_x);
  Serial.print(",");
 }*/
 

  //Store prior inputs and outputs for filter
    accel_x_past_out[3] =  accel_x_past_out[2];
    accel_x_past_out[2] =  accel_x_past_out[1];
    accel_x_past_out[1] =  accel_x_past_out[0];
    accel_x_past_out[0] =  vel_delta_x;
    
    accel_x_past_in[3] =  accel_x_past_in[2];
    accel_x_past_in[2] =  accel_x_past_in[1];
    accel_x_past_in[1] =  accel_x_past_in[0];
    accel_x_past_in[0] =  accel_x_comp;

}

enum hand_state hand_calcs(sensors_event_t accel,enum hand_state hand,float accel_z_past_raw[], float accel_z_past_in[],float accel_z_past_out[], char hand_char){
  
    //Filter that gets rid of Gravity
    float accel_z_avg = (accel.acceleration.z*0.2+accel_z_past_raw[0]*.2+accel_z_past_raw[1]*.2+accel_z_past_raw[2]*.2+accel_z_past_raw[3]*.2);
    float accel_z_comp = accel.acceleration.z-accel_z_avg;
    accel_z_past_raw[3] = accel_z_past_raw[2];
    accel_z_past_raw[2] = accel_z_past_raw[1];
    accel_z_past_raw[1] = accel_z_past_raw[0];
    accel_z_past_raw[0] = accel.acceleration.z;
    
    buttonState = digitalRead(button);
    
    
    //Low Pass IIR Filter
    float decsion_num = irr_filt(accel_z_comp, accel_z_past_in, accel_z_past_out,punch_iir_a,punch_iir_b,4);
    float vel = 0.0;
    //Serial.println(hand);
    if(hand_char =='R'){
      vel  = right_vel+decsion_num;
      right_vel  = right_vel+decsion_num;
      
       
    } else {
      vel  = left_vel+decsion_num;
      left_vel  = left_vel+decsion_num;
      
    }

   // left_vel  = left_vel+decsion_num;
   // vel  = left_vel+decsion_num;

   /* if(buttonState  == HIGH){
      Serial.print(right_vel);
      Serial.print(","); 
    }*/
   
    //Store prior inputs and outputs for filter
    accel_z_past_out[3] =  accel_z_past_out[2];
    accel_z_past_out[2] =  accel_z_past_out[1];
    accel_z_past_out[1] =  accel_z_past_out[0];
    accel_z_past_out[0] =  decsion_num;
    
    accel_z_past_in[3] =  accel_z_past_in[2];
    accel_z_past_in[2] =  accel_z_past_in[1];
    accel_z_past_in[1] =  accel_z_past_in[0];
    accel_z_past_in[0] =  accel_z_comp;
    
    if(vel <-50.0){
      if(hand == ret){
         /* if(hand_char =='R'){
            punch_right = true;
          } else {
             punch_left = true;
          }*/
         hand =  ext_a;
         #ifdef print_hand_movements
          Serial.println(hand);
         #endif
      }
    } else if(vel>25.0){
      if(hand == ext){
       /*if(hand_char =='R'){
            pull_right = true;
          } else {
             pull_left = true;
          }*/
        hand = ret_a;
        #ifdef print_hand_movements
        Serial.println(hand);
       #endif
      }
    } else if( abs(vel) < 5){
      if(hand == ret_a){
       /*if(hand_char =='R'){
            update_right = true;
          } else {
             update_left = true;
          }*/
        hand = ret;
        #ifdef print_hand_movements
       Serial.println(hand);
       #endif
      } else if(hand == ext_a){
       /* if(hand_char =='R'){
            update_right = true;
          } else {
             update_left = true;
          }*/
        hand = ext;
        #ifdef print_hand_movements
       Serial.println(hand);
       #endif
      }
    }
    
     return hand;
     
    
}

void imu_calcs(){
   
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    switch_right();
    unsigned long t1 = micros();

    hand.getEvent(&accel, &gyro, &temp, &mag);
    unsigned long t2 = micros();
    right_hand_state = hand_calcs(accel,right_hand_state,accel_z_past_raw_right, accel_z_past_in_right,accel_z_past_out_right,'R');
    switch_left();
    buttonState = digitalRead(button);
    if(buttonState == HIGH){
    Serial.print(mag.magnetic.z);
    Serial.print(",");
   }
    hand.getEvent(&accel, &gyro, &temp, &mag);
    left_hand_state = hand_calcs(accel,left_hand_state,accel_z_past_raw_left, accel_z_past_in_left,accel_z_past_out_left,'L');
    if(Wire.getWireTimeoutFlag()){
      Serial.print("TIMEOUT!");
      Wire.clearWireTimeoutFlag();  
    }
    
  
    body.getEvent(&accel, &gyro, &temp, &mag);
    
    body_calcs(accel, mag, accel_x_past_raw_body,  accel_x_past_in_body, accel_x_past_out_body);
    
    
    // Prepare data to be sent
    if(right_hand_state == ret){
      bluetooth_data[2] = -16; //Punch
    } else if(right_hand_state == ext){
      bluetooth_data[2] = 15;
    }else if(right_hand_state == ext_a){
      bluetooth_data[2] = 15;
      
    } else if(right_hand_state == ret_a){
      
      bluetooth_data[2] = -16;//Retracted
    }


    if(left_hand_state == ret){
      bluetooth_data[4] = -16;
    } else if(left_hand_state == ext){
      bluetooth_data[4] = 15;
    }else if(left_hand_state == ext_a){
      bluetooth_data[4] = 15;
      
    } else if(left_hand_state == ret_a){
      bluetooth_data[4] = -16;
    }
  
  union conv32 x = {.f32 = body_vel_z};
  bluetooth_data[7] = (x.u32 >> 24 & 0xFF);
  bluetooth_data[8] = (x.u32 >> 16 & 0xFF);
  bluetooth_data[9] = (x.u32 >> 8 & 0xFF);
  bluetooth_data[10] = (x.u32 & 0xFF);
  
  union conv32 y = {.f32 = body_vel_x};
  bluetooth_data[12] = (y.u32 >> 24 & 0xFF);
  bluetooth_data[13] = (y.u32 >> 16 & 0xFF);
  bluetooth_data[14] = (y.u32 >> 8 & 0xFF);
  bluetooth_data[15] = (y.u32 & 0xFF);
  
}

void switch_right(){
  Wire.beginTransmission(0x70);
  Wire.write(right_hand_mux_addr);
  Wire.endTransmission();
  
}

void switch_left(){
  Wire.beginTransmission(0x70);
  Wire.write(left_hand_mux_addr);
  Wire.endTransmission();
  
}


void send_movement_data(){
  

  
  if(bluetooth.available() > 0){
    state = bluetooth.read();
    
  }
   if(prev_right == -16 && bluetooth_data[2] == 15 || prev_right == 15 && bluetooth_data[2] == -16){
    Serial.print("R: ");
   Serial.println((int)bluetooth_data[2]);
   prev_right = bluetooth_data[2];
   }

   if(prev_left == -16 && bluetooth_data[4] == 15 || prev_left == 15 && bluetooth_data[4] == -16){
    Serial.print("L: ");
   Serial.println((int)bluetooth_data[4]);
   prev_left = bluetooth_data[4];
   }
   
   
   
  for(char x: bluetooth_data){
    //Serial.println((int)x);
    bluetooth.write(x);
  }
  
  

  
}


void setup() {
  #ifdef player1
    bluetooth_data[0] = 'A';
  #endif

  #ifdef player2
    bluetooth_data[0] = 'B';
  #endif
  bluetooth_data[16] = 'E';
  bluetooth_data[1] = 'R';
  bluetooth_data[3] = 'L';
  bluetooth_data[5] = 'C';
  bluetooth_data[6] = 'X';
  bluetooth_data[11] = 'Y';
  bluetooth.begin(38400);
  Serial.begin(38400);
  Serial.println("Setup");
  right_hand_state = ret;
  left_hand_state = ret;
  pinMode(3,INPUT);
  pinMode(2,OUTPUT);
  pinMode(button, INPUT);
  pinMode(7, OUTPUT);
  pinMode(10, OUTPUT);
  
  
   Wire.begin(0x70);
   Wire.setWireTimeout(10000, true);
   switch_left();
  
 if(!hand.begin_I2C(0x68,&Wire)){
      Serial.print("Right Hand Not Detected\n");
    } else{
      Serial.print("Right Hand Detected and Starting\n");
    }
   //icm_accel_right = hand.getAccelerometerSensor();
  //hand.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_111_4_HZ);

 hand.setAccelRateDivisor(0); 
 uint16_t accel_divisor = hand.getAccelRateDivisor();
 Serial.print("Accelerometer Right rate divisor set to: ");
 Serial.println(accel_divisor);
  
 
 
 hand.setAccelRateDivisor(1);
 accel_divisor = hand.getAccelRateDivisor();
 Serial.print("Accelerometer Left rate divisor set to: ");
 Serial.println(accel_divisor);
 hand.setAccelRateDivisor(0);
 
  //icm_accel_left = hand.getAccelerometerSensor();
  if(!body.begin_SPI(10)){
    Serial.print("Connection to Body Failed\n");
  } else{
    Serial.print("Body Detected and Starting\n");
  }
  switch_right();
  hand.begin_I2C(0x68,&Wire);
  
  sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
  
  Serial.println(body.getGyroRange());
  Serial.println(body.setMagDataRate(0x08));
  body.getEvent(&accel, &gyro, &temp, &mag);
 
 body_ang_offset = mag.magnetic.z;
  Serial.println(body.getMagDataRate());
  Serial.println(body.getAccelRateDivisor());
  body.setAccelRateDivisor(0);
  Serial.println("Divisor");
  Serial.println(body.getAccelRateDivisor());
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, send_movement_data);
 
}








void loop() {
  
  
  imu_calcs();
  
}
