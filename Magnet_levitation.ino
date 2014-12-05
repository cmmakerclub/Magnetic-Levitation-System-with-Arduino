/*
  magnetic levitation
  Chiangmai Maker Club
  modified 05122014
  by wasin wongkum
 */
 
#include <TimerOne.h>

#define hall        A0
#define magnetic    6
#define filter      0.4f   
#define sampling    1000.0f 

//float state_g = 0;
//float prev_state_g = 0;

float state = 0;
float prev_state = 0;
float ref = 7;
float error =0;
float prev_error =0;
float error_dot = 0;
float error_sum =0;

float kp =500;
float ki =10;
float kd =3.6;

float output  = 0;

void setup() 
{
  Serial.begin(115200);
  pinMode(hall, INPUT);
  pinMode(A1, INPUT);
  pinMode(output, OUTPUT);
  pinMode(12, OUTPUT); 
  
  Timer1.initialize(1000); // sampling rate 1 kHz
  Timer1.attachInterrupt( PID );
}

void loop() 
{
  
  delay(1000);
//    Serial.print(state);
//    Serial.print(ki);
//    Serial.print("     ");
//    Serial.print(error);
//    Serial.print("     ");    
//    Serial.print(error_sum);
//    Serial.print("     "); 
//    Serial.println(output);
}

void PID(void) 
{  
  digitalWrite(12,1);
  
//  prev_state_g = state_g ;
//  state_g = prev_state_g + (filter*(((float)(analogRead(A1))/100.0f)-prev_state_g));
//  ki = state_g /1.0f ;

  prev_state = state ;
  state = prev_state + (filter*(((float)(analogRead(hall))/100.0f)-prev_state));

  prev_error = error;
  error = ref - state;
  error_dot = (error - prev_error)*sampling;
  error_sum = error_sum + error/sampling;
  
  output = kp*error + ki*error_sum + kd*error_dot ;
  
  if(output<0)   output = 0;
  if(output>255) output = 255;
  
  if(state < 5.6)
  {
    output = 0;
    error_sum=0;
  }
  
  analogWrite(magnetic,output);
  digitalWrite(12,0);
}
