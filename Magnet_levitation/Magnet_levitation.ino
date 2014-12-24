/*
  magnetic levitation
  Chiangmai Maker Club
  modified 05-12-2014
  by wasin wongkum
  sensor hall sensor A1302
  mosfet irf 540n
  supply 12v
  coil 10 ohm
 */
 
#include <TimerOne.h>
#define ref_in      A2
#define hall        A0 
#define magnetic    5
#define filter      0.30f   
#define sampling    1000.0f 

//float state_g = 0;
//float prev_state_g = 0;

float state = 0;
float state_f = 0;
float prev_state = 0;
float prev_state_f = 0;
float ref = 7;
float error =0;
float prev_error =0;
float error_dot = 0;
float error_sum =0;

float kp =500;
float ki =30;
float kd =3.65;
float output  = 0;

void setup() 
{
  TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz // divisor change from 64 --> 1 
  Serial.begin(115200);
  pinMode(ref_in, INPUT);
  pinMode(hall, INPUT);
 
  pinMode(output, OUTPUT);
  pinMode(12, OUTPUT); 
  
  Timer1.initialize(1000); // sampling rate 1 kHz
  Timer1.attachInterrupt( PID );
}

void loop() 
{
  
delay(200*64); // divisor change from 64 --> 1  

    Serial.print("Ref ");
    Serial.print(ref);
    Serial.print("     ");
    Serial.print("State ");
    Serial.print(state);
    Serial.print("     ");  
    Serial.print("Error ");
    Serial.print(error);
    Serial.print("     "); 
    Serial.print("Error_sum ");
    Serial.print(error_sum);
    Serial.print("     ");
    Serial.print("Error_dot ");
    Serial.print(error_dot);
    Serial.print("     ");    
    Serial.print("Output "); 
    Serial.println(output);
}

void PID(void) 
{  
  digitalWrite(12,1);
  
  prev_state_f = state_f ;
  state_f = prev_state_f + (0.05*(((float)(analogRead(ref_in))/100.0f)-prev_state_f));
  ref = state_f/3.3f  ;
  
  
  /* if fix referent */
  ref = 0.8  ;
  
  /*  Update Kd gain  */
  kd =3.65 + ref*3.6f ;  
  ref = 6.3f + ref;

  prev_state = state ;
  state = prev_state + (filter*(((float)(analogRead(hall))/100.0f)-prev_state));

  prev_error = error;
  error = ref - state;
  error_dot = (error - prev_error)*sampling;
  error_sum = error_sum + error/sampling;
  
  output = kp*error + ki*error_sum + kd*error_dot + 220;
  
  if(output<0)   output = 0;
  if(output>255) output = 255;
  
  if(state < 6)
  {
    output = 0;
    error_sum=0;
  }
//  output = 0;
  analogWrite(magnetic,output);
  digitalWrite(12,0);
}
