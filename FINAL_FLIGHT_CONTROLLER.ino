//FINAL FLIGHT CONTROLLER SKETCH

#include<Wire.h>

//VARIABLE DECLARATION-------------------------------------
const int MPU_addr=0x68;
float GYX,GYY,GYZ,ACCX,ACCY,ACCZ,acc_total_vector;
float GYX_off,GYY_off,GYZ_off;
float GYX_call=0;
float GYY_call=0;
float GYZ_call=0;
float gyro_pitch,gyro_roll,gyro_yaw;
int TEMP;
float angle_roll,angle_pitch,angle_roll_acc,angle_pitch_acc;
float roll_level_adjust,pitch_level_adjust;
float angle_pitch_acc_sum=0;
float angle_roll_acc_sum=0;
byte last_channel_1,last_channel_2,last_channel_3,last_channel_4,last_channel_6;
int receiver_input_channel_1,receiver_input_channel_2,receiver_input_channel_3,receiver_input_channel_4,receiver_input_channel_6;
int esc_1,esc_2,esc_3,esc_4;
int throttle;
int cal_int=0;
int start=0;
unsigned long timer_channel_1,timer_channel_2,timer_channel_3,timer_channel_4,timer_channel_6,esc_timer,esc_loop_timer;
unsigned long timer_1,timer_2,timer_3,timer_4,timer_6,current_time;
unsigned long loop_timer,check_timer;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
boolean gyro_angles_set=true;

//PID SETTINGS---------------------------------------------
float pid_p_gain_roll=1.5;
float pid_i_gain_roll=0.0009  ;//0.0003
float pid_d_gain_roll=0.00001;//
int pid_max_roll=400;//MAX OUPUT OF PID-CONTROLLER(+/-)

float pid_p_gain_pitch=pid_p_gain_roll;
float pid_i_gain_pitch=pid_i_gain_roll;
float pid_d_gain_pitch=pid_d_gain_roll;
int pid_max_pitch=400;//MAX OUPUT OF PID-CONTROLLER(+/-)

float pid_p_gain_yaw=3;
float pid_i_gain_yaw=0.02;
float pid_d_gain_yaw=0;
int pid_max_yaw=400;//MAX OUTPUT OF PID-CONTROLLER(+/-)

boolean auto_level=true;

//SETUP ROUTINE--------------------------------------------
void setup() {//CONSISTS OF TWO STEPS
  
  // put your setup code here, to run once:

  //RECEIVER PINS(RECEIVE INPUTS FROM THE RECEIVER)
  //DIGITAL PIN 8(CHANNEL1)
  //DIGITAL PIN 9(CHANNEL2)
  //DIGITAL PIN 10(CHANNEL3)
  //DIGITAL PIN 11(CHANNEL4)
  //DIGITAL PIN 12(CHANNEL6) CAN BE CONFIGURED

  //ESC PINS(THE CALCULATED PULSE WIDTH IS SENT TO EACH ESC)
  //DIGITAL PIN 4(ESC1-REAR-LEFT)
  //DIGITAL PIN 5(ESC2-REAR-RIGHT)
  //DIGITAL PIN 6(ESC3-FRONT-RIGHT)
  //DIGITAL PIN 7(ESC4-FRONT LEFT)

  //STEP 1 : SETTING GYROSCOPE SETTINGS AND CALCULATING GYRO OFFSETS

  //BY DEFAULT ARDUINO PINS ARE INPUT

  Serial.begin(9600);
  DDRB |= B00100000;//PIN 13 LED INDICATOR
  DDRD |= B11110000;//CONFIGURING DIGITAL PINS 4,5,6,7 AS OUTPUT
 
 // digitalWrite(12,HIGH);//STARTUP INDICATION LED
  digitalWrite(13,LOW);
  delay(2000);
  
  Wire.begin();//STARTING I2C WITH ARDUINO AS MASTER AND GYROSCOPE AS SLAVE

  //IN I2C COMMUNICATIONS PROTOCOL WE FIRST WRITE ADDRESS AND THEN SPECIFY WHAT TO WRITE
  Wire.beginTransmission(MPU_addr);//GYROSCOPE ADDRESS
  Wire.write(0x6B);//ADDRESS OF THE REGISTER TO WRITE TO
  Wire.write(0);//WRITING 0 , TELLING THE GYRO TO BECOME ACTIVE
  Wire.endTransmission();//END TRANSMISSION

  //GYRO FULL RANGE SETTING IS 500 DEGREES PER SEC
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  //ACCEL FULL RANGE SETTING IS +/-8g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,1);
  while(Wire.available()<1);
  if(Wire.read()!=0x08){
    digitalWrite(13,HIGH);
    while(1)delay(10);
  }
  
  //ENABLING DIGITAL LOW PASS FILTER 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);//CONFIG REGISTER ADDRESS
  Wire.write(0x03);//REMOVES NOISE GREATER THAN 98 Hz , DELAY IS 4.8 ms
  Wire.endTransmission();

  delay(250);//GIVING GYRO TIME TO START

  //CALLIBRATION FOR DETERMINING OFFSET

   for( cal_int ; cal_int<3000 ; cal_int++){
    
    values();
    
    GYX_call=gyro_roll+GYX_call;
    GYY_call=gyro_pitch+GYY_call;
    GYZ_call=gyro_yaw+GYZ_call;

    
   acc_total_vector=sqrt((ACCX*ACCX)+(ACCY*ACCY)+(ACCZ*ACCZ));

   if(abs(ACCX)<acc_total_vector){
   angle_pitch_acc=asin((float)ACCX/acc_total_vector)*57.296;}
   if(abs(ACCY)<acc_total_vector){
   angle_roll_acc=asin((float)ACCY/acc_total_vector)*-57.296;}

   angle_pitch_acc_sum = angle_pitch_acc_sum + angle_pitch_acc;
   angle_roll_acc_sum  = angle_roll_acc_sum  + angle_roll_acc; 
} 

  GYX_call=GYX_call/3000;
  GYY_call=GYY_call/3000;
  GYZ_call=GYZ_call/3000;
  angle_pitch_acc_sum=angle_pitch_acc_sum/3000;
  angle_roll_acc_sum=angle_roll_acc_sum/3000;
  
  //STEP2 :ENABLING PIN CHANGE INTERRUPTS AND WAITING FOR RECEIVER TO BECOME ACTIVE(INDICATED BY BLINKING INDICATOR LED)

  //ENABLING PIN CHANGE INTERRUPTS(PINS 8,9,10,11,12)
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change
  //PCMSK0 |= (1 << PCINT4);                           // set PCINT4 (digital input 12)to trigger an interrupt on state change

  // digitalWrite(12,LOW);

  //WAITING RECEIVER TO BECOME ACTIVE AND SET THROTTLE TO LOWEST POSITION
  while(receiver_input_channel_3<990 || receiver_input_channel_3>1020){
   
    start++;
    
    PORTD |=B11110000;
    delayMicroseconds(1000);
    PORTD &=B00001111;
    delayMicroseconds(3000);
    
    if(start==125){
      digitalWrite(13,!digitalRead(13));
      start=0;
    }
    
  }  

  start=0; 

  loop_timer=micros();
 
  //WHEN THE 2 STEPS ARE OVER SWITCHING INDICATOR LED OFF 
  digitalWrite(13,LOW);
 
}

//MAIN PROGRAM LOOP------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly: 
 // check_timer=micros();

  values();
    
  gyro_pitch_input=gyro_pitch/65.5;//for pid controller 
  gyro_roll_input=gyro_roll/65.5;  //for pid controller
  gyro_yaw_input=gyro_yaw/65.5;    //for pid controller
  
///AUTO LEVEL CODE------------------------------------------------------------------------

 angle_pitch+=gyro_pitch*0.00007786;//TRAVELLED PITCH ANGLE
 angle_roll+=gyro_roll*0.00007786;//TRAVELLED ROLL ANGLE
 
 angle_pitch-=angle_roll*sin(gyro_yaw*0.000001358);   //IF THE IMU HAS YAWED TRANSFER THE ROLL ANGLE TO THE PITCH ANGLE
 angle_roll+=angle_pitch*sin(gyro_yaw*0.000001358);   //IF THE IMU HAS YAWED TRANSFER THE PITCH ANGLE TO THE ROLL ANGLE

 acc_total_vector=sqrt((ACCX*ACCX)+(ACCY*ACCY)+(ACCZ*ACCZ));

 if(abs(ACCX)<acc_total_vector){
 angle_pitch_acc=asin((float)ACCX/acc_total_vector)*57.296;}
 if(abs(ACCY)<acc_total_vector){
 angle_roll_acc=asin((float)ACCY/acc_total_vector)*-57.296;}

 angle_pitch_acc-=angle_pitch_acc_sum;//acceleration callibration values
 angle_roll_acc-=angle_roll_acc_sum;//acceleration callibration values

 angle_pitch=angle_pitch*0.9996+angle_pitch_acc*0.0004;//final angle pitch
 angle_roll=angle_roll*0.9996+angle_roll_acc*0.0004;//final angle roll

 pitch_level_adjust=angle_pitch*15;
 roll_level_adjust=angle_roll*15;

 if(!auto_level){
  pitch_level_adjust=0;
  roll_level_adjust=0;
 }
  
  //FOR STARTING MOTORS THROTTLE AT LOWEST POSITION AND YAW RIGHT
  if(receiver_input_channel_3<1030 && receiver_input_channel_4<1020)
    start=1;
  //WHEN YAW STICK BACK IN CENTER START MOTORS
  if(start==1 && receiver_input_channel_3<1030 && receiver_input_channel_4>1500){
    start=2;

    angle_pitch=angle_pitch_acc;
    angle_roll=angle_roll_acc;
    gyro_angles_set=true;

    //USE WHEN USING I AND D CONTROLLERS
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
 //FOR STOPPING THE MOTORS THROTTLE AT LOWEST POSITION AND YAW AT RIGHT 
 if(start==2 && receiver_input_channel_3<1030 && receiver_input_channel_4>1950)
 start=0;
 
 //CONVERTING THE RECEIVER INPUTS INTO DEGREE PER SECOND

 //THE PID SETPOINT IN DEGREES PER SECOND IS DETERMINED BY THE ROLL RECEIVER INPUT
  pid_roll_setpoint=0;
 //AILERON (CHANNEL 1) IS USED FOR ROLL
 //DEADBAND OF 16us FOR GOOD RESULTS
 if(receiver_input_channel_1<1535)pid_roll_setpoint=receiver_input_channel_1-1535;//IF STICK MOVED TO THE LEFT (-ROLL)
 else if(receiver_input_channel_1>1551)pid_roll_setpoint=receiver_input_channel_1-1551;//IF STICK MOVED TO THE RIGHT (+ROLL)
 //IF receiver_input_channel_1 LIES BETWEEN 1536 AND 1552 THEN SET PID ROLL SETPOINT TO 0.
 //HIGHER ROLL RATES CAN BE INTRODUCED BY DECREASING THE DENOMINATOR VALUE.

 pid_roll_setpoint-=roll_level_adjust;//MAX ANHLE IS AROUND 17 degrees Of ROLL.THIS IS BECAUSE AS U MOVE THE TRANSMITTER AND INCREASE THE ROLL
 pid_roll_setpoint/=3.0;              //THE DIFFERENCE FROM MIDDLE VALUE INCREASES AND AT A PARTICULAR ANGLE SAY 17 degrees THE pid_roll_setpoint and roll_level_adjust become the same
 
 //THE PID SETPOINT IN DEGREES PER SECOND IS DETERMINED BY THE PITCH RECEIVER INPUT
 pid_pitch_setpoint = 0;
 //ELEVATOR (CHANNEL 2) IS USED FOR PITCH
 if(receiver_input_channel_2 > 1521)pid_pitch_setpoint = receiver_input_channel_2 - 1521;//IF STICK MOVED UP (+PITCH)
 else if(receiver_input_channel_2 < 1504)pid_pitch_setpoint = receiver_input_channel_2 - 1504;//IF STICK MOVED DOWN (-PITCH)
 //IF receiver_input_channel_2 LIES BETWEEN 1507 AND 1523 THEN SET PID PITCH SETPOINT TO 0.
 //HIGHER PITCH RATES CAN BE INTRODUCED BY DECREASING THE DENOMINATOR VALUE.

 pid_pitch_setpoint-=pitch_level_adjust;
 pid_pitch_setpoint/=3.0;
  
 //THE PID SETPOINT IN DEGREES PER SEC IS DETERMINED BY THE YAW RECEIVER INPUT
 pid_yaw_setpoint = 0;
 //RUDDER (CHANNEL 4) IS USED FOR YAW
 //DEADBAND OF 16us FOR GOOD RESULTS
 if(receiver_input_channel_3 > 1050){ //DO NOT YAW WHEN TURNING OFF MOTORS
   if(receiver_input_channel_4 > 1520)pid_yaw_setpoint = (receiver_input_channel_4 - 1520)/3;//IF STICK MOVED TO RIGHT (+YAW)
   else if(receiver_input_channel_4 < 1504)pid_yaw_setpoint = (receiver_input_channel_4 - 1504)/3;//IF STICK MOVED TO LEFT (-YAW)
 }
 //IF receiver_input_channel_4 LIES BETWEEN 1523 AND 1507 THEN SET PID YAW SETPOINT TO 0.
 
 calculate_pid();

 throttle=receiver_input_channel_3;//BASE SIGNAL(MIN IS 1000 AND MAX IS 1992)

 if(start==2){
  if(throttle>1800)throttle=1800;//NOT KEEPING FULL THROTTLE SO THAT WE HAVE SOME ROOM FOR CONTROL EVEN AT HIGH THROTTLE
  esc_1 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 1 (rear-left - CCW)
  esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 3 (front-right - CCW)
  esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 4 (front-left - CW)


   if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
   if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
   if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
   if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

   //ALL ESC PULSES ARE IN THE RANGE OF 1200 - 1990
    
   if (esc_1 > 1990) esc_1 = 1990;                                           //Limit the esc-1 pulse to 2000us.
   if (esc_2 > 1990) esc_2 = 1990;                                           //Limit the esc-2 pulse to 2000us.
   if (esc_3 > 1990) esc_3 = 1990;                                           //Limit the esc-3 pulse to 2000us.
   if (esc_4 > 1990) esc_4 = 1990;                                           //Limit the esc-4 pulse to 2000us.  
  
}

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
  
  //STEP4: ALL THE INFORMATION FOR CONTROLLING THE MOTOR IS PRESENT , NOW WE HAVE TO SEND IT TO THE ESCS

  if(micros()-loop_timer>5150)digitalWrite(13,!digitalRead(13));

 
  while(micros()-loop_timer<5100);

  loop_timer=micros();

  PORTD |=B11110000;
  timer_channel_1=esc_1+loop_timer;
  timer_channel_2=esc_2+loop_timer;
  timer_channel_3=esc_3+loop_timer;
  timer_channel_4=esc_4+loop_timer;

  while(PORTD>16){//STAY IN THE LOOP UNTIL DIGITAL PINS 4,5,6,7 ARE LOW
    esc_loop_timer=micros();
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
    
  }

//  Serial.print(micros()-check_timer);
//  Serial.println();
 

}

//FUNCTION TO CALCULATE PID OUTPUTS-------------------------------------------------------- 
void calculate_pid(){

  //GYRO ALWAYS TRIES TO MATCH THE RECEIVER INPUTS  , IF PILOT WANTS 0 MOVEMENT GYRO SHOULD ASLO DISPLAY 0 MOVEMENT , TO DO THIS IT SENDS
  //THE CALCULTED ESC PULSES TO CORRECT MOTOR SPEEDS.
  
  //ROLL CALCULATIONS
  pid_error_temp=gyro_roll_input-pid_roll_setpoint;//ERROR=GYRO-RECEIVER
  pid_i_mem_roll+=pid_i_gain_roll*pid_error_temp;//ALREDY SET pid_i_mem_roll=0 AT STARTING OF MOTORS
  if(pid_i_mem_roll>pid_max_roll)pid_i_mem_roll=pid_max_roll;//THIS IS TO PREVENT I-OUTPUT FROM GOING OUT OF CONTROL WHILE SUMMING PREVIOUS ERRORS
  else if(pid_i_mem_roll<pid_max_roll*-1)pid_i_mem_roll=pid_max_roll*-1;

//                 |--------P-OUTPUT-------------|  |---I-OUTPUT--|  |-----------------D-OUTPUT-----------------------------|
  pid_output_roll = pid_p_gain_roll*pid_error_temp + pid_i_mem_roll + pid_d_gain_roll*(pid_error_temp-pid_last_roll_d_error);
  if(pid_output_roll>pid_max_roll)pid_output_roll=pid_max_roll;//TO PREVENT OUTPUT FROM CROSSING THE MAX SET ROLL OF 500 degrees per sec
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;//IN THE BEGINNING pid_last_roll_d_error WAS SET TO 0 AT START OF MOTOR SEQUENCE.

  //PITCH CALCULATIONS
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //YAW CALCULATIONS
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

//FUNCTION READS THE GYRO VALUES FROM THE SENSOR-------------------------------------------
void values(){
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_addr,14);
  while(Wire.available()<14);
  ACCX=Wire.read()<<8|Wire.read();
  ACCY=Wire.read()<<8|Wire.read();
  ACCZ=Wire.read()<<8|Wire.read();
  TEMP=Wire.read()<<8|Wire.read();
  gyro_roll=Wire.read()<<8|Wire.read();
  gyro_roll=-gyro_roll;
  if(cal_int==3000)gyro_roll-=GYX_call;
  gyro_pitch=Wire.read()<<8|Wire.read();
  if(cal_int==3000){
    gyro_pitch-=GYY_call;
    gyro_pitch=-gyro_pitch;
  }
  gyro_yaw=Wire.read()<<8|Wire.read();
  gyro_yaw=-gyro_yaw;
  if(cal_int==3000)gyro_yaw-=GYZ_call;
  
  }

//SUB - ROUTINE CALLED EVERY TIME THERE IS CHANGE IN PIN STATE -8(CHA1),9(CHA2),10(CHA3),11(CHA4)
ISR(PCINT0_vect){
  
  current_time = micros();
  
  //Channel 1=========================================
  if(PINB & B00000001){                                        //CHECKS IF PIN 8 IS HIGH
    if(last_channel_1 == 0){                                   //PIN CHANGES FROM 0 TO 1
      last_channel_1 = 1;                                      //REMEMBER CURRENT INPUT STATE
      timer_1 = current_time;                                  // SET timer_1 TO current _time
    }
  }
  else if(last_channel_1 == 1){                                //IMPUT 8 IS NOT HIGH CHANGED FROM 1 TO 0
    last_channel_1 = 0;                                        //REMEMBER CURRENT INPUT STATE
    receiver_input_channel_1 = current_time - timer_1;         //CHANNEL 1 IS current_time-timer_1
  }
  
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //CHECKS IF PIN 9 IS HIGH
    if(last_channel_2 == 0){                                   
      last_channel_2 = 1;                                     
      timer_2 = current_time;                                  
    }
  }
  else if(last_channel_2 == 1){                                
    last_channel_2 = 0;                                        
    receiver_input_channel_2 = current_time - timer_2;         
  }
  
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //CHECKS IF PIN 10 IS HIGH
    if(last_channel_3 == 0){                                   
      last_channel_3 = 1;                                    
      timer_3 = current_time;                                  
    }
  }
  else if(last_channel_3 == 1){                               
    last_channel_3 = 0;                                        
    receiver_input_channel_3 = current_time - timer_3;         
  }
  
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //CHECKS IF PIN 11 IS HIGH
    if(last_channel_4 == 0){                                 
      last_channel_4 = 1;                                     
      timer_4 = current_time;                                  
    }
  }
  else if(last_channel_4 == 1){                                
    last_channel_4 = 0;                                        
    receiver_input_channel_4 = current_time - timer_4;         
  }

/*  //Channel 6=========================================
  if(PINB & B00010000 ){
    if(last_channel_6 == 0){
      last_channel_6 = 1;
      timer_6 = current_time;
     }
   }
   else if(last_channel_6 == 1){
    last_channel_6 = 0;
    receiver_input_channel_6 = current_time - timer_6; 
  }*/

  
}


