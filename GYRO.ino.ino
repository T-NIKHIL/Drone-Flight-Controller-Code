#include<Wire.h>

const int MPU_addr=0x68;
float GYX,GYY,GYZ,ACCX,ACCY,ACCZ,acc_total_vector;
float GYX_off,GYY_off,GYZ_off;
float GYX_call=0;
float GYY_call=0;
float GYZ_call=0;
float gyro_pitch,gyro_roll,gyro_yaw;
float gyro_roll_input,gyro_pitch_input,gyro_yaw_input;
int TEMP;
int cal_int=0;

void setup() {
  // put your setup code here, to run once:

  //pinMode(led1,OUTPUT);
  //digitalWrite(led1,HIGH);

  Serial.begin(9600);

  //I2C COMMUNICATIONS PROTOCOL
  //STARTING ARDUINO AS MASTER AND GYROSCOPE AS SALVE
  Wire.begin();
  
  Wire.beginTransmission(MPU_addr);//IT IS THE 6 BIT I2C ADDRESS OF THE MPU-6050
  Wire.write(0x6B);//ADDRESS OF THE POWER MANAGEMENT REGISTER-1 
  Wire.write(0);//WRITING TO THE POWER MANAGEMENT REGISTER-1
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);//IT IS THE 6-BIT I2C ADDRESS OF THE MPU-6050
  Wire.write(0x1B);//ADDRESS OF THE GYRO CONFIGURATION REGISTER OF THE MPU-6050
  Wire.write(0x08);//ENABLING THE 500 degrees per sec FULL RANGE BIT
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


  Serial.println("STARTING CALLIBRATION IN 3 SECONDS ");
  Serial.println("1");
  delay(1000);;
  Serial.println("2");
  delay(1000);
  Serial.println("3");

  //CALLIBRATION FOR DETERMINING OFFSET

   for( cal_int ; cal_int<3000 ; cal_int++){
    
    values();
    
    GYX_call=gyro_roll+GYX_call;
    GYY_call=gyro_pitch+GYY_call;
    GYZ_call=gyro_yaw+GYZ_call;
} 

  GYX_call=GYX_call/3000;
  GYY_call=GYY_call/3000;
  GYZ_call=GYZ_call/3000;
  
  Serial.println("CALLIBRATION COMPLETE");

  digitalWrite(13,!digitalRead(13));

}

  
void loop() {
// put your main code here, to run repeatedly:

  values();//FUNCTION TO OBTAIN GYRO VALUES

  gyro_pitch_input=gyro_pitch/65.5;//for pid controller 
  gyro_roll_input=gyro_roll/65.5;  //for pid controller
  gyro_yaw_input=gyro_yaw/65.5;    //for pid controller
  
  print();//PRINT FUNCTION
  delay(100);
 
 }


void print(){

  Serial.print("-----------------------------------------TEMPERATURE VALUES--------------------------------------------");
  Serial.println();

  Serial.print(TEMP/340+36.53);
  Serial.print(" Celsius ");

  Serial.println();

  Serial.print("------------------------------------------GYROSCOPE VALUES----------------------------------------------");

  Serial.println();
 
  Serial.print(" ROLL:  ");
  Serial.print(gyro_roll_input);//SUBTRACTING THE OBTAINED GYRO VALUES IN DEGREES PER SEC FROM GYX_off TO REMOVE OFFSET
  Serial.print("      ");

  Serial.print(" PITCH:   ");
  Serial.print(gyro_pitch_input);//SUBTRACTING THE OBTAINED GYRO VALUES IN DEGREES PER SEC FROM GYY_off TO REMOVE OFFSET
  Serial.print("      ");

  Serial.print(" YAW:   ");
  Serial.print(gyro_yaw_input);//SUBTRACTING THE OBTAINED GYRO VALUES IN DEGREES PER SEC FROM GYZ_off TO REMOVE OFFSET
  Serial.println();

  Serial.print("---------------------------------------ACCELEROMETER VALUES----------------------------------------------");

  Serial.println();

  Serial.print("ACCELEROMETER X VALUE:");
  Serial.print(ACCX);
  Serial.print("      ");

  Serial.print("ACCELEROMETER Y VALUE:");
  Serial.print(ACCY);
  Serial.print("      ");

  Serial.print("ACCELEROMETER Z VALUE:");
  Serial.print(ACCZ);
  Serial.print("      ");

  Serial.println("----END OF LINE-----");
  
}

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

