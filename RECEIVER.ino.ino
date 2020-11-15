byte last_channel_1,last_channel_2,last_channel_3,last_channel_4;
int receiver_channel_1,receiver_channel_2,receiver_channel_3,receiver_channel_4;
long current_time,timer_1,timer_2,timer_3,timer_4;

void setup() {
  // put your setup code here, to run once:
  PCICR |=(1<<PCIE0);
  PCMSK0 |=(1<<PCINT0);//digital pin 8
  PCMSK0 |=(1<<PCINT1);//digital pin 9
  PCMSK0 |=(1<<PCINT2);//digital pin 10
  PCMSK0 |=(1<<PCINT3);//digital pin 11
  Serial.begin(9600);

  //digital pins 8,9,10,11 already set as input pins by default.
  

}

void loop() {
  // put your main code here, to run repeatedly:
  print();
  
}

void print(){
  Serial.print("   CHANNEL 1 (AILERON):    ");
  Serial.print(receiver_channel_1);
  
  Serial.print("   CHANNEL 2 (ELEVATOR):   ");
  Serial.print(receiver_channel_2);
  
  Serial.print("   CHANNEL 3 (THROTTLE):    ");
  Serial.print(receiver_channel_3);
  
  Serial.print("   CHANNEL 4 (RUDDER):    ");
  Serial.print(receiver_channel_4);

  Serial.println();
  
  
}

ISR(PCINT0_vect){
  current_time=micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}


