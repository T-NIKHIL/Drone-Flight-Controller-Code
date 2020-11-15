//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_loop_timer , zero_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

//FIRST UPLOAD CODE THEN SWITCH ON BATTERY PACK

//Setup routine
void setup(){
//BY DEFAULT ARDUINO PINS ARE INPUT
  DDRD |= B11110000;//CONFIGURING DIGITALMPINS 4(ESC1),5(ESC2),6(ESC3),7(ESC4) AS OUTPUT PINS
  DDRB |= B00010000;//PIN 12 LED INDICATOR

//ENABLING PIN CHANGE INTERRUPTS
  
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change
  
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020){ //WAIT WHILE RECEIVER IS ACTIVE(receiver_input_channel_3<990) AND THROTTLE SET TO LOWEST POSITION(receiver_input_channel_3>1020)
 
    start ++;//While waiting increment start whith every loop.
    
    //DON'T WANT ESC TO BEEPING ANNONIGLY THERFORE GIVING THEM A 1000us PULSE 

    PORTD |= B11110000;                              //Set digital port 4,5,6,7 high.
    delayMicroseconds(1000);                         
    PORTD &= B00001111;                              //Set digital port 4,5,6,7 low.
    delay(3);                                        //Wait 3 milliseconds before the next loop.
    if(start == 125){                                //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));            //This led gives confirmation to pilot that setup for gyro has finished and waits for receiver to turn on. 
      start = 0;                                     //Start again at 0.
    }
  }
  start = 0;
  digitalWrite(12, LOW);                                                    
}

void loop(){

  zero_timer = micros();                                 
  PORTD |= B11110000;//SETTING DIGITAL PORT 4,5,6,7 HIGH
  timer_channel_1 = receiver_input_channel_3 + zero_timer;   //TIME WHEN DIGITAL PORT 8 IS HIGH
  timer_channel_2 = receiver_input_channel_3 + zero_timer;   //TIME WHEN DIGITAL PORT 9 IS HIGH
  timer_channel_3 = receiver_input_channel_3 + zero_timer;   //TIME WHEN DIGITAL PORT 10 IS HIGH
  timer_channel_4 = receiver_input_channel_3 + zero_timer;   //TIME WHEN DIGITAL PORT 11 IS HIGH
  
  while(PORTD > 16){                                         //LOOP EXECUTES TILL DIGITAL PORT 8 TO 11 ARE LOW
    esc_loop_timer = micros();                               //CHECK THE CURRENT TIME
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111; //DIGITAL PORT 8 IS SET LOW
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111; //DIGITAL PORT 9 IS SET LOW
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111; //DIGITAL PORT 10 IS SET LOW
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111; //DIGITAL PORT 11 IS SET LOW
  }
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
}

