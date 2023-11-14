/****************************************************/
/* This is only one example of code structure       */
/* OFFCOURSE this code can be optimized, but        */
/* the idea is let it so simple to be easy catch    */
/* where can do changes and look to the results     */
/****************************************************/

#define VFD_in 8// If 0 write LCD, if 1 read of LCD
#define VFD_clk 9 // if 0 is a command, if 1 is a data0
#define VFD_stb 10 // Must be pulsed to LCD fetch data of bus

//#define VFD_onGreen 4 // Led on/off Green
//#define VFD_onRed 5 // Led on/off Red
//#define VFD_net 6 // Led net

#define AdjustPins    PIND // before is C, but I'm use port C to VFC Controle signals

unsigned char DigitTo7SegEncoder(unsigned char digit, unsigned char common);

/*Global Variables Declarations*/
unsigned char hrs = 0;
unsigned char mins = 0;
unsigned char secs=0;
unsigned char milisec = 0;

unsigned char memory_secs=0;
unsigned char memory_mins=0;

unsigned char number;
unsigned char numberA0=0;
unsigned char numberA1=0;
unsigned char numberB0=0;
unsigned char numberB1=0;
unsigned char numberC0=0;
unsigned char numberC1=0;
unsigned char numberD0=0;
unsigned char numberD1=0;
unsigned char numberE0=0;
unsigned char numberE1=0;
unsigned char numberF0=0;
unsigned char numberF1=0;

unsigned char digit=0;
unsigned char grid=0;
unsigned char gridSegments = 0b00000001; // Here I define the number of GRIDs and Segments! I'm using 5 on this driver.

boolean flag=true;
boolean flagSecs=false;

void clear_VFD(void);

unsigned int segOR[14];
 
unsigned char segNum[20] ={
         (0b00111111),(0b00000000),        
         (0b00000110),(0b00000000), 
         (0b01011011),(0b00000010),
         (0b01001111),(0b00000010),
         (0b01100110),(0b00000000), 
         (0b01101101),(0b00000010),
         (0b01111101),(0b00000000),
         (0b00000111),(0b00000000),
         (0b01111111),(0b00000000),
         (0b01100111),(0b00000010)
         };

unsigned char segNumber[11] ={
         (0b00111111),        
         (0b00000110), 
         (0b01011011),
         (0b01001111),
         (0b01100110), 
         (0b01101101),
         (0b01111101),
         (0b00000111),
         (0b01111111),
         (0b01100111),
         (0b01000000),
         };


void LD1207_init(void)
{
  delayMicroseconds(200); //power_up delay
  // Note: Allways the first byte in the input data after the STB go to LOW is interpret as command!!!

  // Configure VFD display (grids)
  send_command(gridSegments); // cmd 1 // LD1207 is driver until 7 grids
  delayMicroseconds(1);
  // Write to memory display, increment address, normal operation
  send_command(0b01000000);//(BIN(01000000));
  delayMicroseconds(1);
  // Address 00H - 15H ( total of 11*2Bytes=176 Bits)
  send_command(0b11000000);//(BIN(01100110)); 
  delayMicroseconds(1);
  // set DIMM/PWM to value
  send_command((0b10001000) | 7);//0 min - 7 max  )(0b01010000)
  delayMicroseconds(1);
}
void send_data8(unsigned char a)
{
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  digitalWrite(VFD_stb, LOW);
  delayMicroseconds(2);
  data=a;
  //This don't send the strobe signal, to be used in burst data send
         for (mask = 0b00000001; mask>0; mask <<= 1) { //iterate through bit mask
           digitalWrite(VFD_clk, LOW);
                 if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(VFD_in, HIGH);
                    //Serial.print(1);
                 }
                 else{ //if bitwise and resolves to false
                   digitalWrite(VFD_in, LOW);
                   //Serial.print(0);
                 }
          delayMicroseconds(5);
          digitalWrite(VFD_clk, HIGH);
          delayMicroseconds(5);
         }
   //digitalWrite(VFD_clk, LOW);
   //Serial.println(" :8 bits");
}
void send_data4(unsigned char a)
{
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  //This don't send the strobe signal, to be used in burst data send
         for (mask = 0b00010000; mask>0; mask <<= 1) { //iterate through bit mask
           digitalWrite(VFD_clk, LOW);
                 if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(VFD_in, HIGH);
                    //Serial.print(1);
                 }
                 else{ //if bitwise and resolves to false
                   digitalWrite(VFD_in, LOW);
                   //Serial.print(0);
                 }
          delayMicroseconds(5);
          digitalWrite(VFD_clk, HIGH);
          delayMicroseconds(5);
         }
   //digitalWrite(VFD_clk, LOW);
   //Serial.println(" :4 Bits");
}
void send_command(unsigned char a)
{
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  
  //This send the strobe signal
  //Note: The first byte input at in after the STB go LOW is interpreted as a command!!!
   digitalWrite(VFD_stb, HIGH);
   delayMicroseconds(1);
  digitalWrite(VFD_stb, LOW);
  delayMicroseconds(1);
         for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
           digitalWrite(VFD_clk, LOW);
           delayMicroseconds(1);
                 if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(VFD_in, HIGH);
                 }
                 else{ //if bitwise and resolves to false
                   digitalWrite(VFD_in, LOW);
                 }
          digitalWrite(VFD_clk, HIGH);
          delayMicroseconds(1);
         }
   digitalWrite(VFD_stb, HIGH);
   delayMicroseconds(1);
}
void test_msg(void)
{
      send_command(gridSegments); // cmd 1 // LD1207 is a drive of 5 grids
      delayMicroseconds(4);
      send_command(0b01000000); // cmd 2 //B3 is normal operation, B2 increment, B1 & B0 is write to display mode.
      delayMicroseconds(4);
       
                          send_data8(0b11000000); // cmd 2 //Address to start write Importante, it skip 2 by 2: 0,2,4,6,8,A,
                          delayMicroseconds(4);  
                            //Note: Is necessary send 8+8bits, if I send 8+4 fail the sequence of digits!
                            send_data8(~0b01110110); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~0b01111001); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~0b00111000); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~0b00111000); // 0
                            send_data8(0b11110000); //
                            // 
                            send_data8(~0b00111111); // 0
                            send_data8(0b11110000); // 
                            //
                            send_command((0b10001000) | 7); //cmd 4 Bit 3 is display ON, B0,B1,B2 is dimming settings on LED1207 driver.
                            delayMicroseconds(5);
}
void test_0to4(void)
{
      send_command(gridSegments); // cmd 1 // LD1207 is a drive of 5 grids
      delayMicroseconds(4);
      send_command(0b01000000); // cmd 2 //B3 is normal operation, B2 increment, B1 & B0 is write to display mode.
      delayMicroseconds(4);
       
                          send_data8(0b11000000); // cmd 2 //Address to start write Importante, it skip 2 by 2: 0,2,4,6,8,A,
                          delayMicroseconds(4);  
                            //Note: Is necessary send 8+8bits, if I send 8+4 fail the sequence of digits!
                            send_data8(~segNum[0]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~segNum[2]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~segNum[4]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~segNum[6]); // 0
                            send_data8(0b11110000); //
                            // 
                            send_data8(~segNum[8]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_command((0b10001000) | 7); //cmd 4 Bit 3 is display ON, B0,B1,B2 is dimming settings on LED1207 driver.
                            delayMicroseconds(5);
}
void test_5to9(void)
{
      send_command(gridSegments); // cmd 1 // LD1207 is a drive of 5 grids
      delayMicroseconds(4);
      send_command(0b01000000); // cmd 2 //B3 is normal operation, B2 increment, B1 & B0 is write to display mode.
      delayMicroseconds(4);
       
                          send_data8(0b11000000); // cmd 2 //Address to start write Importante, it skip 2 by 2: 0,2,4,6,8,A,
                          delayMicroseconds(4);  
                            //Note: Is necessary send 8+8bits, if I send 8+4 fail the sequence of digits!
                            send_data8(~segNum[10]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~segNum[12]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~segNum[14]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_data8(~segNum[16]); // 0
                            send_data8(0b11110000); //
                            // 
                            send_data8(~segNum[18]); // 0
                            send_data8(0b11110000); // 
                            //
                            send_command((0b10001000) | 7); //cmd 4 Bit 3 is display ON, B0,B1,B2 is dimming settings on LED1207 driver.
                            delayMicroseconds(5);
}
void test_Seg(void)
{
  send_command(gridSegments); // cmd 1 // LD1207 is a drive of 5 grids
  delayMicroseconds(4);
 // 
  for(unsigned char i=0x00; i < 9; i=i+2){
    for(unsigned char seg = 0b00000001; seg > 0; seg <<=1){
      send_command(0b01000000); // cmd 2 //B3 is normal operation, B2 increment, B1 & B0 is write to display mode.
      delayMicroseconds(4); 
                          send_data8(0b11000000 | i); // cmd 2 //Address to start write 
                          delayMicroseconds(4); 
                            send_data8(~seg); // 0
                            send_data4(0b11110000); // 
                            //
                            send_command((0b10001000) | 7); //cmd 4 Bit 3 is display ON, B0,B1,B2 is dimming settings on LED1207 driver.
                            delayMicroseconds(2);
                            //
                            for(int s=0; s < 15000; s++){
                            delayMicroseconds(5);
                          }
        
        }                  
  }
}
void test_CleanSeg(void)
{
  send_command(gridSegments); // cmd 1 // LD1207 is a drive of 5 grids
  delayMicroseconds(4);
  
  for(unsigned int i=0x00; i < 9; i=i+2){
    for(unsigned char seg = 0b00000001; seg > 0; seg <<=1){
      send_command(0b01000000); // cmd 2 //B3 is normal operation, B2 increment, B1 & B0 is write to display mode.
      delayMicroseconds(4); 
                          send_data8(0b11000000 | i); // cmd 2 //Address to start write 
                          delayMicroseconds(4); 
                            send_data8(0b11111111); // 0
                            send_data4(0b11110000); // 
                            //digitalWrite(VFD_stb, HIGH);
                            send_command((0b10001000) | 7); //cmd 4 Bit 3 is display ON, B0,B1,B2 is dimming settings on LED1207 driver.
                            delayMicroseconds(2);
                            Serial.println(seg, BIN);
                          }            
  }
  Serial.println("End of Process!!! ............................................................");
}
void test_panel_DVD(void)
{
  for (int x=0; x< 8; x++){
    digit=x;
 
          for (int i=0; i< 10;i++){
              delayMicroseconds(1);
              send_command(gridSegments); // cmd 1 // LD1207 is a driver of 7 grids
              send_command(0b10000000); // cmd 2 //Normal operation; Set pulse as 1/16
              digitalWrite(VFD_stb, LOW);
              delayMicroseconds(1);
              send_data8(0b11000000); // Grids of display... they have done the swap of this pins with segments
            //                    
                                switch (i) {
                                  case 0: 
                                        for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 1: 
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 2:
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        } break;
                                  case 3: 
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 4: 
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 5:
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 6: 
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 7: 
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 8: 
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        }break;
                                  case 9:
                                  for(int v=0; v<14; v++){
                                          send_data8(segNum[v]<<digit);
                                        } break;
                                  default:break;
                                }  
                  digitalWrite(VFD_stb, HIGH);
                  send_command((0b10001000) | 7); //cmd 4
                 delay(100);
          }
  }
}
void write_panel_DVD( )
{
  send_command(gridSegments); // cmd 1 // LD1207 is a drive of 5 grids
  delayMicroseconds(4);
  send_command(0b01000000); // cmd 2 //B3 is normal operation, B2 increment, B1 & B0 is write to display mode.
  delayMicroseconds(4);
                          send_data8(0b11000000); // cmd 2 //Address to start write 
                          delayMicroseconds(4);  
                            //
                            send_data8(~segNumber[numberD0]); // 0
                            send_data8(0b00000000); // 
                            //
                            send_data8(~segNumber[numberC0]); // 0
                            send_data8(0b00000000); // 
                            //
                            send_data8(~segNumber[10]); // 0
                            send_data8(0b00000000); // 
                            //
                            send_data8(~segNumber[numberB0]); // 0
                            send_data8(0b00000000); //
                            // 
                            send_data8(~segNumber[numberA0]); // 0
                            send_data8(0b00000000); // 
                            //
                            send_command((0b10001000) | 7); //cmd 4 Bit 3 is display ON, B0,B1,B2 is dimming settings on LED1207 driver.
                            delayMicroseconds(2);                                
}
void clear_VFD(void)
{
  /*
  Here I clean all registers 
  Could be done only on the number of grid
  to be more fast. The 12 * 3 bytes = 36 registers
  */
      for (int n=0; n < 5; n++){  // important be 10, if not, bright the half of wells./this on the VFD of 6 grids)
        //send_command(gridSegments); // cmd 1 // LD1207 is fixed to 5 grids
        send_command(0b10000000); //       cmd 2 //Normal operation; Set pulse as 1/16
        digitalWrite(VFD_stb, LOW);
        delayMicroseconds(1);
            send_data8((0b11000000) | n); // cmd 3 //wich define the start address (00H to 15H)
            send_data8(0b00000000); // Data to fill table of 5 grids, first byte
            send_data8(0b00000000); // Data to fill table of 6 grids, half of second byte.
            //
            digitalWrite(VFD_stb, HIGH);
            delayMicroseconds(1);
     }
}
void setup() 
{
// initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  secs = 0x00;
  mins =0x00;
  hrs = 0x00;

  /*CS12  CS11 CS10 DESCRIPTION
  0        0     0  Timer/Counter1 Disabled 
  0        0     1  No Prescaling
  0        1     0  Clock / 8
  0        1     1  Clock / 64
  1        0     0  Clock / 256
  1        0     1  Clock / 1024
  1        1     0  External clock source on T1 pin, Clock on Falling edge
  1        1     1  External clock source on T1 pin, Clock on rising edge
 */
  // initialize timer1 
  cli();           // disable all interrupts
  //initialize timer1 
  //noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;// This initialisations is very important, to have sure the trigger take place!!!
  
  TCNT1  = 0;
  
  // Use 62499 to generate a cycle of 1 sex 2 X 0.5 Secs (16MHz / (2*256*(1+62449) = 0.5
  OCR1A = 62498;            // compare match register 16MHz/256/2Hz
  //OCR1A = 1500; // only to use in test, increment secs to fast!
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= ((1 << CS12) | (0 << CS11) | (0 << CS10));    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

// Note: this counts is done to a Arduino 1 with Atmega 328... Is possible you need adjust
// a little the value 62499 upper or lower if the clock have a delay or advance on hrs.
   
//  a=0x33;
//  b=0x01;

CLKPR=(0x80);
//Set PORT NOTES:
//Sometimes you might need to set multiple output pins at exactly the same time. Calling 
//digitalWrite(10,HIGH); followed by digitalWrite(11,HIGH); will cause pin 10 to go HIGH 
//several microsecs before pin 11, which may confuse certain time-sensitive external 
//digital circuits you have hooked up. Alternatively, you could set both pins high at 
//exactly the same moment in time using PORTB |= B1100;
//DDRD = B11111110;  // sets Arduino pins 1 to 7 as outputs, pin 0 as input
//DDRD = DDRD | B11111100;  // this is safer as it sets pins 2 to 7 as outputs
//                          // without changing the value of pins 0 & 1, which are RX & TX
DDRD = 0x0F;  // IMPORTANT: from pin 0 to 7 is port D, from pin 8 to 13 is port B
PORTD=0xFF;
DDRB =0xFF;
PORTB =0x00;

LD1207_init();

clear_VFD();

//only here I active the enable of interrupts to allow run the test of VFD
//interrupts();             // enable all interrupts
sei();
}

/******************************************************************/
/************************** Update Clock **************************/
/******************************************************************/
void send_update_clock(void)
{
    if (secs >=60){
      secs = 0;
      mins++;
    }
    if (mins >=60){
      mins = 0;
      hrs++;
    }
    if (hrs >=24){
      hrs = 0;
    }
  
    //*************************************************************
    numberA0=DigitTo7SegEncoder(secs%10);
    numberB0=DigitTo7SegEncoder(secs/10);
    //*************************************************************
    numberC0=DigitTo7SegEncoder(mins%10);
    numberD0=DigitTo7SegEncoder(mins/10);
    //**************************************************************
    numberE0=DigitTo7SegEncoder(hrs%10);
    numberF0=DigitTo7SegEncoder(hrs/10);
    //**************************************************************
   //The lines below are just to continue with the debug, leave comments!
   //Serial.print(numberF0, DEC);Serial.print(":");Serial.print(numberE0, DEC);Serial.print(":");Serial.print(numberD0, DEC); Serial.print(":");
   //Serial.print(numberC0, DEC);Serial.print(":");Serial.print(numberB0, DEC);Serial.print(":");Serial.println(numberA0, DEC);
}
unsigned char DigitTo7SegEncoder( unsigned char digit)
{
  // Note the array (segments[]) to draw the numbers is with 2 bytes!!! 20 chars, extract 2 by 2 the number you need. 
  switch(digit)
  {
    case 0:   number=0;      break;  // if remove the LongX, need put here the segments[x]
    case 1:   number=1;      break;
    case 2:   number=2;      break;
    case 3:   number=3;      break;
    case 4:   number=4;      break;
    case 5:   number=5;      break;
    case 6:   number=6;      break;
    case 7:   number=7;      break;
    case 8:   number=8;      break;
    case 9:   number=9;      break;
  }
  return number;
} 
void adjustHMS()
{
 // Important is necessary put a pull-up resistor to the VCC(+5VDC) to this pins (3, 4, 5)
 // if dont want adjust of the time comment the call of function on the loop
  /* Reset secs to 00 Pin number 3 Switch to GND*/
    if((AdjustPins & 0x80) == 0 )
    {
      delayMicroseconds(200);
      secs=00;
    }
    
    /* Set mins when SegCntrl Pin 4 Switch is Pressed*/
    if((AdjustPins & 0x40) == 0 )
    {
      delayMicroseconds(200);
      if(mins < 59)
      mins++;
      else
      mins = 0;
    }
    /* Set hrs when SegCntrl Pin 5 Switch is Pressed*/
    if((AdjustPins & 0x20) == 0 )
    {
      delayMicroseconds(200);
      if(hrs < 23)
      hrs++;
      else
      hrs = 0;
    }
}
void readButtons()
{
//This function is to read the buttons by serial data pin! Not key mod direct!
//Take special attention to the initialize digital pin LED_BUILTIN as an output.
//
int ledPin = 13;   // LED connected to digital pin 13
int inPin = 7;     // pushbutton connected to digital pin 7
int val = 0;       // variable to store the read value
int dataIn=0;

byte array[8] = {0,0,0,0,0,0,0,0};
byte together = 0;

unsigned char receive = 7; //define our transmit pin
unsigned char data = 0; //value to transmit, binary 10101010
unsigned char mask = 1; //our bitmask

array[0] = 1;

unsigned char btn1 = 0x41;

      digitalWrite(VFD_stb, LOW);
        delayMicroseconds(2);
      send_data8(0b01000010); // cmd 2 //10=Read Keys; 00=Wr DSP;
      delayMicroseconds(2);
       // send_data8((0b11000000)); //cmd 3 wich define the start address (00H to 15H)
     // send without stb
  
  pinMode(7, INPUT);  // Important this point! Here I'm changing the direction of the pin to INPUT data.
  delayMicroseconds(2);
  //PORTD != B01010100; // this will set only the pins you want and leave the rest alone at
  //their current value (0 or 1), be careful setting an input pin though as you may turn 
  //on or off the pull up resistor  
  //This don't send the strobe signal, to be used in burst data send
         for (int z = 0; z < 5; z++){
             //for (mask=00000001; mask > 0; mask <<= 1) { //iterate through bit mask
                   for (int h =8; h > 0; h--) {
                      digitalWrite(VFD_clk, HIGH);  // Remember wich the read data happen when the clk go from LOW to HIGH! Reverse from write data to out.
                      delayMicroseconds(2);
                     val = digitalRead(inPin);
                      //digitalWrite(ledPin, val);    // sets the LED to the button's value
                           if (val & mask){ // if bitwise AND resolves to true
                             //Serial.print(val);
                            //data =data | (1 << mask);
                            array[h] = 1;
                           }
                           else{ //if bitwise and resolves to false
                            //Serial.print(val);
                           // data = data | (1 << mask);
                           array[h] = 0;
                           }
                    digitalWrite(VFD_clk, LOW);
                    delayMicroseconds(2);
                   } 
             
              Serial.print(z);  // All the lines of print is only used to debug, comment it, please!
              Serial.print(" - " );
                        
                                  for (int bits = 7 ; bits > -1; bits--) {
                                      Serial.print(array[bits]);
                                   }
                        
                          
                          if (z==0){
                            if(array[5] == 1){
                             flagSecs = !flagSecs;  // This change the app to hrs or secs
                            }
                        }
                        
                         
                        if (z==0){
                            if(array[7] == 1){
                              digitalWrite(10, !digitalRead(10));
                          }
                        }
                        
                        if (z==3){
                           if(array[7] == 1){
                             //digitalWrite(VFD_onRed, !digitalRead(VFD_onRed));
                             //digitalWrite(VFD_onGreen, !digitalRead(VFD_onGreen));
                            }
                          }                        
                  Serial.println();
          }  // End of "for" of "z"
      Serial.println();  // This line is only used to debug, please comment it!

 digitalWrite(VFD_stb, HIGH);
 delayMicroseconds(2);
 send_command((0b10001000) | 7); //cmd 4
 delayMicroseconds(2);
 pinMode(7, OUTPUT);  // Important this point! Here I'm changing the direction of the pin to OUTPUT data.
 delay(1); 
}
void loop()
{
test_CleanSeg();
test_Seg();
test_msg();
delay(2000);
test_0to4();
delay(2000);
test_5to9();
delay(2000);

// I decide not use the dinamic refreshing, but you can do it!
       while(1){
              if (flag=!flag){
                send_update_clock();
                delay(100);
                //readButtons();  //This function is only to be used at drivers like PT6315 or PT6311...
                adjustHMS();
                delay(100);
                 write_panel_DVD();//
             }
       }
}

ISR(TIMER1_COMPA_vect)   {  //This is the interrupt request
                            // https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
      secs++;
      flag = !flag; 
} 
