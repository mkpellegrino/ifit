#include <LiquidCrystal.h>

byte tx_buffer[32];  // rx buffer
byte rx_buffer[32];  // rx buffer
int response_count = 0;
int tx_length=0;

//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);
void setup()
{
  for( int i=0; i<32; i++ )
    {
      tx_buffer[i] = 0;
      rx_buffer[i] = 0;
    }

  Serial.begin(9600, SERIAL_8N1);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("   tx/rx 8N1");
}

void init_buffers()
{
  for( int i=0; i<32; i++ )
  {
    tx_buffer[i] = 0x00;
    rx_buffer[i] = 0x00;
  }
}

void init_rx_buffer()
{
  for( int i=0; i<32; i++ )
  {
    rx_buffer[i] = 0x00;
  }
}

void init_tx_buffer()
{
  for( int i=0; i<32; i++ )
  {
    tx_buffer[i] = 0x00;
  }
}

bool isValid()
{
  if( rx_buffer[0] == 'A' ) return true;
   if( rx_buffer[0] != 0xF1 ) return false;
   for( int i=1; i<11; i++ )
   {
    if( rx_buffer[i] == 0xF2 ) return true;
   }
   return false;
  
}

void getDataFromLaptop()
{
  lcd.setCursor(0,1);
  lcd.print("<< ");
  
  response_count=Serial.readBytes(rx_buffer, 10);

  if( response_count > 0 )
  {
     // display response
     for( int i=0; i < response_count; i++ )
     {
        lcd.print( rx_buffer[i], HEX);
        lcd.print( " " );    
     }
  }
}

void loop()
{
  getDataFromLaptop();
  if( response_count > 0 )
    {
      // process the response that's stored in buffer
      if( isValid() ) 
      {
        int cmd = rx_buffer[1];
        init_tx_buffer(); 
        switch( cmd )
        {
          case 0x92:
            // cmdGetID
            tx_buffer[0]=0xF1;
            tx_buffer[1]=0x01;
            tx_buffer[2]=0x02;
            tx_buffer[3]=0x03;
            tx_buffer[4]=0x04;
            tx_buffer[5]=0x05;
            tx_buffer[6]=0x01;
            tx_buffer[7]=0xF2;
            tx_length=8;
            break;
          case 0xA8:
          // cmdGetGrade
            tx_buffer[0]=0xF1;
            tx_buffer[1]=0x07;
            tx_buffer[2]=0x00;
            tx_buffer[3]=0x4A;
            tx_buffer[4]=0x4D;
            tx_buffer[5]=0xF2;
            tx_length=6;
            break;
          default:
          // empty frame
            tx_buffer[0]=0xF1;
            tx_buffer[1]=0x00;
            tx_buffer[2]=0xF2;
            tx_length=3;
        }              
        Serial.write( tx_buffer, tx_length );
      }
      response_count=0;
    }
}
