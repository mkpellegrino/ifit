#include <LiquidCrystal.h>

int buffer[32];  // tx & rx buffer
int response_count = 0;

//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);
int the_button_that_was_pushed;

void clearBuffer()
{
    for( int i=0; i<32; i++ )
    {
      buffer[i] = 0;
    }
  return;
}

void setup()
{
  clearBuffer();
  Serial.begin(9600, SERIAL_8N1);
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  //         0123456789ABCDEF
  lcd.print(" CSAFE TRX ^v<> ");
}

void sendBuffer()
{
  lcd.setCursor(0,0);
  int send_count=0;
  int eof=0;
  while( eof != 0xF2 )
    {
      Serial.write( buffer[send_count] );
      lcd.print( buffer[send_count], HEX );
      lcd.print( " " );    
      eof=buffer[send_count];
      send_count++;
    }
  clearBuffer();

}

void getResponse()
{
  lcd.setCursor(0,1);
  lcd.print("<<");

  response_count=0;
  int eof = 0;
  while(eof != 0xF2)
//  while(eof != 0x7A)
    {
      if( Serial.available() )
	{
	  int rx = Serial.read();
	  lcd.print(rx, HEX);
	  lcd.print(" ");
	  buffer[response_count]=rx;
	  eof=rx;
	  response_count++;
	}
    }
}

void emptyFrame()
{
  buffer[0] = 0xF1;
  buffer[1] = 0x00;
  buffer[2] = 0xF2;
  sendBuffer();
  getResponse();

}

void cmdAutoUpload()
{
  buffer[0]=0xF1;
  buffer[1]=0x01;
  buffer[2]=0x08;
  buffer[3]=0xF2;
  sendBuffer();
  getResponse();

}

void cmdGoIdle()
{
  buffer[0]=0xF1;
  buffer[1]=0x82;
  buffer[2]=0x82;
  buffer[3]=0xF2;
  sendBuffer();
  getResponse();
}

void cmdReset()
{
  buffer[0]=0xF1;
  buffer[1]=0x81;
  buffer[2]=0x81;
  buffer[3]=0xF2;
  sendBuffer();
  getResponse();
}

void cmdGoReady()
{
  buffer[0]=0xF1;
  buffer[1]=0x87;
  buffer[2]=0x87;
  buffer[3]=0xF2;
  sendBuffer();
  getResponse();
}

void cmdGoInUse()
{
  buffer[0]=0xF1;
  buffer[1]=0x85;
  buffer[2]=0x85;
  buffer[3]=0xF2;
  sendBuffer();
  getResponse();
}

void cmdGoFinished()
{
  buffer[0]=0xF1;
  buffer[1]=0x86;
  buffer[2]=0x86;
  buffer[3]=0xF2;
  sendBuffer();
  getResponse();
}

void cmdSetGear()
{
  // 0xF1 0x29 0x05 0x2C 0xF2
  buffer[0] = 0xF1;
  buffer[1] = 0x29;
  buffer[2] = 0x05;
  buffer[3] = 0x2C;
  buffer[4] = 0xF2;
  sendBuffer();
  getResponse();
}

void cmdSetGrade()
{
  //0xF1 0x28 0x32 0x00 0x4A 0x50 0xF2 
  buffer[0] = 0xF1;
  
  buffer[1] = 0x28;
  buffer[2] = 0x32;
  buffer[3] = 0x00;
  buffer[4] = 0x4A;
  buffer[5] = 0x50;
  buffer[6] = 0xF2;
  sendBuffer();
  getResponse();
}

void loop()
{
  the_button_that_was_pushed = analogRead(0);

  if (the_button_that_was_pushed < 60) 
    {
      clearBuffer();
      cmdSetGear();
    }
  else if (the_button_that_was_pushed < 200)
    {
      clearBuffer();
      cmdSetGrade(); 
    }
  else if (the_button_that_was_pushed < 400)
    {
      clearBuffer();
      cmdGoReady();
    }
  else if (the_button_that_was_pushed < 600)
    {
      clearBuffer();
      cmdReset();
    }
  else if (the_button_that_was_pushed < 800)
    {
      clearBuffer();
      cmdGoInUse();
    }
      
  if( response_count > 0 )
    {
      // process the response that's stored in buffer
    }
}
