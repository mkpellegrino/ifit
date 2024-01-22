#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
using namespace std;
#include <vector>
#include <string>
#include <cstring>




// Uses POSIX serial port functions to send and receive data to and from an iFit enabled device.
#define cmdEmptyFrame 0x00
#define cmdGetStatus 0x80
#define cmdReset 0x81
#define cmdGoIdle 0x82
#define cmdGoHaveID 0x83
#define cmdGoInUse 0x85
#define cmdGoFinished 0x86
#define cmdGoReady 0x87
#define cmdBadID 0x88
#define cmdAutoUpload 0x01
#define cmdUpList 0x02
#define cmdUpStatusSec 0x04
#define cmdUpListSec 0x05

#define cmdIDDigits 0x10
#define cmdSetTime 0x11
#define cmdSetDate 0x12
#define cmdSetTimeout 0x13
#define cmdSetTWork 0x20
#define cmdSetHorizontal 0x21
#define cmdSetVertical 0x22
#define cmdSetCalories 0x23
#define cmdSetProgram 0x24
#define cmdSetSpeed 0x25
#define cmdSetGrade 0x28
#define cmdSetGear 0x29
#define cmdSetUserInfo 0x2B
#define cmdSetTorque 0x2C
#define cmdSetLevel 0x2D
#define cmdSetTargetHR 0x30
#define cmdSetGoal 0x32
#define cmdSetMETS 0x33
#define cmdSetPower 0x34
#define cmdSetHRZone 0x35
#define cmdSetHRMax 0x36
#define cmdSetChannelRange 0x40
#define cmdSetVolumenRange 0x41
#define cmdSetAudioMute 0x42
#define cmdSetAudioChannel 0x43
#define cmdSetAudioVolume 0x44




#define qryGetCaps 0x70
#define qryGetOdometer 0x98
#define qryGetUtilization 0x99

#define qryGetTWork 0xA0
#define qryGetHorizontal 0xA1
#define qryGetVertical 0xA2
#define qryGetCalories 0xA3
#define qryGetProgram 0xA4
#define qryGetSpeed 0xA5
#define qryGetPace 0xA6
#define qryGetCadence 0xA7
#define qryGetGrade 0xA8
#define qryGetGear 0xA9

#define qryGetUserInfo 0xAB

#define flgAutoStatus 0x00
#define flgUpStatus 0x01
#define flgUpList 0x02
#define flgAck 0x04
#define flgExternControl 0x06


#define untNone 0x00
#define untMale 0x01
#define untFemale 0x02
#define untMile 0x01
#define untTenthOfMile 0x02
#define untHundredthOfMile 0x03
#define untThousandthOfMile 0x04
#define untFeet 0x05
#define untInch 0x06
#define untPounds 0x07
#define untTenthOfPound 0x08
#define untTenFeet 0x0A
#define untMPH 0x10 // miles/hour
#define untTenthOfMPH 0x11
#define untHundredthOfMPH 0x12
#define untFPM 0x13 // ft/minute
#define untKM 0x21 // Kilometers
#define untTenthOfKM 0x22
#define untHundredthOfKM 0x23
#define untM 0x24 // Meter
#define untTenthOfM 0x25
#define untCM 0x26 // Centimeter
#define untKG 0x27 // Kilogram
#define untTenthOfKG 0x28
#define untKPH 0x30 // Kilometers/Hour
#define untTenthOfKPH 0x31
#define untHundredthOfKPH 0x32
#define untMPM 0x33 // Meters/Minute

#define untPercentGrade 0x4A // Percent Grade
#define untHundredthOfPercentGrade 0x4B // Percent Grade
#define untTenthOfPercentGrade 0x4C // Percent Grade


// the file descriptor for the serial port
int fd;

// the frame buffer
vector <uint8_t> frame_buffer;
vector <uint8_t> checksum_buffer;
vector <uint8_t> packet_buffer;

ssize_t received;

// The Maximum Number of Bytes that the CSAFE Protocol is meant to handle is 120
uint8_t send_buffer[120] =
  {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

uint8_t rx_buffer1[1] = { 0x00 };
uint8_t rx_buffer2[2] = { 0x00, 0x00 };
uint8_t rx_buffer3[3] = { 0x00, 0x00, 0x00 };






// helper function
void delay(){ usleep(50000); }

// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char * device, uint32_t baud_rate)
{
  
#ifdef DEBUG
  fprintf( stderr, "open serial port\n" );
#endif
  //fd = open(device, O_RDWR | O_NOCTTY);
  // O_RDWR - Read and Write
  // O_NOCTTY - The port never becomes the controlling terminal of the process
  // O_NDELAY - Use non-blocking I/O
  // O_NONBLOCK - same
  //fd = open(device, (O_RDWR | O_NOCTTY | O_NDELAY) );
  fd = open(device, (O_RDWR | O_NOCTTY | O_NONBLOCK) );
#ifdef DEBUG
  if( fd != 0 )
    {
      fprintf( stderr, "open serial port: success\n" );
    }
  else
    {
      fprintf( stderr, "open serial port: failed\n" );
      exit(-1);
    }
#endif

  if(!isatty(fd))
    {
      fprintf( stderr, "Device %d does not point to a serial device.\n", fd );
      return(-1);
    }

  if (fd == -1)
  {
    perror(device);
    return -1;
  }
 
  // Flush away any bytes previously read or written.
  int result = tcflush(fd, TCIOFLUSH);
  if (result)
    {
      fprintf( stderr, "flush failed\n" );
    }
  else
    {
      fprintf( stderr, "flushed old bytes\n" );
    }

  // Get the current configuration of the serial port.
  struct termios options;
  result = tcgetattr(fd, &options);
  if (result)
  {
    fprintf( stderr, "tcgetattr failed\n" );
    close(fd);
    return -1;
  }
  else
    {
      fprintf( stderr, "tcgetattr success\n" );
    }

  // -----------------------------------

  // Turn off any options that might interfere with our ability to send and
  // receive raw binary bytes.
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);




  // control flag options
  // CSIZE       /* character size mask */
  //    CS5         /* 5 bits (pseudo) */
  //    CS6         /* 6 bits */
  //    CS7         /* 7 bits */
  //    CS8         /* 8 bits */
  //    CSTOPB      /* send 2 stop bits */
  //    CREAD       /* enable receiver */
  //    PARENB      /* parity enable */
  //    PARODD      /* odd parity, else even */
  //    HUPCL       /* hang up on last close */
  //    CLOCAL      /* ignore modem status lines */
  //    CCTS_OFLOW  /* CTS flow control of output */
  //    CRTSCTS     /* same as CCTS_OFLOW */
  //    CRTS_IFLOW  /* RTS flow control of input */
  //    MDMBUF      /* flow control output via Carrier */

  
  // c_cflag
  //CRTSCTS : output hardware flow control (only used if the
  //cable has
  //all necessary lines. See sect. 7 of Serial-HOWTO)
  //CS8 : 8n1 (8bit,no parity,1 stopbit)
  //CLOCAL : local connection, no modem contol
  //CREAD : enable receiving characters

      options.c_iflag &= ~(INLCR | ICRNL);
      //options.c_iflag |= IGNPAR | IGNBRK;
      options.c_iflag |=  IGNBRK;
      //options.c_iflag |= IGNBRK;
    options.c_oflag &= ~(OPOST | ONLCR | OCRNL);


    // Turn OFF options we don't want
    options.c_cflag &= ~(CSIZE | CS5 | CS6 | CS7 | CSTOPB | CREAD | PARENB | CLOCAL | PARODD | HUPCL | CCTS_OFLOW | CRTS_IFLOW | MDMBUF | CSTOPB | CSIZE | CRTSCTS );


    options.c_cflag &= (CS8 | CSIZE | CREAD | CLOCAL | HUPCL | MDMBUF );

    //options.c_cflag |= CS5;
    

    //options.c_cflag |= CLOCAL | CREAD | CS8 ;

    
    //options.c_lflag &= ~(ICANON | ISIG | ECHO );
    options.c_lflag &= ~(ICANON | ISIG );


    // 1 = 100ms
    //options.c_cc[VTIME] = 1;

    // 10 should equal 1 sec
    options.c_cc[VTIME] = 10;

    // the minimum read-time is zero so that the program will read as soon as a byte is available
    options.c_cc[VMIN]  = 0;
    





  
  // -----------------------------------
  // This code only supports certain standard baud rates. Supporting
  // non-standard baud rates should be possible but takes more work.
  switch (baud_rate)
  {
  case 4800:
    cfsetospeed(&options, B4800);
    cfsetispeed(&options, B4800);
    break;
  case 9600:
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    break;
  case 19200:
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    break;
  case 38400:
    cfsetospeed(&options, B38400);
    cfsetispeed(&options, B38400);
    break;
  case 115200:
    cfsetospeed(&options, B115200);
    cfsetispeed(&options, B115200);
    break;
  default:
    fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
      baud_rate);
    cfsetospeed(&options, B9600);
    break;
  }


      //Apply settings
    //TCSANOW vs TCSAFLUSH? Was using TCSAFLUSH; settings source above
    //uses TCSANOW.
    if (tcsetattr(fd, TCSANOW, &options) < 0)
      {
      fprintf(stderr, "Error setting serial port attributes.\n");
      close(fd);
      return -2; //Using negative value; -1 used above for different failure
    }

 
  return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(uint8_t * buffer, size_t size)
{
  ssize_t result = write(fd, buffer, size);
  if (result != (ssize_t)size)
  {
    fprintf( stderr, "serial write failed\n" );
    return -1;
  }
  delay();

  return 0;
}
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
ssize_t read_port(uint8_t * buffer, size_t size)
{
  size_t received = 0;
  while (received < size)
  {
    ssize_t r = read(fd, buffer + received, size - received);
    if (r < 0)
    {
      fprintf( stderr, "serial read failed\n" );
      return -1;
    }
    if (r == 0)
    {
      // Timeout
      break;
    }
    received += r;
    delay();
  }

  return received;
}

void cleanBuffers()
{
  packet_buffer.erase (packet_buffer.begin(),packet_buffer.end());
  frame_buffer.erase (frame_buffer.begin(),frame_buffer.end());
  checksum_buffer.erase (checksum_buffer.begin(),checksum_buffer.end());
  return;
}

void insertFrameIntoPacket()
{
#ifdef DEBUG
  fprintf( stderr, "insert frame and checksum into packet\n" );
#endif  
  packet_buffer.push_back(0xF1);
  for( int j = 0; j < frame_buffer.size(); j++ )
    {
      packet_buffer.push_back(frame_buffer[j]);
    }

  if( frame_buffer[0] != 0x00 )
    {
#ifdef DEBUG
      fprintf( stderr, "inserting checksum into packet\n" );
#endif
      packet_buffer.push_back(checksum_buffer[0]);
    }
  packet_buffer.push_back(0xF2);
  return;
}

void displayPacket()
{
  int x = 3;
  if( frame_buffer[0] == 0x00 ) x--;
#ifdef DEBUG
  fprintf( stderr, "PACKET: " );
#endif
  for( int i = 0; i < frame_buffer.size()+x; i++ )
    {
      printf( "0x%02X ", packet_buffer[i] );
    }
  printf( "\n" );
  return;
}

void displayFrame()
{
#ifdef DEBUG
  fprintf( stderr, "FRAME: " );
#endif
  for( int i = 0; i < frame_buffer.size(); i++ )
    {
      printf( "0x%02X ", frame_buffer[i] );
    }
#ifdef DEBUG
  fprintf( stderr, "(%d)", frame_buffer.size() );
#endif
  
  printf( "\n" );
  return;
}

void displaySendBuffer()
{
#ifdef DEBUG
  fprintf( stderr, "SEND BUFFER: " );
#endif
  for( int i = 0; i < packet_buffer.size(); i++ )
    {
      printf( "0x%02X ", send_buffer[i] );
    }
#ifdef DEBUG
  fprintf( stderr, "(%d)", packet_buffer.size() );
#endif
  
  printf( "\n" );
  return;
}

void calculateChecksum()
{
  if( frame_buffer[0] == 0x00 ) return;
#ifdef DEBUG
  fprintf( stderr, "Calculating Checksum\n" );
#endif
  // initial checksum value to XOR with byte[0]
  int checksum_byte = 0x00;

  for( int i = 0; i < frame_buffer.size(); i++ )
    {
      checksum_byte = frame_buffer[i] ^ checksum_byte;
      
#ifdef DEBUG
      fprintf( stderr, "CHKSUM BYTE: 0x%02X\n", checksum_byte );
#endif
    }
  checksum_buffer.push_back( checksum_byte );
  return;
}

int cmd( int c, int data0 = 0x00, int data1 = 0x00, int data2 = 0x00, int data3 = 0x00, int data4 = 0x00)
{
#ifdef DEBUG
  fprintf( stderr, "cmd(0x%02X)\n", c );  
#endif
  cleanBuffers();
  frame_buffer.push_back(c);

  switch( c )
    {
    case cmdAutoUpload:
    case cmdUpStatusSec:
    case cmdUpListSec:
    case cmdIDDigits:
    case cmdSetTimeout:
    case cmdSetGear:
    case cmdSetLevel:
    case cmdSetTargetHR:
    case cmdSetHRMax:
    case cmdSetAudioMute: // 0 = muted  & 1 = not muted
    case cmdSetAudioChannel:
    case cmdSetAudioVolume:
      frame_buffer.push_back( data0 );
      break;
    case cmdSetCalories:
    case cmdSetProgram:
    case cmdSetMETS:
    case cmdSetHRZone:
    case cmdSetChannelRange:
    case cmdSetVolumenRange:
      frame_buffer.push_back( data0 );
      frame_buffer.push_back( data1 );
      break;
      
    case cmdSetTime:
    case cmdSetDate:
    case cmdSetTWork:
    case cmdSetHorizontal:
    case cmdSetVertical:
    case cmdSetSpeed:
    case cmdSetGrade:
    case cmdSetTorque:
    case cmdSetPower:
      frame_buffer.push_back( data0 );
      frame_buffer.push_back( data1 );
      frame_buffer.push_back( data2 );
      break;
      
    case cmdSetUserInfo:
      frame_buffer.push_back( data0 );
      frame_buffer.push_back( data1 );
      frame_buffer.push_back( data2 );
      frame_buffer.push_back( data3 );
      frame_buffer.push_back( data4 );
      
    default:
#ifdef DEBUG
      fprintf( stderr, "dataless command: 0x%02X\n", c );
#endif
      break;
    }


  // uint8_t test_arduino[1];
  // test_arduino[0] = 'h';
  // write_port( test_arduino, sizeof(test_arduino));
  // test_arduino[0] = 'e';
  // write_port( test_arduino, sizeof(test_arduino));
  // test_arduino[0] = 'l';
  // write_port( test_arduino, sizeof(test_arduino));
  // test_arduino[0] = 'o';
  // write_port( test_arduino, sizeof(test_arduino));

  // c = qryGetTWork;
  if( c!=0 ) calculateChecksum();
  
  insertFrameIntoPacket();

  displayPacket();

  int return_value = 0;
  for( int i = 0; i < packet_buffer.size(); i++ )
   {
    uint8_t command[1];
    command[0] = packet_buffer[i];
    return_value+= write_port( command, sizeof(command));
  }


  // if necessary, get the response
  switch( c )
    {
      // 3 bytes
    case qryGetTWork:
    case qryGetHorizontal:
    case qryGetSpeed:
    case qryGetPace:
    case qryGetCadence:
    case qryGetGrade:
      received = read_port( rx_buffer3, sizeof(uint8_t)*3);
      if (received < 0)
	{
	  return -1;
	}
      if (received != sizeof(uint8_t)*3)
	{
	  fprintf(stderr, "(0x%02X) read timeout: expected %u bytes, got %zu\n", c, sizeof(uint8_t)*3, received);
	  return -1;
	}
      printf( "(0x%02X) RX: 0x%02X 0x%02X 0x%02X\n", c, rx_buffer3[0], rx_buffer3[1], rx_buffer3[2] );
      break;
    case qryGetGear:
      received = read_port( rx_buffer1, sizeof(uint8_t));
      if (received < 0)
	{
	  return -1;
	}
      if (received != sizeof(uint8_t))
	{
	  fprintf(stderr, "(0x%02X) read timeout: expected %u bytes, got %zu\n", c, sizeof(uint8_t), received);
	  return -1;
	}
      printf( "(0x%02X) RX: 0x%02X\n", c, rx_buffer1[0] );
      break;
    default:
      // testing
      break;
    }

  return return_value;
}



 
int main()
{

  vector <string> portname_vector;

  portname_vector.push_back( "/dev/cu.usbmodem1421" );
  portname_vector.push_back( "/dev/tty.usbserial-A50285BI" );
  portname_vector.push_back( "/dev/cu.usbserial-A50285BI" );
  portname_vector.push_back( "/dev/tty.usbserial-A9XNR88X" );
  portname_vector.push_back( "/dev/cu.usbserial-A9XNR88X" );
  portname_vector.push_back( "/dev/tty.usbmodem1421" );
  portname_vector.push_back( "/dev/cu.usbmodem1421" );
  portname_vector.push_back( "/dev/tty.usbserial-5" );
  portname_vector.push_back( "/dev/cu.MRP-SerialPort" );
  portname_vector.push_back( "/dev/ttys0" );
  portname_vector.push_back( "/dev/ttys001" );
  portname_vector.push_back( "/dev/ttys000" );

  fprintf( stderr, "Choose a serial port from the list:\n" );
  for( int i=0; i< portname_vector.size(); i++ )
    {
      fprintf( stderr, "%d) %s\n", i+1, portname_vector[i].c_str() );
    }
  int choice = 0x00;
  scanf( "%d", &choice );
  choice--;

  
  //char * device;
  //device =  portname_vector[choice].c_str();
  
  // Linux USB example:          "/dev/ttyACM0"  (see also: /dev/serial/by-id)
  // macOS USB example:          "/dev/cu.usbmodem001234562"
  // Cygwin example:             "/dev/ttyS7"
  //const char * device = "/dev/tty.usbserial-A50285BI";
  //const char * device = "/dev/cu.usbserial-A50285BI";
  //const char * device = "/dev/tty";
  //const char * device = "/dev/tty.usbmodem1421";
  //const char * device = "/dev/tty.usbserial-5";
  //const char * device = "/dev/ttys0";
  //const char * device = "/dev/cu.MRP-SerialPort";

  uint32_t baud_rate = 9600;
  //uint32_t baud_rate = 115200;


  fd = open_serial_port(portname_vector[choice].c_str(), baud_rate );
  //fd = open_serial_port(device, baud_rate);
  if (fd < 0)
    {
      fprintf( stderr, "unable to connect to serial port: %s\n", portname_vector[choice].c_str() );
      return 1;
    }
#ifdef DEBUG
  fprintf( stderr, "connected to serial port: %s\n", portname_vector[choice].c_str() );	  
#endif
  int h=0;

  // Ready -> Idle -> HaveID -> InUse
  printf( "cmdReset (enter a number)\n" );
  cmd(cmdReset);
  scanf( "%d", &h );
  
  
  printf( "cmdGoReady (enter a number)\n" );
  cmd(cmdGoReady);
  scanf( "%d", &h );
  
  printf( "cmdGoIdle (enter a number)\n" );
  cmd(cmdGoIdle);
  scanf( "%d", &h );

  printf( "AutoUpload (enter a number)\n" );
  cmd(cmdAutoUpload, flgAutoStatus );
  scanf( "%d", &h );


  printf( "cmdGoHaveID (enter a number)\n" );
  cmd(cmdGoHaveID);
  scanf( "%d", &h );

  printf("cmdGoInUse (enter a number)\n" );	 
  cmd(cmdGoInUse);
  scanf( "%d", &h );

  printf( "cmdSetSpeed(2.5)\n" );
  cmd(cmdSetSpeed, 25, 0x00, 0x11);
  scanf( "%d", &h );


  printf( "SetUserInfo\n" );
  cmd(cmdSetUserInfo, 215, 0, untPounds, 50, untMale );
  scanf( "%d", &h );
  
  printf( "SetGrade\n" );
  cmd(cmdSetGrade, 50, 0, untPercentGrade);
  scanf( "%d", &h );
  
  printf( "SetGear\n" );
  cmd(cmdSetGear, 5 );
  scanf( "%d", &h );

  
  printf( "SetTorque\n" );
  cmd(cmdSetTorque, 5, 0, 90);
  scanf( "%d", &h );
  
  printf( "SetLevel\n" );
  cmd(cmdSetLevel, 5);
  scanf( "%d", &h );

  printf("cmdGoFinished\n" );
  cmd(cmdGoFinished);
  scanf( "%d", &h );

  
  //cmd(cmdSetGear, 0x80);

  //cmd(cmdSetSpeed, 0x19, 0x00, untTenthOfMPH );
  
  //cmd(cmdEmptyFrame);
  //cmd(cmdGetStatus);


  //cmd(cmdGoInUse);
  //cmd(cmdGoFinished);
  //cmdSetResistance(5);
  //scanf( "%d", &h );
  //cmd(qryGetGrade);
  //cmd(qryGetGrade);
  //cmd(qryGetHorizontal);

  
  close(fd);
  return 0;
}
