#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
using namespace std;
#include <vector>
#include <string>
#include <cstring>

bool auto_upload_set = false;

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
#define cmdSetVolumeRange 0x41
#define cmdSetAudioMute 0x42
#define cmdSetAudioChannel 0x43
#define cmdSetAudioVolume 0x44

#define qryGetCaps 0x70
#define qryGetVersion 0x91
#define qryGetID 0x92
#define qryGetUnits 0x93
#define qryGetSerial 0x94
#define qryGetList 0x98
#define qryGetUtilization 0x99
#define qryGetMotorCurrent 0x9A
#define qryGetOdometer 0x9B
#define qryGetErrorCode 0x9C
#define qryGetServiceCode 0x9D
#define qryGetUserCfg1 0x9E
#define qryGetUserCfg2 0x9F

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
#define qryGetUpList 0xAA
#define qryGetUserInfo 0xAB
#define qryGetTorque 0xAC
#define qryGetHRCur 0xB0
#define qryGetHRTZone 0xB2
#define qryGetMETS 0xB3
#define qryGetPower 0xB4
#define qryGetHRAvg 0xB5
#define qryGetHRMax 0xB6
#define qryGetUserData1 0xBE
#define qryGetUserData2 0xBF

#define qryGetAudioChannel 0xC0
#define qryGetAudioVolume 0xC1
#define qryGetAudioMute 0xC2




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

class menu_item
{
public:
  menu_item( string s, uint8_t b )
  {
    number = b;
    text = new string(s.c_str());
  };
  void display(){ fprintf( stderr, "%s %d", text->c_str(), number); };
  uint8_t getNumber(){return number; };
  string * getText(){return text; };
private:
  string * text;
  uint8_t number;
};

vector <menu_item*> menu_items;
  
// the file descriptor for the serial port
int fd;

// the frame buffer
vector <uint8_t> frame_buffer;
vector <uint8_t> checksum_buffer;
vector <uint8_t> packet_buffer;

int received;

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

uint8_t rx_buffer[120] =
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



// helper function
void delay(){ usleep(500000); }

// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char * device, uint32_t baud_rate)
{
  
#ifdef DEBUG
  fprintf( stderr, "attempting to open serial port: %s\n", device );
#endif
  //fd = open(device, O_RDWR | O_NOCTTY);
  // O_RDWR - Read and Write
  // O_NOCTTY - The port never becomes the controlling terminal of the process
  // O_NDELAY - Use non-blocking I/O
  // O_NONBLOCK - same

  //fd = open(device, (O_RDWR | O_NOCTTY | O_NDELAY  );
  
  fd = open(device, (O_RDWR | O_NOCTTY | O_NONBLOCK) );
  if( fd != 0 )
    {
#ifdef DEBUG
      fprintf( stderr, "fd = open(device, (O_RDWR | O_NOCTTY | O_NONBLOCK) ); success\n" );
#endif
    }
  else
    {
      fprintf( stderr, "fd = open(device, (O_RDWR | O_NOCTTY | O_NONBLOCK) ); failed\n" );
      exit(-1);
    }

  if(!isatty(fd))
    {
      fprintf( stderr, "Device %d does not point to a serial device.\n", fd );
      return(-1);
    }

  if (fd == -1)
    {
      fprintf( stderr, "Device error: %d\n", fd );
      return -1;
    }
 
  // Flush away any bytes previously read or written.
  int result = tcflush(fd, TCIOFLUSH);
  if (result)
    {
      fprintf( stderr, "tcflush(fd, TCIOFLUSH); failed\n" );
    }
#ifdef DEBUG
  else
    {
      fprintf( stderr, "tcflush(fd, TCIOFLUSH); success\n" );
    }
#endif

  // Get the current configuration of the serial port.
  /* https://www.gnu.org/software/libc/manual/html_node/Setting-Modes.html */
  struct termios options;
  result = tcgetattr(fd, &options);
  if (result)
    {
      fprintf( stderr, "result = tcgetattr(fd, &options); failed\n" );
      close(fd);
      return -1;
    }
#ifdef DEBUG
  else
    {
      fprintf( stderr, "result = tcgetattr(fd, &options); success\n" );
    }
#endif
  // -----------------------------------

  // Turn off any options that might interfere with our ability to send and
  // receive raw binary bytes.
  
  /* TERMIOS INPUT FLAGS */
  /* https://www.gnu.org/software/libc/manual/html_node/Input-Modes.html         */
  /*                    AND                                                      */
  /* https://opensource.apple.com/source/xnu/xnu-792/bsd/sys/termios.h.auto.html */
  
  // IGNBRK		0x00000001	/* ignore BREAK condition */
  // BRKINT		0x00000002	/* map BREAK to SIGINTR */
  // IGNPAR		0x00000004	/* ignore (discard) parity errors */
  // PARMRK		0x00000008	/* mark parity and framing errors */
  // INPCK		0x00000010	/* enable checking of parity errors */
  // ISTRIP		0x00000020	/* strip 8th bit off chars */
  // INLCR		0x00000040	/* map NL into CR */
  // IGNCR		0x00000080	/* ignore CR */
  // ICRNL		0x00000100	/* map CR to NL (ala CRMOD) */
  // IXON	         	0x00000200	/* enable output flow control */
  // IXOFF		0x00000400	/* enable input flow control */
  // IXANY		0x00000800	/* any char will restart after stop */
  // IMAXBEL		0x00002000	/* ring bell on input queue full */
  
  options.c_iflag &= ~(IXOFF|IXON|IXANY|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
  //options.c_iflag |= (IGNBRK|IGNPAR);

  /* TERMIOS OUTPUT FLAGS */
  /* https://www.gnu.org/software/libc/manual/html_node/Output-Modes.html */
  /*                    AND                                                      */
  /* https://opensource.apple.com/source/xnu/xnu-792/bsd/sys/termios.h.auto.html */
  // OPOST		0x00000001	/* enable following output processing */
  // ONLCR		0x00000002	/* map NL to CR-NL (ala CRMOD) */
  // OXTABS		0x00000004	/* expand tabs to spaces */
  // ONOEOT		0x00000008	/* discard EOT's (^D) on output) */

  //options.c_oflag &= ~(OPOST|ONLCR|OCRNL);
  options.c_oflag &= ~(OPOST);
  
  /* TERMIOS LOCAL FLAGS */
  
  /* https://www.gnu.org/software/libc/manual/html_node/Local-Modes.html */

  
  options.c_lflag &= ~(ECHO|ECHOE|ICANON|ISIG);
  
  
  /* TERMIOS CONTROL FLAGS */

  // Make sure to wire up CTS (on the cx938) to RTS (on the computer)
  // otherwise don't use CRTS_IFLOW

  /* https://www.gnu.org/software/libc/manual/html_node/Control-Modes.html */
  /*                    AND                                                      */
  /* https://opensource.apple.com/source/xnu/xnu-792/bsd/sys/termios.h.auto.html */

  // CIGNORE		0x00000001	/* ignore control flags */
  // CSIZE		0x00000300	/* character size mask */
  // CS5		    0x00000000	    /* 5 bits (pseudo) */
  // CS6		    0x00000100	    /* 6 bits */
  // CS7		    0x00000200	    /* 7 bits */
  // CS8		    0x00000300	    /* 8 bits */
  // CSTOPB		0x00000400	/* send 2 stop bits */
  // CREAD		0x00000800	/* enable receiver */
  // PARENB		0x00001000	/* parity enable */
  // PARODD		0x00002000	/* odd parity, else even */
  // HUPCL		0x00004000	/* hang up on last close */
  // CLOCAL		0x00008000	/* ignore modem status lines */
  // CCTS_OFLOW	0x00010000	/* CTS flow control of output */
  // CRTSCTS		(CCTS_OFLOW | CRTS_IFLOW)
  // CRTS_IFLOW	0x00020000	/* RTS flow control of input */
  // CDTR_IFLOW	0x00040000	/* DTR flow control of input */
  // CDSR_OFLOW	0x00080000	/* DSR flow control of output */
  // CCAR_OFLOW	0x00100000	/* DCD flow control of output */
  // MDMBUF		0x00100000	/* old name for CCAR_OFLOW */
  //options.c_cflag &= ~(CSIZE|CSTOPB|PARENB|CRTSCTS);
  //options.c_cflag &= ~(CSIZE|CSTOPB|PARENB|CRTSCTS);
  options.c_cflag &= ~(CSIZE|PARENB|CCTS_OFLOW|CRTS_IFLOW|CIGNORE|CSTOPB|PARENB|PARODD);
  options.c_cflag |= (CS8|CREAD|CLOCAL);
  //options.c_cflag |= (CS8);



  // THESE ARE FOR Noncanonical mode ONLY
  // 1 = 100ms
  //options.c_cc[VTIME] = 1;

  // 10 should equal 1 sec (max time to wait for a byte 
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
      fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n", baud_rate);
      cfsetospeed(&options, B9600);
      break;
    }

#ifdef DEBUG
  fprintf( stderr, "baudrate set to %u\n", baud_rate );
#endif

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
int read_port(uint8_t * buffer )
{
  int  received = 0;
  //while (received < size)
  while (received==0)
    {
      //ssize_t r = read(fd, buffer + received, size - received);
      int r = read(fd, buffer, sizeof(buffer)-1);
      
      if (r < 0)
	{
	  fprintf( stderr, "serial read failed\n" );
	  return -1;
	}
      buffer[r] = '\0';
      if (r == 0)
	{
	  // Timeout
	  break;
	}
      received = r;
      delay();
    }
  // for( int i=0; i<received; i++ )
  //   {
  //    buffer[i]^=0xFF;
  //    buffer[i]|=0x80;
  //  }
  return received;
}

int read_many()
{
  vector <ssize_t> tmp_rx_buffer;

  uint8_t buffer_of_one[1];
  
  size_t size = sizeof(buffer_of_one);
  bool keep_reading = true;
  while (keep_reading)
    {
      ssize_t r = read(fd, buffer_of_one, size);
      if (r <= 0)
	{
	  fprintf( stderr, "no (more) data to read\n" );
	  keep_reading = false;
	}
      else
	{
	  tmp_rx_buffer.push_back(buffer_of_one[0]);
	  delay();
	}
    }

  // DUMP THE ARRAY
  for( int i=0; i<tmp_rx_buffer.size(); i++ )
    {
      printf( "0x%2X ", tmp_rx_buffer[i] );
    }
  return tmp_rx_buffer.size();
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
  packet_buffer.push_back(0xF1);
  for( int j = 0; j < frame_buffer.size(); j++ )
    {
      packet_buffer.push_back(frame_buffer[j]);
    }

  if( frame_buffer[0] != 0x00 )
    {
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
#ifdef DEBUG1
  fprintf( stderr, "Calculating Checksum\n" );
#endif
  // initial checksum value to XOR with byte[0]
  int checksum_byte = 0x00;

  for( int i = 0; i < frame_buffer.size(); i++ )
    {
      checksum_byte = frame_buffer[i] ^ checksum_byte;
      
#ifdef DEBUG1
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
    case cmdSetVolumeRange:
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
      // expecting 3 bytes
    case qryGetTWork:
    case qryGetHorizontal:
    case qryGetSpeed:
    case qryGetPace:
    case qryGetCadence:
    case qryGetGrade:
      break;
      // expecting 1 byte
    case qryGetGear:
      break;
    default:
      // testing
      break;
    }
  delay();
  delay();
  received = read_port( rx_buffer );
  
  if (received < 0)
    {
      return -1;
    }
  
  printf( "[cmd 0x%02X] RX: ", c );
  for( int i = 0; i<received; i++ )
    {
      printf( "0x%02X ", rx_buffer[i]);
    }
  printf( "\n" );

  return return_value;
}

void showMenu()
{
  for( int i = 0; i < menu_items.size(); i+=5 )
    {
      for( int j =0; j<5; j++ )
	{
	  if( i+j < menu_items.size() )
	    {
	      fprintf( stderr, "%d) %s\t", i+j, menu_items[i+j]->getText()->c_str() );
	    }
	  
	}
      printf( "\n" );
    }
  fprintf( stderr, "\t\t\t\t(-1) Exit Program\n" );

}

 
int main()
{

  vector <string> portname_vector;

  portname_vector.push_back( "/dev/tty.usbmodem1421" );
  portname_vector.push_back( "/dev/tty.usbserial-A50285BI" );
  portname_vector.push_back( "/dev/tty.usbserial-A9XNR88X" );
  portname_vector.push_back( "/dev/tty.usbserial-5" );
  portname_vector.push_back( "/dev/tty.MRP-SerialPort" );
  portname_vector.push_back( "/dev/ttys0" );
  //portname_vector.push_back( "/dev/ttys001" );
  //portname_vector.push_back( "/dev/ttys000" );

  fprintf( stderr, "Choose a serial port from the list:\n" );
  for( int i=0; i< portname_vector.size(); i++ )
    {
      fprintf( stderr, "%d) %s\n", i, portname_vector[i].c_str() );
    }
  int choice = 0x00;

  fprintf( stderr, "#>" );
  scanf( "%d", &choice );

  fprintf( stderr, "You entered %d\n", choice );

  //choice--;

  
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

  uint32_t baud_rate = B9600;
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



  menu_items.push_back( new menu_item( string("Empty Frame"), cmdEmptyFrame ));
  menu_items.push_back( new menu_item( string("Get Status"), cmdGetStatus ));
  menu_items.push_back( new menu_item( string("Reset"), cmdReset ));
  menu_items.push_back( new menu_item( string("Go Idle"), cmdGoIdle ));
  menu_items.push_back( new menu_item( string("Go Have ID"), cmdGoHaveID ));
  menu_items.push_back( new menu_item( string("Go In Use"), cmdGoInUse ));
  menu_items.push_back( new menu_item( string("Go Finished"), cmdGoFinished ));
  menu_items.push_back( new menu_item( string("Go Ready"), cmdGoReady ));
  menu_items.push_back( new menu_item( string("Bad ID"), cmdBadID ));
  menu_items.push_back( new menu_item( string("Auto Upload"), cmdAutoUpload ));
  menu_items.push_back( new menu_item( string("Up List"), cmdUpList ));
  menu_items.push_back( new menu_item( string("Up StatSec"), cmdUpStatusSec ));
  menu_items.push_back( new menu_item( string("Up ListSec"), cmdUpListSec ));
  menu_items.push_back( new menu_item( string("ID Digits"), cmdIDDigits ));
  menu_items.push_back( new menu_item( string("Set Time"), cmdSetTime ));
  menu_items.push_back( new menu_item( string("Set Date"), cmdSetDate ));
  menu_items.push_back( new menu_item( string("Set Timeou"), cmdSetTimeout ));
  menu_items.push_back( new menu_item( string("Set T-Work"), 0x20 ));
  menu_items.push_back( new menu_item( string("Set Horizo"), cmdSetHorizontal ));
  menu_items.push_back( new menu_item( string("Set Vert"), cmdSetVertical ));
  menu_items.push_back( new menu_item( string("Set Calor"), cmdSetCalories ));
  menu_items.push_back( new menu_item( string("Set Program"), cmdSetProgram ));
  menu_items.push_back( new menu_item( string("Set Speed"), cmdSetSpeed ));
  menu_items.push_back( new menu_item( string("Set Grade"), cmdSetGrade ));
  menu_items.push_back( new menu_item( string("Set Gear"), cmdSetGear ));
  menu_items.push_back( new menu_item( string("Set UsrInfo"), cmdSetUserInfo ));
  menu_items.push_back( new menu_item( string("Set Torque"), cmdSetTorque ));
  menu_items.push_back( new menu_item( string("Set Level"), cmdSetLevel ));
  menu_items.push_back( new menu_item( string("Set Targ HR"), cmdSetTargetHR ));
  menu_items.push_back( new menu_item( string("Set Goal"), cmdSetGoal ));
  menu_items.push_back( new menu_item( string("Set METS"), cmdSetMETS ));
  menu_items.push_back( new menu_item( string("Set Power"), cmdSetPower ));
  menu_items.push_back( new menu_item( string("Set HR Zone"), cmdSetHRZone ));
  menu_items.push_back( new menu_item( string("Set HRMax"), cmdSetHRMax ));
  menu_items.push_back( new menu_item( string("Set Chn Rng"), cmdSetChannelRange));
  menu_items.push_back( new menu_item( string("Set Vol Rng"), cmdSetVolumeRange ));
  menu_items.push_back( new menu_item( string("Set Aud Mte"), cmdSetAudioMute ));
  menu_items.push_back( new menu_item( string("Set Aud Chn"), cmdSetAudioChannel ));
  menu_items.push_back( new menu_item( string("Set Aud Vol"), cmdSetAudioVolume ));
  menu_items.push_back( new menu_item( string("Get Grade"), qryGetGrade ));
  menu_items.push_back( new menu_item( string("Get Ser. #"), qryGetSerial )); // serial number


  menu_items.push_back( new menu_item( string("Get Caps" ), qryGetCaps ));
  menu_items.push_back( new menu_item( string("Get Ver."), qryGetVersion ));
  menu_items.push_back( new menu_item( string("Get ID"), qryGetID ));
  menu_items.push_back( new menu_item( string("Get Units" ), qryGetUnits ));
  menu_items.push_back( new menu_item( string("Get Ser. #"), qryGetSerial )); // serial number

  
  menu_items.push_back( new menu_item( string("Get List"), qryGetList ));
  menu_items.push_back( new menu_item( string("Get Util"), qryGetUtilization ));
  menu_items.push_back( new menu_item( string("Get Motor"), qryGetMotorCurrent ));
  menu_items.push_back( new menu_item( string("Get Odom"), qryGetOdometer ));
  menu_items.push_back( new menu_item( string("Get Error"), qryGetErrorCode ));
  menu_items.push_back( new menu_item( string("Get Service"), qryGetServiceCode ));
  menu_items.push_back( new menu_item( string("Get Prog"), qryGetProgram ));
  menu_items.push_back( new menu_item( string("Get Speed"), qryGetSpeed ));
  menu_items.push_back( new menu_item( string("Get Gear"), qryGetGear ));
  menu_items.push_back( new menu_item( string("Get UserInf"), qryGetUserInfo));
  menu_items.push_back( new menu_item( string("Get Torq"), qryGetTorque));
  menu_items.push_back( new menu_item( string("Get Power"), qryGetPower));
  

  
  
  fprintf( stderr, "(1) Interactive Mode\t\t(2) Automatic Mode\n" );
  fprintf( stderr, "#>" );
  scanf( "%d", &h );

  if( h==2 )
    {
      fprintf(stderr, "qryGetGrade\n" );
      fprintf( stderr, "#>" );

      scanf("%d", &h );
      cmd(qryGetGrade);
      
      // Ready -> Idle -> HaveID -> InUse
      fprintf( stderr,"cmdReset\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdReset);

      
      fprintf( stderr, "cmdGoReady\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdGoReady);
        
      fprintf( stderr, "cmdGoIdle\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdGoIdle);
      

      while(1)
	{
	  fprintf( stderr, "qryGetGrade\n" );
	  fprintf( stderr, "#>" );
	  scanf("%d",&h);
	  cmd(qryGetGrade);
	  
	  printf( "cmdSetGrade (-1 to break loop)\n" );
	  fprintf( stderr, "#>" );
	  scanf("%d",&h);
	  cmd(cmdSetGrade, 0xFB, 0x00, untPercentGrade);
	  if( h == -1 ) break;
	}

      fprintf( stderr, "cmdAutoUpload\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdAutoUpload, flgAutoStatus );

      fprintf( stderr, "cmdGoHaveID\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdGoHaveID);

      fprintf(stderr,"cmdGoInUse\n" );	 
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdGoInUse);

      fprintf(stderr, "cmdSetSpeed(2.5)\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdSetSpeed, 25, 0x00, 0x11);

      fprintf( stderr, "cmdSetUserInfo [215lbs, 50yo, Male]\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdSetUserInfo, 215, 0, untPounds, 50, untMale );
  
      fprintf( stderr,"cmdSetGrade [50, PercentGrade]\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdSetGrade, 50, 0, untPercentGrade);
  
      fprintf( stderr,"cmdSetGear [5]\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdSetGear, 5 );

  
      fprintf( stderr,"cmdSetTorque [5, 90]\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdSetTorque, 5, 0, 90);
  
      fprintf( stderr,"cmdSetLevel [5]\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdSetLevel, 5);

      fprintf(stderr,"cmdGoFinished\n" );
      fprintf( stderr, "#>" );
      scanf("%d",&h);
      cmd(cmdGoFinished);
    }
  else
    {
      int keep_going = 1;
      while( keep_going )
	{
	  // display menu
	  showMenu();
	  
	  fprintf( stderr, "#>" );
	  scanf("%d",&h);
	  if( h == -1 )
	    {
	      keep_going = 0;
	    }
	  else if( h >= 0 && h < menu_items.size() )
	    {
	      switch( menu_items[h]->getNumber()  )
		{
		case cmdSetLevel:
		  cmd(cmdSetLevel, 50 );
		  break;
		case cmdSetGear:
		  cmd(cmdSetGear, 50 );
		  break;
		case cmdSetGrade:
		  cmd(cmdSetGrade, 50, 0, untPercentGrade);
		  break;
		case cmdSetSpeed:
		  cmd( cmdSetSpeed, 25, 0x00, 0x11);
		  break;
		default:
		  cmd(menu_items[h]->getNumber());
		}
	    }
	  else
	    {
	      fprintf( stderr, "invalid selection error\n" );

	    }
	}

    }

  
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



/*

  (0x0A) RX: 0x7B 0x79 0x3D
  (0xA8) RX: 0xFE 0xA8 0xF5
  (0xA8) RX: 0xFF 0xFE 0x28
  (0xA8) RX: 0xC0 0xFA 0xB9
  (0xA8) RX: 0xFF 0xFE 0xA8
  (0xA8) RX: 0xEA 0xFF 0xFE
  (0xA8) RX: 0x28 0xC0 0xFF


  7B 79 3D 
  FE A8 F5 FF 
  FE 28 C0 FA B9 FF 
  FE A8 EA FF 
  FE 28 C0 FF



  0111 1011 0111 1001 0011 1101
  1111 1110 1010 1000 1111 0101 1111 1111
  1111 1110 0010 1000 1100 0000 1111 1010 1011 1001 1111 1111
  1111 1110 1010 1000 1110 1010 1111 1111
  1111 1110 0010 1000 1010 0000 1111 1111



*/
