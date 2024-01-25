#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define SERIAL_PORT "/dev/tty.usbserial-A9XNR88X"  // Black Cable

int main() {
    int serialPort;
    struct termios serialConfig;

    // Open the serial port
    serialPort = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        perror("Error opening serial port");
        return 1;
    }

    // Configure the serial port
    tcgetattr(serialPort, &serialConfig);
    cfsetispeed(&serialConfig, B9600);  // Set baud rate
    cfsetospeed(&serialConfig, B9600);
    serialConfig.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and set local mode
    serialConfig.c_cflag &= ~PARENB;  // No parity
    serialConfig.c_cflag &= ~CSTOPB;  // 1 stop bit
    serialConfig.c_cflag &= ~CSIZE;
    serialConfig.c_cflag |= CS8;  // 8 data bits
    tcsetattr(serialPort, TCSANOW, &serialConfig);

    // Example command to send to the CSAFE Fitness Machine
    char command[] = {0x05, 0x20, 0x01};  // Example command bytes

    // Write the command to the serial port
    if (write(serialPort, command, sizeof(command)) != sizeof(command)) {
        perror("Error writing to serial port");
        close(serialPort);
        return 1;
    }

    // Read response from the serial port
    char response[255];
    usleep(500000);
    int bytesRead = read(serialPort, response, sizeof(response));
    if (bytesRead == -1) {
        perror("Error reading from serial port");
    } else {
        printf("Received %d bytes: ", bytesRead);
        for (int i = 0; i < bytesRead; i++) {
            printf("%02X ", (unsigned char)response[i]);
        }
        printf("\n");
    }

    // Close the serial port
    close(serialPort);

    return 0;
}
