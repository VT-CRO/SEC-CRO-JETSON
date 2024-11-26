#include <termios.h>
#include <unistd.h>
#include <sys/fcntl.h>

#include "crobot_hardware/serial_comm.hpp"

SerialComm::SerialComm() : fd(-1) {

}

SerialComm::~SerialComm() {
    disconnect();
}

void SerialComm::connect(const std::string &serial_device, int32_t)
{
    const char* portName = serial_device.c_str();
    
    struct termios port_options;

    tcgetattr(fd, &port_options);

    fd = open(portName, O_RDWR | O_NOCTTY);

    tcflush(fd, TCIFLUSH);
    tcflush(fd, TCIOFLUSH);

    if (fd == -1)
    {
        return;
    }

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size
    port_options.c_cflag |=  CS8;               // Set the data bits = 8
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;

    port_options.c_cc[VMIN]  = VMIN;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly

    cfsetispeed(&port_options, 115200);    // Set Read  Speed
    cfsetospeed(&port_options, 115200);    // Set Write Speed

    int att = tcsetattr(fd, TCSANOW, &port_options);

    if (att != 0)
    {
        return;
    } else {
        return;
    }

    tcflush(fd, TCIFLUSH);
    tcflush(fd, TCIOFLUSH);
}

void SerialComm::disconnect()
{
    close(fd);
}

bool SerialComm::connected() const
{
    return fd != -1;
}


void SerialComm::writeBytes(const uint8_t* bytes, int numBytes) {
    if (connected()) {
        write(fd, bytes, numBytes);
    }
}

int SerialComm::readBytes(uint8_t * buff, int numBytes) {
    if (connected()) {
        return read(fd, buff, numBytes);
    } else {
        return -1;
    }
}