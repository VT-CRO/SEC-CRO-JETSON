#include "crobot_hardware/serial_comm.hpp"

#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include "rclcpp/rclcpp.hpp"

namespace crobot_hardware
{
    SerialComm::SerialComm() : fd_(INVALID_FD), timeout_ms_(0)
    {
    }

    SerialComm::~SerialComm()
    {
        disconnect();
    }

    bool SerialComm::connect(const std::string &device_path, int baud_rate, int timeout_ms)
    {
        RCLCPP_INFO(rclcpp::get_logger("SerialComm"), 
                    "Attempting to open %s at %d baud",
                    device_path.c_str(), baud_rate);

        // O_RDWR: Read and write access
        // O_NOCTTY: Do not make this terminal the controlling terminal
        // O_NONBLOCK: Non-blocking mode
        fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        timeout_ms_ = timeout_ms;

        // Get terminal settings
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"),
                         "Failed to read terminal settings: %s", strerror(errno));
            close(fd_);
            fd_ = INVALID_FD;
            return false;
        }

        tcflush(fd_, TCIOFLUSH);

        // TODO: Confirm serial port flags

        tty.c_cflag &= ~PARENB;         // No parity
        tty.c_cflag &= ~CSTOPB;         // 1 stop bit
        tty.c_cflag &= ~CSIZE;          // Clear data size bits
        tty.c_cflag |= CS8;             // 8 data bits
        tty.c_cflag &= ~CRTSCTS;        // Disable hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

        // turn off s/w flow ctrl
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         

         // disable special input processing
        tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        tty.c_oflag &= ~OPOST;          // disable output processing

        tty.c_lflag = 0;                // non-canonical mode

        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 0;            // no read timeout

        speed_t speed;
        switch (baud_rate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default:
                RCLCPP_WARN(rclcpp::get_logger("SerialComm"),
                             "Unsupported baud rate: %d, using 115200", baud_rate);
                speed = B115200;
        }

        cfsetispeed(&tty, speed);   // Set input baud rate
        cfsetospeed(&tty, speed);   // Set output baud rate

        // Apply terminal settings
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"),
                         "Failed to set terminal attributes: %s", strerror(errno));
            close(fd_);
            fd_ = INVALID_FD;
            return false;
        }

        tcflush(fd_, TCIOFLUSH);

        RCLCPP_INFO(rclcpp::get_logger("SerialComm"),
                    "Successfully connected to %s at %d baud",
                    device_path.c_str(), baud_rate);

        return true;
    }

    void SerialComm::disconnect()
    {
        if (isConnected()) {
            close(fd_);
            fd_ = INVALID_FD;
            RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Disconnected serial port");
        }
    }

    bool SerialComm::isConnected() const
    {
        return fd_ != INVALID_FD;
    }

    int SerialComm::writeBytes(const char *bytes, int numBytes)
    {
        if (!isConnected()) {
            RCLCPP_WARN(rclcpp::get_logger("SerialComm"),
                         "Attempted to write but serial port is not connected");

            return -1;
        }

        if (numBytes <= 0 || bytes == nullptr) {
            return 0;
        }

        int bytesWritten = write(fd_, bytes, numBytes);

        if (bytesWritten < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"),
                         "Failed to write to serial port: %s", strerror(errno));
        }

        return bytesWritten;
    }

    int SerialComm::readBytes(char *buff, int numBytes)
    {
        if (!isConnected()) {
            RCLCPP_WARN(rclcpp::get_logger("SerialComm"),
                         "Attempted to read but serial port is not connected");

            return -1;
        }

        if (numBytes <= 0 || buff == nullptr) {
            return 0;
        }

        // TODO: Determine if read and open should be non-blocking or blocking with timeout
        
        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;

        int pollResult = poll(&pfd, 1, timeout_ms_);

        if (pollResult < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"),
                         "Poll error on serial port: %s", strerror(errno));
            return -1;
        } else if (pollResult == 0) {
            // Timeout
            return 0;
        }

        int bytesRead = read(fd_, buff, numBytes);

        if (bytesRead < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"),
                         "Failed to read from serial port: %s", strerror(errno));
        }

        return bytesRead;
    }

    void SerialComm::clearBuffers()
    {
        if (isConnected()) {
            tcflush(fd_, TCIOFLUSH);
        }
    }
}