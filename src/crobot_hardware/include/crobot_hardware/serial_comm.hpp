#ifndef SERIAL_COMM_HPP_
#define SERIAL_COMM_HPP_

#include <string>

namespace crobot_hardware
{
    class SerialComm
    {
        public:
            SerialComm();
            ~SerialComm();

            bool connect(const std::string &device_path, int baud_rate, int timeout_ms = 0);
            void disconnect();

            bool isConnected() const;

            int writeBytes(const char *bytes, int numBytes);
            int readBytes(char *buff, int numBytes);
            std::string readLine();

            void clearBuffers();

        private:
            int fd_;
            int timeout_ms_;
            static constexpr int INVALID_FD = -1;
            std::string read_buffer_;
    };
}

#endif // SERIAL_COMM_HPP_