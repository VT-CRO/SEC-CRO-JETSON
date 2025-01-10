#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <string>

class SerialComm
{

public:

  SerialComm();

  ~SerialComm();

  void connect(const std::string &serial_device, int32_t timeout_ms);

  void disconnect();

  bool connected() const;

  void writeBytes(const uint8_t* bytes, int numBytes);

  int readBytes(uint8_t * buff, int numBytes);

private:
  int fd;
  int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP