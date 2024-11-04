#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>

class SerialComm
{

public:

  SerialComm() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) {
    return;
  }

  void disconnect() {
    return;
  }

  bool connected() const {
    return false;
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false) {
    return "";
  }


  void send_empty_msg() {
    return;
  }

  void read_encoder_values(int &val_1, int &val_2) {
    return;
  }

  void set_motor_values(int val_1, int val_2) {
    return;
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o) {
    return;
  }

private:
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP