#ifndef CROBOT_HARDWARE_HPP
#define CROBOT_HARDWARE_HPP

#include "crobot_hardware/serial_comm.hpp"
#include "stdint.h"

/**
 * @brief Maximum packet size acceptable for this application
 */
#define CROBOT_CONTROL_PKT_MAX_NUM_BYTES_IN_FRAME 32

/**
 * @brief Number of bytes consumer by packet header
 */
#define CROBOT_CONTROL_PKT_NUM_HEADER_BYTES 2

/**
 * @brief Number of bytes used for opcpde
 */
#define CROBOT_CONTROL_PKT_NUM_OPCODE_BYTES  1

/**
 * @brief Number of bytes used for motor ID
 */
#define CROBOT_CONTROL_PKT_NUM_ID_BYTES 1

/**
 * @brief Number of bytes used for packet length field
 */
#define CROBOT_CONTROL_PKT_NUM_LEN_BYTES 1

/**
 * @brief Number of bytes used for CRC
 */
#define CROBOT_CONTROL_PKT_NUM_CRC_BYTES 2

/**
 * @brief Maximum number of bytes in the parameters array
 */
#define CROBOT_CONTROL_PKT_MAX_NUM_PARAM_BYTES    CROBOT_CONTROL_PKT_MAX_NUM_BYTES_IN_FRAME - \
                                                    CROBOT_CONTROL_PKT_NUM_HEADER_BYTES - \
                                                    CROBOT_CONTROL_PKT_NUM_OPCODE_BYTES - \
                                                    CROBOT_CONTROL_PKT_NUM_ID_BYTES - \
                                                    CROBOT_CONTROL_PKT_NUM_LEN_BYTES

#define CROBOT_CONTROL_MAKE_16B_WORD(a, b) ((uint16_t)a << 8) | ((uint16_t)b)
#define CROBOT_CONTROL_GET_UPPER_16B(a) (uint8_t)(((uint16_t)a >> 8) & 0xFF)
#define CROBOT_CONTROL_GET_LOWER_16B(a) (uint8_t)((uint16_t)a & 0xFF)


typedef enum {
  ECHO,
  READ_STATUS,
  READ_ANGLE,
  WRITE_ANGLE,
  WRITE_PID,
} crobot_opcode_t;


typedef enum {
  COMM_SUCCESS,
  COMM_FAIL,
  COMM_INSTRUCTION_ERROR,
  COMM_CRC_ERROR,
  COMM_ID_ERROR,
  COMM_PARAM_ERROR
} crobot_comm_result_t;


class CrobotHardwareInterface
{
public:

    CrobotHardwareInterface(std::string portName);
    ~CrobotHardwareInterface();


    typedef struct {
        uint8_t motorId;
        uint8_t opcode;
        uint8_t numParams;
        uint8_t pkt_params[CROBOT_CONTROL_PKT_MAX_NUM_PARAM_BYTES];
    } CrobotHardwareResponse;

    void BuildPacket(uint8_t id, uint8_t opcode, uint8_t* params, std::size_t num_params);

    void SendTxRx();

    void SendTx();
    
private:

    uint16_t CRC16(uint16_t crc_accum, uint8_t *data, uint16_t data_blk_size);

    SerialComm ser;

    typedef struct
    {
        union
        {
            uint8_t data_packet[CROBOT_CONTROL_PKT_MAX_NUM_BYTES_IN_FRAME];

            struct {
                uint8_t header[CROBOT_CONTROL_PKT_NUM_HEADER_BYTES];
                uint8_t len;
                uint8_t motorId;
                uint8_t opcode;
                uint8_t pkt_params[CROBOT_CONTROL_PKT_MAX_NUM_PARAM_BYTES];
            } data_packet_s;
        } data_packet_u;
    } CrobotControl_data_packet_s;

    CrobotControl_data_packet_s rx;
    CrobotControl_data_packet_s tx;
};

#endif