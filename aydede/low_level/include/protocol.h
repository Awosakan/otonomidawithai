#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sync Bytes
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

// Message IDs
#define MSG_HEARTBEAT       0x01
#define MSG_PHONE_COMMANDS  0x02
#define MSG_STM32_TELEMETRY 0x03
#define MSG_PID_TUNING      0x04

// System Modes
#define MODE_IDLE       0x00
#define MODE_AUTO       0x01
#define MODE_MANUAL     0x02
#define MODE_FAILSAFE   0x03
#define MODE_EMERGENCY  0x04

#pragma pack(push, 1)

// Heartbeat Struct (2 bytes)
typedef struct {
    uint8_t status; // 1 = OK, 2 = Error
    uint8_t mode;   // Current system mode (0: Idle, 1: Auto)
} Heartbeat_t;

// Phone Commands Struct (9 bytes)
typedef struct {
    uint8_t control_mode;  // 0 = Differential thrust, 1 = Speed & Heading
    float target_speed;    // m/s or percentage
    float target_heading;  // degrees (0-360)
} PhoneCommands_t;

// STM32 Telemetry Struct (54 bytes)
typedef struct {
    double lat;
    double lon;
    float sog; // Speed over ground
    float cog; // Course over ground
    uint8_t gps_lock;
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float battery;
    uint8_t mode;
} Telemetry_t;

// PID Tuning Struct (12 bytes)
typedef struct {
    float kp;
    float ki;
    float kd;
} PIDTuning_t;

#pragma pack(pop)

// CRC16 Modbus Calculation in C
static inline uint16_t calculate_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Parser State Machine
typedef enum {
    STATE_WAIT_SYNC1 = 0,
    STATE_WAIT_SYNC2,
    STATE_WAIT_MSG_ID,
    STATE_WAIT_LEN,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CRC_LSB,
    STATE_WAIT_CRC_MSB
} ParserState_t;

typedef struct {
    ParserState_t state;
    uint8_t msg_id;
    uint8_t payload_len;
    uint8_t payload_idx;
    uint8_t payload[255];
    uint8_t header_buf[4]; // Stores Sync1, Sync2, MsgID, Len for CRC
    uint16_t received_crc;
} ProtocolParser_t;

static inline void protocol_parser_init(ProtocolParser_t *parser) {
    parser->state = STATE_WAIT_SYNC1;
    parser->payload_idx = 0;
}

// Feeds a single byte into the parser state machine
// Returns 1 if a valid packet has been parsed, 0 otherwise
static inline uint8_t protocol_parser_feed(ProtocolParser_t *parser, uint8_t b) {
    switch (parser->state) {
        case STATE_WAIT_SYNC1:
            if (b == SYNC_BYTE_1) {
                parser->header_buf[0] = b;
                parser->state = STATE_WAIT_SYNC2;
            }
            break;
            
        case STATE_WAIT_SYNC2:
            if (b == SYNC_BYTE_2) {
                parser->header_buf[1] = b;
                parser->state = STATE_WAIT_MSG_ID;
            } else {
                parser->state = STATE_WAIT_SYNC1;
            }
            break;
            
        case STATE_WAIT_MSG_ID:
            parser->msg_id = b;
            parser->header_buf[2] = b;
            parser->state = STATE_WAIT_LEN;
            break;
            
        case STATE_WAIT_LEN:
            parser->payload_len = b;
            parser->header_buf[3] = b;
            parser->payload_idx = 0;
            if (parser->payload_len > 0) {
                parser->state = STATE_WAIT_PAYLOAD;
            } else {
                parser->state = STATE_WAIT_CRC_LSB;
            }
            break;
            
        case STATE_WAIT_PAYLOAD:
            parser->payload[parser->payload_idx++] = b;
            if (parser->payload_idx >= parser->payload_len) {
                parser->state = STATE_WAIT_CRC_LSB;
            }
            break;
            
        case STATE_WAIT_CRC_LSB:
            parser->received_crc = b;
            parser->state = STATE_WAIT_CRC_MSB;
            break;
            
        case STATE_WAIT_CRC_MSB:
            parser->received_crc |= ((uint16_t)b << 8);
            
            // Calculate expected CRC: header (4 bytes) + payload
            uint8_t crc_temp[300];
            memcpy(crc_temp, parser->header_buf, 4);
            if (parser->payload_len > 0) {
                memcpy(crc_temp + 4, parser->payload, parser->payload_len);
            }
            
            uint16_t expected_crc = calculate_crc16(crc_temp, 4 + parser->payload_len);
            
            // Reset parser state for next packet
            parser->state = STATE_WAIT_SYNC1;
            
            if (parser->received_crc == expected_crc) {
                return 1; // Valid packet parsed
            }
            break;
    }
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
