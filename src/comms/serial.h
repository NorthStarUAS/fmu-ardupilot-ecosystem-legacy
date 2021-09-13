#pragma once

#include <AP_HAL/AP_HAL.h>

class SerialLink {

private:

    // port
    AP_HAL::UARTDriver *_port;

    // parser
    int state = 0;
    int counter = 0;
    uint8_t cksum_lo = 0, cksum_hi = 0;

    static const uint8_t START_OF_MSG0 = 147;
    static const uint8_t START_OF_MSG1 = 224;

    void checksum( uint8_t id, uint8_t len_lo, uint8_t len_hi,
                   uint8_t *buf, uint16_t buf_size,
                   uint8_t *cksum0, uint8_t *cksum1 );

public:

    uint8_t pkt_id = 0;
    uint8_t pkt_len_lo = 0;
    uint8_t pkt_len_hi = 0;
    uint16_t pkt_len = 0;
    uint8_t *payload = nullptr;
    uint16_t payload_len = 0;

    uint32_t parse_errors = 0;

    SerialLink();
    ~SerialLink();

    bool open( int baud, AP_HAL::UARTDriver *port );
    bool update();
    int bytes_available();
    uint16_t write_packet(uint8_t packet_id, uint8_t *buf, uint16_t buf_size);
    bool close();
    size_t txspace() { return _port->txspace(); }
};
