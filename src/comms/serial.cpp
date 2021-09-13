#include "setup_board.h"

#include "serial.h"

SerialLink::SerialLink() {
}

SerialLink::~SerialLink() {
}

void SerialLink::checksum( uint8_t id, uint8_t len_lo, uint8_t len_hi,
                           uint8_t *buf, uint16_t buf_size,
                           uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += id;
    c1 += c0;

    c0 += len_lo;
    c1 += c0;

    c0 += len_hi;
    c1 += c0;

    for ( uint16_t i = 0; i < buf_size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}

bool SerialLink::open( int baud, AP_HAL::UARTDriver *port ) {
    console->printf("Opening comms port @ %d\n", baud);
    _port = port;
    // uint8_t opts = _port->get_options();
    // opts |= AP_HAL::UARTDriver::OPTION_NODMA_RX;
    // opts |= AP_HAL::UARTDriver::OPTION_NODMA_TX;
    // _port->set_options(opts);
    _port->begin(baud);
    // _port->begin(baud, 2048, 2048);
    return true;
}

bool SerialLink::update() {
    // 0 = looking for SOM0
    // 1 = looking for SOM1
    // 2 = looking for packet id
    // 3 = looking for packet len (note 2 bytes)
    // 4 = looking for packet data
    // 5 = looking for checksum_lo
    // 6 = looking for checksum_l=hi
    uint8_t input;
    bool new_data = false;
    // console->print("start read_commands(): "); console->println(state);

    if ( state == 0 ) {
        counter = 0;
        while ( _port->available() >= 1 ) {
            // scan for start of message
            input = _port->read();
            // console->println(input);
            if ( input == START_OF_MSG0 ) {
                // console->println("start of msg0");
                state = 1;
                break;
            }
        }
    }
    if ( state == 1 ) {
        if ( _port->available() >= 1 ) {
            input = _port->read();
            if ( input == START_OF_MSG1 ) {
                // console->println("start of msg1");
                state = 2;
            } 
            else if ( input == START_OF_MSG0 ) {
                // no change
            } else {
                // oops
                state = 0;
            }
        }
    }
    if ( state == 2 ) {
        if ( _port->available() >= 1 ) {
            pkt_id = _port->read();
            // console->print("id="); console->println(message_id);
            state = 3;
        }
    }
    if ( state == 3 ) {
        if ( _port->available() >= 2 ) {
            pkt_len_lo = _port->read();
            pkt_len_hi = _port->read();
            pkt_len = pkt_len_hi << 8 | pkt_len_lo;
            // console->printf("size=%d\n", pkt_len);
            if ( pkt_len > 4096 ) {
                console->printf("nonsense packet size, skipping.\n");
                // ignore nonsensical sizes
                state = 0;
            }  else {
                state = 4;
            }
        }
    }
    if ( state == 4 ) {
        if ( pkt_len > payload_len ) {
            // reallocate buffer if new payload is larger than
            // anything we've read so far.
            void *newbuf = hal.util->std_realloc(payload, pkt_len);
            if ( newbuf == nullptr ) {
                console->printf("payload heap allocation failed.\n");
                state = 0;
                return false;
            } else {
                payload = (uint8_t *)newbuf;
                payload_len = pkt_len;
            }
        }
        while ( _port->available() >= 1 && counter < pkt_len ) {
            payload[counter++] = _port->read();
            // console->println(buf[i], DEC);
        }
        if ( counter >= pkt_len ) {
            state = 5;
        }
    }
    if ( state == 5 ) {
        if ( _port->available() >= 1 ) {
            cksum_lo = _port->read();
            state = 6;
        }
    }
    if ( state == 6 ) {
        if ( _port->available() >= 1 ) {
            cksum_hi = _port->read();
            uint8_t cksum0, cksum1;
            checksum( pkt_id, pkt_len_lo, pkt_len_hi, payload, pkt_len,
                      &cksum0, &cksum1 );
            if ( cksum_lo == cksum0 && cksum_hi == cksum1 ) {
                // console->println("passed check sum!");
                // console->print("pkt_id = "); console->println(pkt_id);
                // console->print("size="); console->println(pkt_len);
                // parse_message_bin( pkt_id, payload, pkt_len );
                new_data = true;
                state = 0;
            } else {
                console->printf("failed check sum id: %d len: %d\n",
                                pkt_id, pkt_len);
                // check sum failure
                state = 0;
            }
        }
    }

    return new_data;
}

int SerialLink::bytes_available() {
    return _port->available();
}

uint16_t SerialLink::write_packet(uint8_t packet_id, uint8_t *buf, uint16_t buf_size) {
    // static int min_space = _port->txspace();
    // if ( _port->txspace() > 0 and _port->txspace() < min_space ) {
    //     console->printf("tx space low water mark: %d\n", min_space);
    //     min_space = _port->txspace();
    // }

    if ( ! _port->is_initialized() ) {
        return 0;
    }
    if ( _port->txspace() < buf_size + 7U ) {
        // console->printf("tx space: %ld\n", _port->txspace());
        return 0;
    }
    
    // start of message sync (2) bytes
    _port->write(START_OF_MSG0);
    _port->write(START_OF_MSG1);

    // packet id (1 byte)
    _port->write(packet_id);
    
    // packet length (2 bytes)
    uint8_t len_lo = buf_size & 0xFF;
    uint8_t len_hi = buf_size >> 8;
    _port->write(len_lo);
    _port->write(len_hi);

    // write payload
    _port->write( buf, buf_size );
    
    // check sum (2 bytes)
    uint8_t cksum0, cksum1;
    checksum( packet_id, len_lo, len_hi, buf, buf_size, &cksum0, &cksum1 );
    _port->write(cksum0);
    _port->write(cksum1);

    return buf_size + 7U;
}

bool SerialLink::close() {
    _port->end();
    return true;
}
