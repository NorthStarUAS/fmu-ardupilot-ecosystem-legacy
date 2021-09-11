#include "comms_relay.h"

comms_relay_t::comms_relay_t() {
}

comms_relay_t::~comms_relay_t() {
}

int comms_relay_t::forward_packet( dest_enum dest,
                                   uint8_t id, uint8_t *buf, uint16_t buf_size )
{
    if ( dest == dest_enum::host_dest and host_link != nullptr ) {
        return host_link->write_packet(id, buf, buf_size);
    } else if ( dest == dest_enum::gcs_dest and gcs_link != nullptr ) {
        return gcs_link->write_packet(id, buf, buf_size);
    }
    return 0;
}

comms_relay_t comms_relay;
