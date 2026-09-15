/* SPDX-License-Identifier: GPL-3.0-only */
#include "j2534.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "board.h"
#include "isotp.h"
#include "j2534.pb.h"
#include "j2534_scheduler.h"
#include "kline_codec.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define CHANNELS (J2534_PHYSICAL_MAX + J2534_LOGICAL_MAX)
#define RX_BYTES 8192u
#define MSG_HEADER 28u
#define TX_BYTES (J2534_DATA_MAX + MSG_HEADER)

_Static_assert(J2534_PERIODIC_MAX == J2534_SCHEDULE_SLOTS &&
                   J2534_REPEAT_MAX == J2534_SCHEDULE_SLOTS,
               "scheduler quotas");

enum {
    IOCTL_GET_CONFIG = 1,
    IOCTL_SET_CONFIG = 2,
    IOCTL_READ_VBATT = 3,
    IOCTL_CLEAR_FILTERS = 10,
    IOCTL_READ_PROG_VOLTAGE = 14,
};

enum {
    CONFIG_ISOTP_BLOCK_SIZE = 0x1e,
    CONFIG_ISOTP_STMIN = 0x1f,
    CONFIG_ISOTP_WAIT_LIMIT = 0x25,
    CONFIG_ISOTP_PAD_VALUE = 0x2b,
    CONFIG_N_AS_MAX = 0x2c,
    CONFIG_N_AR_MAX = 0x2d,
    CONFIG_N_BS_MAX = 0x2e,
    CONFIG_N_CR_MAX = 0x2f,
};

enum {
    FILTER_PASS = 1,
    FILTER_BLOCK = 2,
    LOGICAL_FULL_DUPLEX = 1,
    SELECT_READABLE = 1,
};

typedef struct {
    uint32_t id;
    uint32_t type;
    uint32_t flags;
    uint8_t len;
    uint8_t mask[12];
    uint8_t pattern[12];
} filter_t;

typedef enum {
    TX_NONE,
    TX_MESSAGE,
    TX_ISOTP,
    TX_FLOW_CONTROL,
    TX_SCHEDULED,
} tx_kind_t;

typedef struct {
    tx_kind_t kind;
    uint32_t channel;
    uint32_t job;
    uint32_t handle;
    bool discard;
    bool confirmed;
    uint32_t rx_generation;
    struct can_frame frame;
} pending_tx_t;

typedef struct {
    uint32_t id;
    uint32_t parent;
    uint32_t protocol;
    uint32_t flags;
    uint32_t rate;
    uint32_t echo;
    vif_bus_t bus;

    uint8_t *rx;
    uint8_t *tx;
    uint8_t *assembly;
    size_t rx_head;
    size_t rx_used;
    size_t tx_len;
    bool overflow;
    bool tx_active;
    bool link_down;

    filter_t filters[J2534_FILTER_MAX];
    j2534_scheduler_t scheduler;
    pending_tx_t pending;
    uint32_t rx_sequence;
    unsigned tx_cursor;
    bool rx_drained;
    bool flow_control_pending;
    uint32_t rx_generation;
    struct can_frame flow_control;

    isotp_config_t config;
    isotp_rx_t iso_rx;
    isotp_tx_t iso_tx;
    uint32_t local_flags;
    uint32_t remote_flags;
    bool legacy_channel;
    uint32_t tx_timeout_a_us;
    uint8_t local[5];
    uint8_t remote[5];
} channel_t;

static channel_t channels[CHANNELS];
static uint32_t device;
static uint32_t next_id = 1;
static uint32_t last_error;
static int voltage_pin = -1;
static opendiag_Request request;
static opendiag_Response response;
/* Scratch is shared only by the link task, never placed on its 4 KiB stack. */
static uint8_t scratch[TX_BYTES];
_Static_assert(opendiag_Request_size <= J2534_RPC_MAX,
               "protobuf request buffer");

static uint32_t now_us(void) { return (uint32_t)esp_timer_get_time(); }

static uint32_t new_id(void) {
    if (!++next_id)
        ++next_id;
    return next_id;
}

static uint32_t be32(const uint8_t *data) {
    return (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 |
           (uint32_t)data[2] << 8 | data[3];
}

static void put_be32(uint8_t *data, uint32_t value) {
    for (unsigned i = 0; i < 4; i++)
        data[i] = (uint8_t)(value >> (24 - 8 * i));
}

static channel_t *find_channel(uint32_t id) {
    for (unsigned i = 0; i < CHANNELS; i++)
        if (id && channels[i].id == id)
            return &channels[i];

    return NULL;
}

static channel_t *physical_channel(channel_t *channel) {
    return channel->parent ? find_channel(channel->parent) : channel;
}

static bool channel_is_live(channel_t *channel) {
    return channel && physical_channel(channel) &&
           vif_bus_is_open(VIF_OWNER_LINK, channel->bus);
}

static uint32_t bus_error(int rc) {
    switch (rc) {
    case 0:
        return J2534_OK;
    case BUS_ERR_TIMEOUT:
        return J2534_TIMEOUT;
    case BUS_ERR_NO_SPACE:
        return J2534_FULL;
    case BUS_ERR_TOO_LONG:
    case BUS_ERR_BAD_ARG:
        return J2534_MSG;
    case BUS_ERR_UNSUPPORTED:
        return J2534_NOT_SUPPORTED;
    case BUS_ERR_INIT:
        return J2534_INIT;
    case BUS_ERR_NOT_READY:
        return J2534_CHANNEL;
    default:
        return J2534_FAILED;
    }
}

static void cancel_channel_tx(channel_t *channel);

static void release_channel(channel_t *channel) {
    free(channel->rx);
    free(channel->tx);
    free(channel->assembly);
    memset(channel, 0, sizeof(*channel));
}

static uint32_t disconnect_channel(channel_t *channel) {
    cancel_channel_tx(channel);
    if (!channel->parent) {
        if (vif_bus_close(VIF_OWNER_LINK, channel->bus) != ESP_OK)
            return J2534_FAILED;

        for (unsigned i = 0; i < CHANNELS; i++)
            if (channels[i].parent == channel->id)
                release_channel(&channels[i]);
    }
    release_channel(channel);
    return J2534_OK;
}

static channel_t *allocate_channel(bool logical) {
    unsigned first = logical ? J2534_PHYSICAL_MAX : 0;
    unsigned end = logical ? CHANNELS : J2534_PHYSICAL_MAX;

    for (unsigned i = first; i < end; i++) {
        channel_t *channel = &channels[i];

        if (channel->id)
            continue;

        channel->rx = malloc(RX_BYTES);
        channel->tx = malloc(TX_BYTES);
        if (logical)
            channel->assembly = malloc(ISOTP_MAX_PAYLOAD + 5);
        if (!channel->rx || !channel->tx || (logical && !channel->assembly)) {
            release_channel(channel);
            return NULL;
        }
        channel->id = new_id();
        return channel;
    }
    return NULL;
}

static void ring_copy(channel_t *channel, size_t offset, uint8_t *data,
                      size_t len, bool write) {
    size_t pos = (channel->rx_head + offset) % RX_BYTES;
    for (size_t i = 0; i < len; i++) {
        if (write)
            channel->rx[pos] = data[i];
        else
            data[i] = channel->rx[pos];
        pos = (pos + 1) % RX_BYTES;
    }
}

static void enqueue_rx(channel_t *channel, uint32_t handle, uint32_t status,
                       uint32_t timestamp, const uint8_t *data, size_t len,
                       bool indication) {
    size_t total = MSG_HEADER + len;
    if (total > RX_BYTES - channel->rx_used) {
        channel->overflow = true;
        return;
    }
    uint8_t header[MSG_HEADER];

    j2534_put32(header, channel->protocol);
    j2534_put32(header + 4, handle);
    j2534_put32(header + 8,
                status | (channel->overflow ? J2534_RX_OVERFLOW : 0));
    j2534_put32(header + 12, 0);
    j2534_put32(header + 16, timestamp);
    j2534_put32(header + 20, len);
    j2534_put32(header + 24, indication ? 0 : len);
    ring_copy(channel, channel->rx_used, header, sizeof(header), true);
    ring_copy(channel, channel->rx_used + MSG_HEADER, (uint8_t *)data, len,
              true);
    channel->rx_used += total;
    channel->overflow = false;
}

static bool filter_accepts(channel_t *channel, const uint8_t *data, size_t len,
                           uint32_t flags) {
    bool pass = false;
    for (unsigned i = 0; i < J2534_FILTER_MAX; i++) {
        filter_t *filter = &channel->filters[i];

        if (!filter->id || filter->len > len ||
            ((filter->flags ^ flags) & J2534_29BIT))
            continue;

        bool match = true;

        for (unsigned j = 0; j < filter->len; j++)
            if ((data[j] & filter->mask[j]) !=
                (filter->pattern[j] & filter->mask[j]))
                match = false;
        if (!match)
            continue;

        if (filter->type == FILTER_BLOCK)
            return false;

        pass = true;
    }
    return pass;
}

static bool can_flags_match(channel_t *channel, uint32_t flags) {
    return (channel->flags & J2534_CAN_BOTH) ||
           !((channel->flags ^ flags) & J2534_29BIT);
}

static bool can_id_valid(const uint8_t *data, uint32_t flags) {
    return be32(data) <= ((flags & J2534_29BIT) ? CAN_EFF_MASK : CAN_SFF_MASK);
}

static uint32_t validate_message(channel_t *channel, uint32_t protocol,
                                 uint32_t flags, const uint8_t *data,
                                 size_t len, bool periodic) {
    if (protocol != channel->protocol)
        return J2534_MSG_PROTOCOL;
    if (!len || len > J2534_DATA_MAX)
        return J2534_MSG;

    if (channel->protocol == J2534_CAN || channel->parent) {
        uint32_t allowed =
            J2534_29BIT | (channel->parent ? J2534_ADDR | J2534_PAD : 0);
        unsigned address = channel->parent && (flags & J2534_ADDR) ? 5 : 4;

        if (len < address + (channel->parent ? 1u : 0u) || flags & ~allowed ||
            !can_id_valid(data, flags) ||
            !can_flags_match(physical_channel(channel), flags))
            return J2534_MSG;

        if (!channel->parent)
            return len <= 12 ? J2534_OK : J2534_MSG;

        if (len - address > ISOTP_MAX_PAYLOAD || (periodic && len > 11))
            return J2534_MSG;

        unsigned sf = 7 - (address - 4);

        if (len - address > sf && (flags != channel->remote_flags ||
                                   memcmp(data, channel->remote, address)))
            return J2534_MSG_NOT_ALLOWED;

        /* The shared engine requires the same addressing format in both
         * directions. */
        if ((flags ^ channel->remote_flags) & (J2534_ADDR | J2534_29BIT))
            return J2534_MSG_NOT_ALLOWED;

        return J2534_OK;
    }
    if (channel->protocol == J2534_PWM || channel->protocol == J2534_VPW) {
        if (flags || len > (channel->protocol == J2534_PWM ? 10u : 11u) ||
            (channel->protocol == J2534_PWM && len < 3))
            return J2534_MSG;

        if (channel->protocol == J2534_PWM) {
            uint32_t node;

            if (vif_bus_param_get(VIF_OWNER_LINK, channel->bus,
                                  BUS_P_NODE_ADDRESS, &node) ||
                data[2] != node)
                return J2534_MSG;
        }
        return J2534_OK;
    }
    if (flags & ~J2534_WAIT_P3 ||
        len > 260u - !(channel->flags & J2534_CHECKSUM_DISABLED) ||
        (periodic && len > 12))
        return J2534_MSG;

    if (channel->protocol == J2534_ISO14230) {
        unsigned header = (data[0] & 0xc0) ? 3 : 1;
        unsigned body = data[0] & 63;

        if ((data[0] & 0xc0) == 0x40 && data[0] != 0x48 && data[0] != 0x68)
            return J2534_MSG;

        if (!body) {
            if (len <= header)
                return J2534_MSG;

            body = data[header++];
        }
        if (!body ||
            len != header + body + !!(channel->flags & J2534_CHECKSUM_DISABLED))
            return J2534_MSG;
    }
    return J2534_OK;
}

static void echo_can(channel_t *physical, const struct can_frame *frame,
                     uint32_t timestamp) {
    if (!physical->echo)
        return;
    uint8_t data[12];
    put_be32(data, frame->id & CAN_EFF_MASK);
    memcpy(data + 4, frame->data, frame->dlc);
    uint32_t flags = frame->id & CAN_EFF_FLAG ? J2534_29BIT : 0;
    if (filter_accepts(physical, data, frame->dlc + 4, flags))
        enqueue_rx(physical, 0, flags | J2534_TX_MSG, timestamp, data,
                   frame->dlc + 4, false);
}

static void cancel_channel_tx(channel_t *channel) {
    channel_t *physical = physical_channel(channel);
    if (!physical || physical->pending.channel != channel->id)
        return;
    physical->pending.discard = true;
    vif_bus_tx_cancel(VIF_OWNER_LINK, physical->bus);
}

static void tx_done(channel_t *channel, bool ok) {
    uint32_t handle = j2534_u32(channel->tx + 4);

    if (handle)
        enqueue_rx(channel, handle, ok ? J2534_TX_SUCCESS : J2534_TX_FAILED,
                   now_us(), NULL, 0, true);
    channel->tx_len = 0;
    channel->tx_active = false;
}

static int send_byte_message(channel_t *channel) {
    bus_msg_t msg;
    const uint8_t *data = channel->tx + MSG_HEADER;
    uint32_t flags = j2534_u32(channel->tx + 12);

    bus_msg_tx(&msg, data, channel->tx_len - MSG_HEADER, 0);
    return vif_bus_send(VIF_OWNER_LINK, channel->bus, &msg,
                        flags & J2534_WAIT_P3 ? BUS_TX_WAIT_P3_MIN_ONLY : 0);
}

static void confirm_protocol_tx(channel_t *channel, const pending_tx_t *pending,
                                bool success, uint32_t timestamp) {
    if (pending->kind == TX_FLOW_CONTROL) {
        if (pending->rx_generation == channel->rx_generation)
            isotp_rx_confirm(&channel->iso_rx, success, timestamp);
    } else if (pending->kind == TX_ISOTP) {
        int rc = isotp_tx_confirm(&channel->iso_tx, success, timestamp);
        if (rc < 0 || channel->iso_tx.state == ISOTP_TX_IDLE)
            tx_done(channel, rc >= 0);
    } else {
        tx_done(channel, success);
    }
}

static void finish_bus_tx(channel_t *physical, const bus_tx_result_t *result) {
    pending_tx_t pending = physical->pending;
    memset(&physical->pending, 0, sizeof(physical->pending));
    channel_t *channel = find_channel(pending.channel);
    if (result->status == 0 && physical->bus == VIF_BUS_CAN)
        echo_can(physical, &pending.frame, result->ended_us);
    if (!channel)
        return;
    bool success = result->status == 0;
    if (pending.kind == TX_SCHEDULED) {
        j2534_schedule_job_t *job =
            j2534_schedule_find(&channel->scheduler, pending.job);
        bool deferred =
            result->status == BUS_ERR_RX_PENDING ||
            (result->status == BUS_ERR_CANCELLED && pending.discard);
        if (job) {
            uint32_t start = success ? result->started_us : now_us();
            uint32_t end = success ? result->ended_us : start;
            if (deferred)
                j2534_schedule_defer(job);
            else
                j2534_schedule_complete(job, success, start, end);
        }
        if (!pending.discard && !deferred &&
            result->status != BUS_ERR_CANCELLED && pending.handle)
            enqueue_rx(channel, pending.handle,
                       success ? J2534_TX_SUCCESS : J2534_TX_FAILED,
                       result->ended_us, NULL, 0, true);
        return;
    }
    if (pending.discard || pending.confirmed)
        return;
    confirm_protocol_tx(channel, &pending, success, now_us());
}

static void collect_bus_tx(channel_t *physical) {
    bus_tx_result_t result;
    if (physical->pending.kind == TX_NONE ||
        !vif_bus_tx_poll(VIF_OWNER_LINK, physical->bus, &result))
        return;
    if (result.complete) {
        finish_bus_tx(physical, &result);
    } else if (!physical->pending.discard &&
               (physical->pending.kind == TX_ISOTP ||
                physical->pending.kind == TX_FLOW_CONTROL)) {
        channel_t *channel = find_channel(physical->pending.channel);
        if (channel && !physical->pending.confirmed) {
            confirm_protocol_tx(channel, &physical->pending, true,
                                result.ended_us);
            physical->pending.confirmed = true;
        }
    } else if (physical->pending.kind == TX_SCHEDULED) {
        channel_t *channel = find_channel(physical->pending.channel);
        j2534_schedule_job_t *job =
            channel ? j2534_schedule_find(&channel->scheduler,
                                          physical->pending.job)
                    : NULL;
        if (job)
            j2534_schedule_transmitted(job, result.started_us, result.ended_us);
    }
}

static void submit_bus_tx(channel_t *channel, tx_kind_t kind, uint32_t job,
                          uint32_t handle, const bus_msg_t *msg,
                          uint32_t bus_flags) {
    channel_t *physical = physical_channel(channel);
    physical->pending = (pending_tx_t){
        .kind = kind,
        .channel = channel->id,
        .job = job,
        .handle = handle,
        .rx_generation = channel->rx_generation,
    };
    if (physical->bus == VIF_BUS_CAN) {
        physical->pending.frame.id = msg->id;
        physical->pending.frame.dlc = msg->len;
        memcpy(physical->pending.frame.data, msg->data, msg->len);
        bus_flags |= BUS_TX_WAIT_DONE;
    }
    int rc = vif_bus_submit(VIF_OWNER_LINK, physical->bus, msg, bus_flags,
                            physical->rx_sequence);
    if (rc) {
        bus_tx_result_t result = {.status = rc,
                                  .complete = true,
                                  .started_us = now_us(),
                                  .ended_us = now_us()};
        finish_bus_tx(physical, &result);
    }
}

static void submit_can(channel_t *channel, tx_kind_t kind,
                       const struct can_frame *frame) {
    bus_msg_t msg;
    bus_msg_tx(&msg, frame->data, frame->dlc, frame->id);
    submit_bus_tx(channel, kind, 0, 0, &msg, 0);
}

static void poll_tx(channel_t *channel) {
    if (!channel->tx_len)
        return;
    const uint8_t *data = channel->tx + MSG_HEADER;
    size_t len = channel->tx_len - MSG_HEADER;
    uint32_t flags = j2534_u32(channel->tx + 12);
    if (channel->parent) {
        if (!channel->tx_active) {
            if (!(channel->flags & LOGICAL_FULL_DUPLEX) &&
                channel->iso_rx.state != ISOTP_RX_IDLE)
                return;
            isotp_config_t cfg = channel->config;
            cfg.tx_id = be32(data) | (flags & J2534_29BIT ? CAN_EFF_FLAG : 0);
            cfg.tx_address = flags & J2534_ADDR ? data[4] : 0;
            cfg.pad = !!(flags & J2534_PAD);
            cfg.timeout_a_us = channel->tx_timeout_a_us;
            cfg.functional =
                memcmp(data, channel->remote, 4 + cfg.extended_address) != 0;
            unsigned address = 4 + cfg.extended_address;
            isotp_tx_init(&channel->iso_tx, &cfg);
            if (isotp_tx_start(&channel->iso_tx, data + address, len - address,
                               now_us()) < 0) {
                tx_done(channel, false);
                return;
            }
            channel->tx_active = true;
        }
        struct can_frame frame;
        int rc = isotp_tx_next(&channel->iso_tx, &frame, now_us());
        if (rc == ISOTP_FRAME)
            submit_can(channel, TX_ISOTP, &frame);
        else if (rc < 0)
            tx_done(channel, false);
        return;
    }
    bus_msg_t msg;
    if (channel->bus == VIF_BUS_CAN)
        bus_msg_tx(&msg, data + 4, len - 4,
                   be32(data) | (flags & J2534_29BIT ? CAN_EFF_FLAG : 0));
    else
        bus_msg_tx(&msg, data, len, 0);
    submit_bus_tx(channel, TX_MESSAGE, 0, 0, &msg,
                  flags & J2534_WAIT_P3 ? BUS_TX_WAIT_P3_MIN_ONLY : 0);
}

static void submit_scheduled(channel_t *channel, j2534_schedule_job_t *job) {
    const j2534_scheduled_message_t *message = &job->message;
    const uint8_t *data = message->data;
    uint32_t flags = message->flags;
    bus_msg_t msg;
    struct can_frame frame;
    if (channel->parent) {
        isotp_config_t cfg = channel->config;
        cfg.tx_id = be32(data) | (flags & J2534_29BIT ? CAN_EFF_FLAG : 0);
        cfg.tx_address = flags & J2534_ADDR ? data[4] : 0;
        cfg.pad = !!(flags & J2534_PAD);
        unsigned address = 4 + cfg.extended_address;
        isotp_tx_t tx;
        isotp_tx_init(&tx, &cfg);
        if (isotp_tx_start(&tx, data + address, message->len - address,
                           now_us()) < 0 ||
            isotp_tx_next(&tx, &frame, now_us()) != ISOTP_FRAME) {
            job->active = false;
            return;
        }
        bus_msg_tx(&msg, frame.data, frame.dlc, frame.id);
    } else if (channel->bus == VIF_BUS_CAN) {
        bus_msg_tx(&msg, data + 4, message->len - 4,
                   be32(data) | (flags & J2534_29BIT ? CAN_EFF_FLAG : 0));
    } else {
        bus_msg_tx(&msg, data, message->len, 0);
    }
    uint32_t bus_flags = flags & J2534_WAIT_P3 ? BUS_TX_WAIT_P3_MIN_ONLY : 0;
    if (job->kind != J2534_SCHEDULE_PERIODIC)
        bus_flags |= BUS_TX_CHECK_RX;
    job->pending = true;
    job->transmission_ended = false;
    submit_bus_tx(channel, TX_SCHEDULED, job->id, message->handle, &msg,
                  bus_flags);
}

static void logical_receive(channel_t *channel, const struct can_frame *frame,
                            uint32_t timestamp) {
    if (channel->tx_active) {
        int rc = isotp_tx_flow_control(&channel->iso_tx, frame, now_us());
        if (rc < 0)
            tx_done(channel, false);
        if (!(channel->flags & LOGICAL_FULL_DUPLEX))
            return;
    }
    isotp_segment_t segment;
    int rc = isotp_rx_feed(&channel->iso_rx, frame, now_us(), &segment);

    if (rc <= 0)
        return;
    unsigned address = 4 + channel->config.extended_address;
    uint32_t flags = channel->local_flags;

    put_be32(channel->assembly, frame->id & CAN_EFF_MASK);
    if (address == 5)
        channel->assembly[4] = frame->data[0];
    if (segment.started)
        channel->rx_generation++;
    if (segment.started && !segment.complete)
        enqueue_rx(channel, 0, flags | J2534_START, timestamp,
                   channel->assembly, address, true);
    if (segment.complete) {
        j2534_schedule_receive(&channel->scheduler, channel->assembly,
                               address + segment.total, flags, timestamp);
        if (channel->legacy_channel) {
            for (unsigned i = J2534_PHYSICAL_MAX; i < CHANNELS; i++) {
                channel_t *peer = &channels[i];
                if (peer != channel && peer->id && peer->legacy_channel &&
                    peer->parent == channel->parent)
                    j2534_schedule_receive(&peer->scheduler, channel->assembly,
                                           address + segment.total, flags,
                                           timestamp);
            }
        }
        enqueue_rx(
            channel, 0, flags | (segment.padding_error ? J2534_PADDING : 0),
            timestamp, channel->assembly, address + segment.total, false);
    }
    struct can_frame flow_control;

    if (isotp_rx_flow_control(&channel->iso_rx, &flow_control, now_us()) ==
        ISOTP_FRAME) {
        channel->flow_control = flow_control;
        channel->flow_control_pending = true;
    }
}

static void poll_rx(channel_t *channel) {
    channel->rx_drained = false;
    for (unsigned burst = 0; burst < 32; burst++) {
        bus_msg_t msg;

        bus_msg_init(&msg, scratch + 4, J2534_DATA_MAX);
        int received = vif_bus_recv(VIF_OWNER_LINK, channel->bus, &msg, 0);

        if (received < 0) {
            channel->rx_drained = true;
            break;
        }
        collect_bus_tx(channel);
        if ((int32_t)(msg.sequence - channel->rx_sequence) > 0)
            channel->rx_sequence = msg.sequence;
        if (msg.status & BUS_RX_LINK_DOWN) {
            const uint8_t reason[4] = {1, 0, 0, 0};

            channel->link_down = true;
            for (unsigned i = 0; i < CHANNELS; i++) {
                channel_t *affected = &channels[i];

                if (affected != channel && affected->parent != channel->id)
                    continue;

                j2534_schedule_receive_lost(&affected->scheduler);
                cancel_channel_tx(affected);
                if (affected->tx_len)
                    tx_done(affected, false);
                if (affected->parent) {
                    isotp_tx_init(&affected->iso_tx, &affected->config);

                    affected->iso_rx.state = ISOTP_RX_IDLE;
                }
                enqueue_rx(affected, 0, J2534_ERROR, msg.timestamp_us, reason,
                           sizeof(reason), true);
            }
            continue;
        }
        if (msg.status & BUS_RX_BUFFER_OVERFLOW) {
            channel->overflow = true;
            j2534_schedule_receive_lost(&channel->scheduler);
            for (unsigned i = J2534_PHYSICAL_MAX; i < CHANNELS; i++)
                if (channels[i].parent == channel->id) {
                    channels[i].overflow = true;
                    j2534_schedule_receive_lost(&channels[i].scheduler);
                }
        }
        if (!msg.len && (msg.status & BUS_RX_BUFFER_OVERFLOW))
            continue;
        if (msg.status & BUS_RX_BREAK) {
            enqueue_rx(channel, 0, J2534_BREAK, msg.timestamp_us, NULL, 0,
                       true);
            continue;
        }
        if (msg.status & BUS_RX_START_OF_MSG) {
            enqueue_rx(channel, 0, J2534_START, msg.timestamp_us, NULL, 0,
                       true);
            continue;
        }
        uint32_t flags = 0;
        uint8_t *data = scratch + 4;
        size_t len = msg.len;

        if (channel->protocol == J2534_CAN) {
            if ((msg.id & CAN_RTR_FLAG) || len > 8)
                continue;

            flags = msg.id & CAN_EFF_FLAG ? J2534_29BIT : 0;
            if (!can_flags_match(channel, flags))
                continue;

            struct can_frame frame = {.id = msg.id, .dlc = len};

            memcpy(frame.data, data, len);
            for (unsigned i = J2534_PHYSICAL_MAX; i < CHANNELS; i++)
                if (channels[i].parent == channel->id)
                    logical_receive(&channels[i], &frame, msg.timestamp_us);
            data = scratch;
            put_be32(data, msg.id & CAN_EFF_MASK);
            len += 4;
        } else {
            if (!(channel->flags & J2534_CHECKSUM_DISABLED)) {
                if (!len || (msg.status & BUS_RX_BAD_CHECKSUM))
                    continue;

                len--;
            }
            if (!len)
                continue;
        }
        if (msg.status & BUS_RX_TX_MSG_TYPE)
            flags |= J2534_TX_MSG;
        else
            j2534_schedule_receive(&channel->scheduler, data, len, flags,
                                   msg.timestamp_us);
        if (filter_accepts(channel, data, len, flags))
            enqueue_rx(channel, 0, flags, msg.timestamp_us, data, len, false);
    }
}

static void service_bus(channel_t *physical) {
    collect_bus_tx(physical);
    poll_rx(physical);
    uint32_t now = now_us();
    channel_t *selected = NULL;
    j2534_schedule_job_t *next = NULL;
    for (unsigned i = 0; i < CHANNELS; i++) {
        channel_t *channel = &channels[i];
        if (!channel->id || physical_channel(channel) != physical)
            continue;
        if (channel->parent)
            isotp_rx_check_timeout(&channel->iso_rx, now);
        if (physical->rx_drained)
            j2534_schedule_expire(&channel->scheduler, now);
        j2534_schedule_job_t *job =
            j2534_schedule_next(&channel->scheduler, now, physical->rx_drained);
        if (job && (!next || (int32_t)(job->due_us - next->due_us) < 0 ||
                    (job->due_us == next->due_us &&
                     (int32_t)(job->id - next->id) < 0))) {
            next = job;
            selected = channel;
        }
    }
    if (physical->pending.kind == TX_SCHEDULED) {
        channel_t *owner = find_channel(physical->pending.channel);
        j2534_schedule_job_t *job =
            owner
                ? j2534_schedule_find(&owner->scheduler, physical->pending.job)
                : NULL;
        if (!job || !job->active)
            vif_bus_tx_cancel(VIF_OWNER_LINK, physical->bus);
    }
    if (physical->pending.kind != TX_NONE || physical->link_down)
        return;

    /* Flow control has protocol deadlines; scheduled data precedes queued data.
     */
    for (unsigned sent = 0; sent < J2534_LOGICAL_MAX; sent++) {
        channel_t *flow = NULL;
        for (unsigned i = J2534_PHYSICAL_MAX; i < CHANNELS; i++) {
            channel_t *channel = &channels[i];
            if (channel->parent == physical->id &&
                channel->flow_control_pending &&
                (!flow || (int32_t)(channel->iso_rx.deadline_us -
                                    flow->iso_rx.deadline_us) < 0))
                flow = channel;
        }
        if (!flow)
            break;
        flow->flow_control_pending = false;
        submit_can(flow, TX_FLOW_CONTROL, &flow->flow_control);
        collect_bus_tx(physical);
        if (physical->pending.kind != TX_NONE)
            return;
    }
    if (next) {
        submit_scheduled(selected, next);
    } else {
        for (unsigned offset = 0; offset < CHANNELS; offset++) {
            unsigned i = (physical->tx_cursor + offset) % CHANNELS;
            channel_t *channel = &channels[i];
            if (channel->id && physical_channel(channel) == physical) {
                poll_tx(channel);
                if (physical->pending.kind != TX_NONE) {
                    physical->tx_cursor = (i + 1) % CHANNELS;
                    break;
                }
            }
        }
    }
    collect_bus_tx(physical);
}

void j2534_core_poll(void) {
    if (!device)
        return;
    for (unsigned i = 0; i < J2534_PHYSICAL_MAX; i++)
        if (channels[i].id && channel_is_live(&channels[i]))
            service_bus(&channels[i]);
}

static uint32_t connect_channel(const opendiag_Connect *req) {
    if (req->device != device)
        return J2534_DEVICE;

    uint32_t proto = req->protocol, flags = req->flags;
    uint32_t rate = req->baudrate, count = req->pins_count;

    if (req->connector != 1)
        return J2534_PIN;

    vif_bus_t bus;
    uint32_t pin1, pin2 = 0, allowed = 0;

    switch (proto) {
    case J2534_CAN:
        bus = VIF_BUS_CAN;
        pin1 = 6;
        pin2 = 14;
        allowed = J2534_29BIT | J2534_CAN_BOTH;
        if (rate != 50000 && rate != 125000 && rate != 250000 &&
            rate != 500000 && rate != 1000000)
            return J2534_BAUD;

        break;
    case J2534_PWM:
        bus = VIF_BUS_J1850_PWM;
        pin1 = 2;
        pin2 = 10;
        if (rate != 41600)
            return J2534_BAUD;

        break;
    case J2534_VPW:
        bus = VIF_BUS_J1850_VPW;
        pin1 = 2;
        if (rate != 10400)
            return J2534_BAUD;

        break;
    case J2534_ISO9141:
    case J2534_ISO14230:
        bus = VIF_BUS_KLINE;
        pin1 = 7;
        allowed = J2534_CHECKSUM_DISABLED | J2534_K_ONLY;
        if (!(flags & J2534_K_ONLY)) {
            pin2 = 15;
        }

        if (rate < KLINE_BAUD_MIN || rate > KLINE_BAUD_HW_MAX)
            return J2534_BAUD;

        break;
    default:
        return J2534_PROTOCOL;
    }
    if (flags & ~allowed)
        return J2534_FLAGS;

    if (count != (pin2 ? 2u : 1u) || req->pins[0] != pin1 ||
        (pin2 && req->pins[1] != pin2))
        return J2534_PIN;

    if (vif_any_pin_active() ||
        vif_bus_current(VIF_OWNER_LINK, bus) != VIF_BUS_NONE ||
        vif_bus_current(VIF_OWNER_SHELL, bus) != VIF_BUS_NONE)
        return J2534_CONFLICT;

    channel_t *channel = allocate_channel(false);

    if (!channel)
        return J2534_LIMIT;

    channel->bus = bus;
    channel->protocol = proto;
    channel->flags = flags;
    channel->rate = rate;
    vif_bus_cfg_t cfg = {.bitrate = rate, .manual_recovery = true};

    if (vif_bus_open(VIF_OWNER_LINK, bus, &cfg) != ESP_OK) {
        /* VIF retains failed claims for teardown; never forget those claims. */
        vif_bus_close(VIF_OWNER_LINK, bus);
        release_channel(channel);
        return J2534_CONFLICT;
    }
    int rc = 0;

    if (bus == VIF_BUS_KLINE) {
        rc = vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_K_LINE_ONLY,
                               !!(flags & J2534_K_ONLY));
        if (!rc) {
            rc = vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_DATA_RATE, rate);
        }
        if (!rc)
            rc = vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_CHECKSUM_RX,
                                   !(flags & J2534_CHECKSUM_DISABLED));
        if (!rc)
            rc = vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_CHECKSUM_TX,
                                   !(flags & J2534_CHECKSUM_DISABLED));
        if (!rc)
            rc = vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_KLINE_FRAME_MODE,
                                   proto == J2534_ISO9141
                                       ? BUS_KLINE_FRAME_ISO9141
                                       : BUS_KLINE_FRAME_ISO14230);
        if (!rc)
            rc = vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_PERIODIC_MS, 0);
        if (!rc)
            rc =
                vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_PERIODIC_QUIET, 0);
    }
    if (bus != VIF_BUS_CAN)
        vif_bus_param_set(VIF_OWNER_LINK, bus, BUS_P_DUPLICATE_MS, 0);
    if (rc) {
        disconnect_channel(channel);
        return J2534_VALUE;
    }
    response.id = channel->id;
    return J2534_OK;
}

static uint32_t logical_connect(channel_t *parent,
                                const opendiag_LogicalConnect *req) {
    if (parent->protocol != J2534_CAN)
        return J2534_LOGICAL;
    if (req->protocol != J2534_ISOTP)
        return J2534_PROTOCOL;

    uint32_t flags = req->flags, local_flags = req->local_flags,
             remote_flags = req->remote_flags;
    const uint8_t *local = req->local_address.bytes,
                  *remote = req->remote_address.bytes;

    if (flags & ~(uint32_t)LOGICAL_FULL_DUPLEX)
        return J2534_FLAGS;

    if (req->local_address.size != 4 + !!(local_flags & J2534_ADDR) ||
        req->remote_address.size != 4 + !!(remote_flags & J2534_ADDR) ||
        local_flags & ~(J2534_29BIT | J2534_ADDR) ||
        remote_flags & ~(J2534_29BIT | J2534_ADDR | J2534_PAD) ||
        !can_id_valid(local, local_flags) ||
        !can_id_valid(remote, remote_flags) ||
        !can_flags_match(parent, local_flags) ||
        !can_flags_match(parent, remote_flags) ||
        ((local_flags ^ remote_flags) & (J2534_ADDR | J2534_29BIT)))
        return J2534_DESCRIPTOR;

    for (unsigned i = J2534_PHYSICAL_MAX; i < CHANNELS; i++) {
        channel_t *channel = &channels[i];

        if (channel->parent == parent->id &&
            channel->local_flags == local_flags &&
            !memcmp(channel->local, local, 4 + !!(local_flags & J2534_ADDR)))
            return J2534_NOT_UNIQUE;
    }
    channel_t *channel = allocate_channel(true);

    if (!channel)
        return J2534_LIMIT;

    channel->parent = parent->id;
    channel->bus = parent->bus;
    channel->protocol = J2534_ISOTP;
    channel->flags = flags;
    channel->local_flags = local_flags;
    channel->remote_flags = remote_flags;
    channel->tx_timeout_a_us = ISOTP_DEFAULT_TIMEOUT_US;
    memcpy(channel->local, local, 5);
    memcpy(channel->remote, remote, 5);
    channel->config = isotp_config_default(
        be32(remote) | (remote_flags & J2534_29BIT ? CAN_EFF_FLAG : 0),
        be32(local) | (local_flags & J2534_29BIT ? CAN_EFF_FLAG : 0));
    channel->config.extended_address = !!(local_flags & J2534_ADDR);
    channel->config.rx_address = local[4];
    channel->legacy_channel = req->legacy_channel;
    channel->config.tx_address = remote[4];
    channel->config.pad = !!(remote_flags & J2534_PAD);
    isotp_rx_init(&channel->iso_rx, &channel->config,
                  channel->assembly + 4 + channel->config.extended_address,
                  ISOTP_MAX_PAYLOAD);
    isotp_tx_init(&channel->iso_tx, &channel->config);

    response.id = channel->id;
    return J2534_OK;
}

static uint32_t configure_logical(channel_t *channel, bool set, uint32_t param,
                                  uint32_t *value) {
    if (set && (channel->tx_len || channel->iso_rx.state != ISOTP_RX_IDLE))
        return J2534_CONFLICT;
    uint32_t current_value;

    switch (param) {
    case CONFIG_ISOTP_BLOCK_SIZE:
        current_value = channel->config.block_size;
        break;
    case CONFIG_ISOTP_STMIN:
        current_value = channel->config.stmin;
        break;
    case CONFIG_ISOTP_WAIT_LIMIT:
        current_value = channel->config.wait_limit;
        break;
    case CONFIG_ISOTP_PAD_VALUE:
        current_value = channel->config.pad_value;
        break;
    case CONFIG_N_AS_MAX:
        current_value = channel->tx_timeout_a_us / 1000;
        break;
    case CONFIG_N_AR_MAX:
        current_value = channel->config.timeout_a_us / 1000;
        break;
    case CONFIG_N_BS_MAX:
        current_value = channel->config.timeout_bs_us / 1000;
        break;
    case CONFIG_N_CR_MAX:
        current_value = channel->config.timeout_cr_us / 1000;
        break;
    default:
        return J2534_PARAM;
    }
    if (!set) {
        *value = current_value;
        return J2534_OK;
    }
    if ((param <= CONFIG_ISOTP_PAD_VALUE && *value > 255) ||
        (param == CONFIG_ISOTP_STMIN && *value > 0x7f &&
         (*value < 0xf1 || *value > 0xf9)) ||
        (param >= CONFIG_N_AS_MAX && (!*value || *value > 60000)))
        return J2534_VALUE;

    switch (param) {
    case CONFIG_ISOTP_BLOCK_SIZE:
        channel->config.block_size = *value;
        break;
    case CONFIG_ISOTP_STMIN:
        channel->config.stmin = *value;
        break;
    case CONFIG_ISOTP_WAIT_LIMIT:
        channel->config.wait_limit = *value;
        break;
    case CONFIG_ISOTP_PAD_VALUE:
        channel->config.pad_value = *value;
        break;
    case CONFIG_N_AS_MAX:
        channel->tx_timeout_a_us = *value * 1000;
        break;
    case CONFIG_N_AR_MAX:
        channel->config.timeout_a_us = *value * 1000;
        break;
    case CONFIG_N_BS_MAX:
        channel->config.timeout_bs_us = *value * 1000;
        break;
    case CONFIG_N_CR_MAX:
        channel->config.timeout_cr_us = *value * 1000;
        break;
    }
    isotp_rx_init(&channel->iso_rx, &channel->config,
                  channel->assembly + 4 + channel->config.extended_address,
                  ISOTP_MAX_PAYLOAD);

    return J2534_OK;
}

static uint32_t configure_channel(channel_t *channel, bool set, uint32_t param,
                                  uint32_t *value) {
    if (channel->parent)
        return configure_logical(channel, set, param, value);

    if (param == BUS_P_LOOPBACK) {
        if (!set) {
            *value = channel->echo;
            return J2534_OK;
        }
        if (*value > 1)
            return J2534_VALUE;

        if (channel->bus != VIF_BUS_CAN &&
            vif_bus_param_set(VIF_OWNER_LINK, channel->bus, param, *value))
            return J2534_VALUE;

        channel->echo = *value;
        return J2534_OK;
    }
    if (param >= BUS_P_VENDOR_BASE)
        return J2534_PARAM;

    if (param == BUS_P_DATA_RATE && channel->bus == VIF_BUS_CAN) {
        if (!set) {
            *value = channel->rate;
            return J2534_OK;
        }
        /* Reconnect to retime without disturbing active logical channels. */
        return *value == channel->rate ? J2534_OK : J2534_VALUE;
    }
    int rc =
        set ? vif_bus_param_set(VIF_OWNER_LINK, channel->bus, param, *value)
            : vif_bus_param_get(VIF_OWNER_LINK, channel->bus, param, value);

    if (!rc) {
        if (set && param == BUS_P_DATA_RATE)
            channel->rate = *value;
        return J2534_OK;
    }
    return rc == BUS_ERR_UNSUPPORTED ? J2534_PARAM : J2534_VALUE;
}

static uint32_t ioctl_request(channel_t *channel, const opendiag_Ioctl *req) {
    uint32_t id = req->id;
    size_t len = req->data.size;
    const uint8_t *data = req->data.bytes;

    if (id == IOCTL_GET_CONFIG || id == IOCTL_SET_CONFIG) {
        if (len || req->connector || req->pin)
            return J2534_VALUE;

        for (unsigned i = 0; i < req->config_count; i++) {
            opendiag_Config v = req->config[i];
            uint32_t rc = configure_channel(channel, id == IOCTL_SET_CONFIG,
                                            v.parameter, &v.value);

            if (rc)
                return rc;

            response.config[response.config_count++] = v;
        }
        return J2534_OK;
    }
    if (req->config_count || req->connector || req->pin)
        return J2534_VALUE;

    if (id == BUS_IOCTL_FIVE_BAUD_INIT || id == BUS_IOCTL_FAST_INIT) {
        if (channel->bus != VIF_BUS_KLINE || channel->parent)
            return J2534_IOCTL;

        if (channel->tx_len)
            return J2534_CONFLICT;

        bus_init_t init = {0};

        if (id == BUS_IOCTL_FIVE_BAUD_INIT) {
            if (len != 1)
                return J2534_MSG;

            init.address = data[0];
        } else {
            if (channel->protocol != J2534_ISO14230)
                return J2534_IOCTL;

            if (!len || len > 8)
                return J2534_MSG;

            if (channel->flags & J2534_CHECKSUM_DISABLED)
                return J2534_NOT_SUPPORTED;

            uint32_t rc = validate_message(channel, channel->protocol, 0, data,
                                           len, false);

            if (rc)
                return rc;

            init.msg_len = len;
            memcpy(init.msg, data, init.msg_len);
        }
        int rc = vif_bus_ioctl(VIF_OWNER_LINK, channel->bus, id, &init, &init);

        vif_bus_param_set(VIF_OWNER_LINK, channel->bus, BUS_P_PERIODIC_MS, 0);
        if (rc)
            return J2534_INIT;

        if (id == BUS_IOCTL_FIVE_BAUD_INIT) {
            memcpy(response.data.bytes, init.key, 2);
            response.data.size = 2;
        } else {
            response.data.size = init.reply_len ? init.reply_len - 1 : 0;
            if (response.data.size > sizeof(response.data.bytes))
                return J2534_FULL;
            memcpy(response.data.bytes, init.reply, response.data.size);
        }
        return J2534_OK;
    }
    if (id == BUS_IOCTL_ADD_FUNCT_ADDR || id == BUS_IOCTL_DEL_FUNCT_ADDR) {
        if (channel->protocol != J2534_PWM)
            return J2534_IOCTL;

        for (size_t i = 0; i < len; i++) {
            int rc =
                vif_bus_ioctl(VIF_OWNER_LINK, channel->bus, id, &data[i], NULL);

            if (rc)
                return bus_error(rc);
        }
        return J2534_OK;
    }
    if (len)
        return J2534_MSG;

    switch (id) {
    case BUS_IOCTL_CLEAR_TX_QUEUE:
        cancel_channel_tx(channel);
        channel->tx_len = 0;
        channel->tx_active = false;
        if (channel->parent)
            isotp_tx_init(&channel->iso_tx, &channel->config);
        return J2534_OK;
    case BUS_IOCTL_CLEAR_RX_QUEUE: {
        channel_t *physical = physical_channel(channel);
        /* Repeat conditions still observe the input removed from the API queue.
         */
        poll_rx(physical);
        if (!physical->rx_drained)
            poll_rx(physical);
        channel->rx_head = channel->rx_used = 0;
        channel->overflow = false;
        if (channel->parent) {
            channel->rx_generation++;
            channel->flow_control_pending = false;
            isotp_rx_init(&channel->iso_rx, &channel->config,
                          channel->assembly + 4 +
                              channel->config.extended_address,
                          ISOTP_MAX_PAYLOAD);
        }
        return J2534_OK;
    }
    case BUS_IOCTL_CLEAR_PERIODIC:
        for (unsigned i = 0; i < J2534_SCHEDULE_SLOTS; i++) {
            j2534_schedule_job_t *job = &channel->scheduler.jobs[i];
            if (job->id) {
                channel_t *physical = physical_channel(channel);
                if (physical->pending.kind == TX_SCHEDULED &&
                    physical->pending.job == job->id) {
                    physical->pending.discard = true;
                    vif_bus_tx_cancel(VIF_OWNER_LINK, physical->bus);
                }
                memset(job, 0, sizeof(*job));
            }
        }
        return J2534_OK;
    case IOCTL_CLEAR_FILTERS:
        if (channel->parent)
            return J2534_IOCTL;

        memset(channel->filters, 0, sizeof(channel->filters));
        return J2534_OK;
    case BUS_IOCTL_BUS_ON: {
        if (channel->protocol != J2534_CAN)
            return J2534_IOCTL;

        poll_rx(channel);
        int rc = vif_bus_ioctl(VIF_OWNER_LINK, channel->bus, BUS_IOCTL_BUS_ON,
                               NULL, NULL);

        if (rc)
            return J2534_INIT;

        channel->link_down = false;
        return J2534_OK;
    }
    case BUS_IOCTL_CLEAR_FUNCT_TABLE:
        if (channel->protocol != J2534_PWM)
            return J2534_IOCTL;

        return bus_error(
            vif_bus_ioctl(VIF_OWNER_LINK, channel->bus, id, NULL, NULL));
    default:
        return J2534_IOCTL;
    }
}

static uint32_t set_programming_voltage(const opendiag_Voltage *req) {
    if (req->device != device)
        return J2534_DEVICE;

    if (req->connector != 1)
        return J2534_PIN;

    uint32_t pin = req->pin;
    uint32_t mv = req->millivolts;
    vif_pin_mode_t mode;

    if (pin != 6 && pin != 9 && pin != 11 && pin != 12 && pin != 13 &&
        pin != 14 && pin != 15)
        return J2534_PIN;

    if (mv == UINT32_MAX) {
        if (vif_pin_set(VIF_OWNER_LINK, pin, VIF_PIN_OFF, 0) != ESP_OK)
            return J2534_PIN_IN_USE;

        if ((int)pin == voltage_pin)
            voltage_pin = -1;
        return J2534_OK;
    }
    vif_bus_claim_t claims[VIF_BUS_GROUPS];
    vif_bus_info(claims);
    for (unsigned i = 0; i < VIF_BUS_GROUPS; i++)
        if ((claims[i].bus == VIF_BUS_CAN && (pin == 6 || pin == 14)) ||
            (claims[i].bus == VIF_BUS_KLINE && pin == 15))
            return J2534_PIN_IN_USE;

    if (mv == UINT32_MAX - 1) {
        bool groundable = (pin == 15);
#if OPENDIAG_HW_LS_OBD_9
        groundable = groundable || (pin == 9);
#endif
        if (!groundable)
            return J2534_VALUE;

        mode = VIF_PIN_GROUND;
        mv = 0;
    } else {
        if (pin == 15 || mv < 5000 || mv > 20000)
            return J2534_VALUE;

        if (voltage_pin >= 0 && voltage_pin != (int)pin)
            return J2534_VOLTAGE_IN_USE;

        mode = VIF_PIN_VOLTAGE;
    }
    if (vif_pin_set(VIF_OWNER_LINK, pin, mode, mv) != ESP_OK)
        return J2534_PIN_IN_USE;

    if (mode == VIF_PIN_VOLTAGE)
        voltage_pin = pin;

    return J2534_OK;
}

static void store_tx(channel_t *channel, const opendiag_Message *message) {
    memset(channel->tx, 0, MSG_HEADER);
    j2534_put32(channel->tx, message->protocol);
    j2534_put32(channel->tx + 4, message->handle);
    j2534_put32(channel->tx + 12, message->tx_flags);
    memcpy(channel->tx + MSG_HEADER, message->data.bytes, message->data.size);
    channel->tx_len = MSG_HEADER + message->data.size;
}

_Static_assert(BUS_INIT_MSG_MAX == sizeof(((opendiag_FastInit *)0)->data.bytes),
               "FAST_INIT capability and protobuf capacity must agree");

static uint32_t fast_init(channel_t *channel, const opendiag_FastInit *req) {
    if (channel->bus != VIF_BUS_KLINE || channel->parent)
        return J2534_IOCTL;

    bool checksum = !(channel->flags & J2534_CHECKSUM_DISABLED);
    if (req->tx_flags & ~J2534_WAIT_P3 ||
        req->data.size > BUS_INIT_MSG_MAX - (checksum ? 1u : 0u))
        return J2534_MSG;

    if (channel->pending.kind != TX_NONE) {
        int rc = vif_bus_tx_wait(VIF_OWNER_LINK, channel->bus);
        if (rc)
            return bus_error(rc);
        collect_bus_tx(channel);
    }
    if (channel->tx_len) {
        int rc = send_byte_message(channel);
        tx_done(channel, rc == 0);
        if (rc)
            return bus_error(rc);
    }
    bus_init_t init = {0};
    init.raw_response = true;
    init.no_response = req->no_response;
    init.msg_len = req->data.size;
    init.tx_flags = req->tx_flags & J2534_WAIT_P3 ? BUS_TX_WAIT_P3_MIN_ONLY : 0;
    memcpy(init.msg, req->data.bytes, init.msg_len);
    int rc = vif_bus_ioctl(VIF_OWNER_LINK, channel->bus, BUS_IOCTL_FAST_INIT,
                           &init, &init);
    if (rc)
        return bus_error(rc);
    if (!req->no_response) {
        if (init.reply_len <= (checksum ? 1u : 0u) ||
            init.reply_len > sizeof(init.reply))
            return J2534_INIT;
        response.has_message = true;
        response.message.protocol = channel->protocol;
        response.message.timestamp_us = init.reply_timestamp_us;
        response.message.data.size = init.reply_len - (checksum ? 1u : 0u);
        response.message.extra_data_index = response.message.data.size;
        memcpy(response.message.data.bytes, init.reply,
               response.message.data.size);
    }
    return J2534_OK;
}

static uint32_t read_capabilities(void) {
    response.has_capabilities = true;
    response.capabilities = (opendiag_Capabilities){
        .wire_version = 1,
        .api_version = 0x0500,
        .physical_channels = J2534_PHYSICAL_MAX,
        .logical_channels = J2534_LOGICAL_MAX,
        .filters_per_channel = J2534_FILTER_MAX,
        .periodic_per_channel = J2534_PERIODIC_MAX,
        .fragment_bytes = J2534_FRAGMENT_MAX,
        .rpc_bytes = J2534_RPC_MAX,
        .rx_queue_bytes = RX_BYTES,
        .tx_queue_messages = 1,
        .fast_init_max_data = BUS_INIT_MSG_MAX,
        .repeat_per_channel = J2534_REPEAT_MAX,
        .protocols_count = 6,
        .protocols = {{J2534_CAN, 12, 12, 12},
                      {J2534_PWM, 10, 11, 10},
                      {J2534_VPW, 11, 11, 11},
                      {J2534_ISO9141, 259, 259, 12},
                      {J2534_ISO14230, 259, 259, 12},
                      {J2534_ISOTP, 4100, 4100, 11}},
    };
    return J2534_OK;
}

static uint32_t select_channels(const opendiag_Select *select) {
    if (select->type != SELECT_READABLE)
        return J2534_SELECT;

    for (unsigned i = 0; i < select->channels_count; i++)
        if (!channel_is_live(find_channel(select->channels[i])))
            return J2534_CHANNEL;

    for (unsigned i = 0; i < select->channels_count; i++) {
        channel_t *channel = find_channel(select->channels[i]);

        if (channel->rx_used)
            response.channels[response.channels_count++] = channel->id;
    }
    return J2534_OK;
}

static uint32_t read_message(channel_t *channel) {
    opendiag_Message *message = &response.message;
    uint8_t header[MSG_HEADER];
    size_t total;

    if (!channel->rx_used) {
        if (channel->overflow) {
            channel->overflow = false;
            return J2534_OVERFLOW;
        }
        return J2534_EMPTY;
    }
    ring_copy(channel, 0, header, MSG_HEADER, false);
    message->protocol = j2534_u32(header);
    message->handle = j2534_u32(header + 4);
    message->rx_status = j2534_u32(header + 8);
    message->tx_flags = j2534_u32(header + 12);
    message->timestamp_us = j2534_u32(header + 16);
    message->data.size = j2534_u32(header + 20);
    message->extra_data_index = j2534_u32(header + 24);
    ring_copy(channel, MSG_HEADER, message->data.bytes, message->data.size,
              false);
    total = MSG_HEADER + message->data.size;
    channel->rx_head = (channel->rx_head + total) % RX_BYTES;
    channel->rx_used -= total;
    response.has_message = true;
    response.count = 1;
    return J2534_OK;
}

static uint32_t queue_message(channel_t *channel, const opendiag_Queue *queue) {
    const opendiag_Message *message = &queue->message;
    uint32_t rc;

    if (!queue->has_message)
        return J2534_MSG;

    rc = validate_message(channel, message->protocol, message->tx_flags,
                          message->data.bytes, message->data.size, false);
    if (rc)
        return rc;

    if (channel->tx_len)
        return J2534_FULL;

    store_tx(channel, message);
    response.count = 1;
    return J2534_OK;
}

static uint32_t start_periodic(channel_t *channel,
                               const opendiag_Periodic *periodic) {
    const opendiag_Message *message = &periodic->message;
    uint32_t rc;

    if (!periodic->has_message)
        return J2534_MSG;

    if (periodic->interval_ms < 5 || periodic->interval_ms > 65535)
        return J2534_INTERVAL;

    rc = validate_message(channel, message->protocol, message->tx_flags,
                          message->data.bytes, message->data.size, true);
    if (rc)
        return rc;

    j2534_schedule_job_t *job =
        j2534_schedule_allocate(&channel->scheduler, false);
    if (!job)
        return J2534_LIMIT;
    *job = (j2534_schedule_job_t){
        .id = new_id(),
        .kind = J2534_SCHEDULE_PERIODIC,
        .interval_us = periodic->interval_ms * 1000,
        .due_us = now_us(),
        .active = true,
        .message = {.flags = message->tx_flags,
                    .handle = message->handle,
                    .len = message->data.size},
    };
    memcpy(job->message.data, message->data.bytes, message->data.size);
    response.id = job->id;
    return J2534_OK;
}

static uint32_t start_filter(channel_t *channel,
                             const opendiag_Filter *filter) {
    size_t len = filter->mask.size;

    if (channel->parent)
        return J2534_FILTER_TYPE;

    if (filter->type != FILTER_PASS && filter->type != FILTER_BLOCK)
        return J2534_FILTER_TYPE;

    if (!len || len > (channel->protocol == J2534_PWM ? 10u : 12u) ||
        filter->pattern.size != len)
        return J2534_MSG;

    if (filter->flags & ~(channel->protocol == J2534_CAN ? J2534_29BIT : 0) ||
        (channel->protocol == J2534_CAN &&
         !can_flags_match(channel, filter->flags)))
        return J2534_MSG;

    for (unsigned i = 0; i < J2534_FILTER_MAX; i++) {
        filter_t *entry = &channel->filters[i];

        if (entry->id)
            continue;

        entry->id = new_id();
        entry->type = filter->type;
        entry->flags = filter->flags;
        entry->len = len;
        memcpy(entry->mask, filter->mask.bytes, len);
        memcpy(entry->pattern, filter->pattern.bytes, len);
        response.id = entry->id;
        return J2534_OK;
    }
    return J2534_LIMIT;
}

static uint32_t stop_scheduled(channel_t *channel, uint32_t id, bool repeat) {
    j2534_schedule_job_t *job = j2534_schedule_find(&channel->scheduler, id);
    if (!job || (job->kind != J2534_SCHEDULE_PERIODIC) != repeat)
        return J2534_MSG_ID;
    channel_t *physical = physical_channel(channel);
    if (physical->pending.kind == TX_SCHEDULED && physical->pending.job == id) {
        physical->pending.discard = true;
        vif_bus_tx_cancel(VIF_OWNER_LINK, physical->bus);
    }
    memset(job, 0, sizeof(*job));
    return J2534_OK;
}

static uint32_t stop_periodic(channel_t *channel, uint32_t id) {
    return stop_scheduled(channel, id, false);
}

uint32_t j2534_core_repeat_start(uint32_t id, const j2534_repeat_setup_t *setup,
                                 uint32_t *repeat_id) {
    channel_t *channel = find_channel(id);
    if (!channel_is_live(channel))
        return J2534_CHANNEL;
    if (!setup || !repeat_id)
        return J2534_VALUE;
    if (setup->interval_ms < 5 || setup->interval_ms > 65535)
        return J2534_INTERVAL;
    if (setup->condition > 1)
        return J2534_VALUE;
    const j2534_repeat_message_t *message = &setup->message;
    const j2534_repeat_message_t *mask = &setup->mask;
    const j2534_repeat_message_t *pattern = &setup->pattern;
    if (message->len > sizeof(message->data) ||
        mask->len > sizeof(mask->data) || !mask->len ||
        mask->len != pattern->len || mask->flags != pattern->flags)
        return J2534_MSG;
    if (mask->protocol != channel->protocol ||
        pattern->protocol != channel->protocol)
        return J2534_MSG_PROTOCOL;
    uint32_t rc = validate_message(channel, message->protocol, message->flags,
                                   message->data, message->len, true);
    if (rc)
        return rc;
    uint32_t match_flags = channel->parent
                               ? J2534_ADDR | J2534_29BIT | J2534_PAD
                           : channel->bus == VIF_BUS_CAN   ? J2534_29BIT
                           : channel->bus == VIF_BUS_KLINE ? J2534_WAIT_P3
                                                           : 0;
    if (mask->flags & ~match_flags)
        return J2534_FLAGS;
    if (channel->bus == VIF_BUS_CAN &&
        !can_flags_match(physical_channel(channel), mask->flags))
        return J2534_FLAGS;
    if (channel->parent &&
        !!(mask->flags & J2534_ADDR) != channel->config.extended_address)
        return J2534_FLAGS;
    unsigned max_match = channel->parent                  ? 11
                         : channel->protocol == J2534_PWM ? 10
                         : channel->protocol == J2534_VPW ? 11
                                                          : 12;
    if (mask->len > max_match)
        return J2534_MSG;
    j2534_schedule_job_t *job =
        j2534_schedule_allocate(&channel->scheduler, true);
    if (!job)
        return J2534_LIMIT;
    uint32_t now = now_us();
    *job = (j2534_schedule_job_t){
        .id = new_id(),
        .kind = setup->condition ? J2534_SCHEDULE_WHILE : J2534_SCHEDULE_UNTIL,
        .interval_us = setup->interval_ms * 1000,
        .due_us = now,
        .created_us = now,
        .active = true,
        .message = {.flags = message->flags,
                    .handle = message->handle,
                    .len = message->len},
        .match_flags = mask->flags & (J2534_ADDR | J2534_29BIT),
        .match_len = mask->len,
    };
    memcpy(job->message.data, message->data, message->len);
    memcpy(job->mask, mask->data, mask->len);
    memcpy(job->pattern, pattern->data, mask->len);
    *repeat_id = job->id;
    return J2534_OK;
}

uint32_t j2534_core_repeat_query(uint32_t channel_id, uint32_t id,
                                 bool *active) {
    channel_t *channel = find_channel(channel_id);
    if (!channel_is_live(channel))
        return J2534_CHANNEL;
    if (!active)
        return J2534_VALUE;
    j2534_schedule_job_t *job = j2534_schedule_find(&channel->scheduler, id);
    if (!job || job->kind == J2534_SCHEDULE_PERIODIC)
        return J2534_MSG_ID;
    *active = job->active;
    return J2534_OK;
}

uint32_t j2534_core_repeat_stop(uint32_t channel_id, uint32_t id) {
    channel_t *channel = find_channel(channel_id);
    if (!channel_is_live(channel))
        return J2534_CHANNEL;
    return stop_scheduled(channel, id, true);
}

static void decode_repeat_message(const opendiag_RepeatMessage *source,
                                  j2534_repeat_message_t *message) {
    _Static_assert(sizeof(source->data.bytes) == sizeof(message->data),
                   "Repeat wire and scheduler capacities must agree");
    message->protocol = source->protocol;
    message->handle = source->handle;
    message->flags = source->tx_flags;
    message->len = source->data.size;
    memcpy(message->data, source->data.bytes, source->data.size);
}

static uint32_t start_repeat(const opendiag_Repeat *req) {
    if (!req->has_message || !req->has_mask || !req->has_pattern)
        return J2534_MSG;
    j2534_repeat_setup_t setup = {
        .interval_ms = req->interval_ms,
        .condition = req->condition,
    };
    decode_repeat_message(&req->message, &setup.message);
    decode_repeat_message(&req->mask, &setup.mask);
    decode_repeat_message(&req->pattern, &setup.pattern);
    return j2534_core_repeat_start(req->channel, &setup, &response.id);
}

static uint32_t stop_filter(channel_t *channel, uint32_t id) {
    for (unsigned i = 0; i < J2534_FILTER_MAX; i++)
        if (channel->filters[i].id && channel->filters[i].id == id) {
            memset(&channel->filters[i], 0, sizeof(channel->filters[i]));
            return J2534_OK;
        }
    return J2534_FILTER_ID;
}

static uint32_t dispatch_channel_request(const opendiag_Request *req) {
    channel_t *channel;
    uint32_t id;

    switch (req->which_command) {
    case opendiag_Request_disconnect_tag:
        id = req->command.disconnect.id;
        break;
    case opendiag_Request_logical_disconnect_tag:
        id = req->command.logical_disconnect.id;
        break;
    case opendiag_Request_read_tag:
        id = req->command.read.id;
        break;
    case opendiag_Request_queue_tag:
        id = req->command.queue.channel;
        break;
    case opendiag_Request_start_periodic_tag:
        id = req->command.start_periodic.channel;
        break;
    case opendiag_Request_stop_periodic_tag:
        id = req->command.stop_periodic.channel;
        break;
    case opendiag_Request_start_repeat_tag:
        id = req->command.start_repeat.channel;
        break;
    case opendiag_Request_query_repeat_tag:
        id = req->command.query_repeat.channel;
        break;
    case opendiag_Request_stop_repeat_tag:
        id = req->command.stop_repeat.channel;
        break;
    case opendiag_Request_start_filter_tag:
        id = req->command.start_filter.channel;
        break;
    case opendiag_Request_stop_filter_tag:
        id = req->command.stop_filter.channel;
        break;
    case opendiag_Request_fast_init_tag:
        id = req->command.fast_init.channel;
        break;
    case opendiag_Request_ioctl_tag:
        id = req->command.ioctl.target;
        break;
    case opendiag_Request_logical_connect_tag:
        id = req->command.logical_connect.physical;
        break;
    default:
        return J2534_NOT_SUPPORTED;
    }
    channel = find_channel(id);
    if (!channel)
        return J2534_CHANNEL;

    if (req->which_command == opendiag_Request_disconnect_tag ||
        req->which_command == opendiag_Request_logical_disconnect_tag) {
        if ((req->which_command == opendiag_Request_logical_disconnect_tag) !=
            !!channel->parent)
            return J2534_CHANNEL;

        return disconnect_channel(channel);
    }
    if (!channel_is_live(channel))
        return J2534_CHANNEL;

    switch (req->which_command) {
    case opendiag_Request_read_tag:
        return read_message(channel);
    case opendiag_Request_queue_tag:
        return queue_message(channel, &req->command.queue);
    case opendiag_Request_start_periodic_tag:
        return start_periodic(channel, &req->command.start_periodic);
    case opendiag_Request_stop_periodic_tag:
        return stop_periodic(channel, req->command.stop_periodic.id);
    case opendiag_Request_start_repeat_tag:
        return start_repeat(&req->command.start_repeat);
    case opendiag_Request_query_repeat_tag:
        return j2534_core_repeat_query(id, req->command.query_repeat.id,
                                       &response.repeat_active);
    case opendiag_Request_stop_repeat_tag:
        return j2534_core_repeat_stop(id, req->command.stop_repeat.id);
    case opendiag_Request_start_filter_tag:
        return start_filter(channel, &req->command.start_filter);
    case opendiag_Request_stop_filter_tag:
        return stop_filter(channel, req->command.stop_filter.id);
    case opendiag_Request_fast_init_tag:
        return fast_init(channel, &req->command.fast_init);
    case opendiag_Request_ioctl_tag:
        return ioctl_request(channel, &req->command.ioctl);
    case opendiag_Request_logical_connect_tag:
        return logical_connect(channel, &req->command.logical_connect);
    default:
        return J2534_NOT_SUPPORTED;
    }
}

static uint32_t read_voltage(const opendiag_Ioctl *io) {
    int32_t mv;

    if (io->target != device)
        return J2534_DEVICE;

    if (io->data.size || io->config_count)
        return J2534_VALUE;

    if ((io->id == IOCTL_READ_VBATT && (io->connector != 1 || io->pin != 16)) ||
        (io->id == IOCTL_READ_PROG_VOLTAGE && (io->connector || io->pin)))
        return J2534_PIN;

    mv = io->id == IOCTL_READ_VBATT ? vif_vbatt_mv() : vif_hs_vsense_mv();
    if (mv < 0)
        return J2534_FAILED;

    response.millivolts = mv;
    return J2534_OK;
}

static uint32_t dispatch_request(void) {
    const opendiag_Request *req = &request;

    switch (req->which_command) {
    case opendiag_Request_capabilities_tag:
        return read_capabilities();
    case opendiag_Request_last_error_tag:
        snprintf(response.text, sizeof(response.text), "J2534 status 0x%02lx",
                 (unsigned long)last_error);
        return J2534_OK;
    case opendiag_Request_open_tag:
        if (device)
            return J2534_IN_USE;

        /* A failed earlier open may have left a driver requiring teardown. */
        if (vif_bus_release_all(VIF_OWNER_LINK) != ESP_OK)
            return J2534_FAILED;

        device = new_id();
        response.id = device;
        return J2534_OK;
    default:
        break;
    }
    if (!device)
        return J2534_NOT_OPEN;

    switch (req->which_command) {
    case opendiag_Request_close_tag:
        if (req->command.close.id != device)
            return J2534_DEVICE;

        if (vif_pin_release_all(VIF_OWNER_LINK) != ESP_OK)
            return J2534_FAILED;

        voltage_pin = -1;
        if (vif_bus_release_all(VIF_OWNER_LINK) != ESP_OK)
            return J2534_FAILED;

        for (unsigned i = 0; i < CHANNELS; i++)
            release_channel(&channels[i]);
        device = 0;
        return J2534_OK;
    case opendiag_Request_connect_tag:
        return connect_channel(&req->command.connect);
    case opendiag_Request_voltage_tag:
        return set_programming_voltage(&req->command.voltage);
    case opendiag_Request_version_tag:
        if (req->command.version.id != device)
            return J2534_DEVICE;

        snprintf(response.text, sizeof(response.text),
                 "OpenDIAG %s; protobuf wire 1; J2534 05.00 subset",
                 esp_app_get_description()->version);
        return J2534_OK;
    case opendiag_Request_select_tag:
        return select_channels(&req->command.select);
    case opendiag_Request_ioctl_tag:
        if (req->command.ioctl.id == IOCTL_READ_VBATT ||
            req->command.ioctl.id == IOCTL_READ_PROG_VOLTAGE)
            return read_voltage(&req->command.ioctl);

        break;
    default:
        break;
    }
    return dispatch_channel_request(req);
}

size_t j2534_core_request(uint8_t op, const uint8_t *in, size_t len,
                          uint8_t out[J2534_RPC_MAX]) {
    memset(&request, 0, sizeof(request));
    memset(&response, 0, sizeof(response));
    pb_istream_t input = pb_istream_from_buffer(in, len);

    if (!pb_decode(&input, opendiag_Request_fields, &request) ||
        request.which_command != (unsigned)op + 1)
        response.status = J2534_MSG;
    else
        response.status = dispatch_request();
    if (response.status)
        last_error = response.status;
    pb_ostream_t output = pb_ostream_from_buffer(out, J2534_RPC_MAX);

    if (!pb_encode(&output, opendiag_Response_fields, &response)) {
        /* Fallback encodes Response.status (field 1, varint). */
        out[0] = 8;
        out[1] = J2534_FAILED;
        return 2;
    }
    return output.bytes_written;
}

bool j2534_core_start(void) {
    j2534_core_stop();
    last_error = 0;
    return true;
}

void j2534_core_stop(void) {
    /* VIF joins workers after stop(); cancel queued copies before dropping IDs.
     */
    for (unsigned i = 0; i < J2534_PHYSICAL_MAX; i++)
        if (channels[i].id)
            vif_bus_tx_cancel(VIF_OWNER_LINK, channels[i].bus);
    for (unsigned i = 0; i < CHANNELS; i++)
        release_channel(&channels[i]);
    device = 0;
    voltage_pin = -1;
}
