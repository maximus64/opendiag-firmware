/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "fake_bus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_port.h"
#include "j2534.h"
#include "j2534.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "td_test.h"

static opendiag_Request req;
static opendiag_Response res;
static uint8_t encoded[J2534_RPC_MAX], reply[J2534_RPC_MAX];
static uint32_t dev;
static uint32_t wire_call(void);

static void command(unsigned tag) {
    memset(&req, 0, sizeof(req));
    req.which_command = tag;
}
static uint32_t call(void) {
    pb_ostream_t output = pb_ostream_from_buffer(encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(pb_encode(&output, opendiag_Request_fields, &req));
    size_t len = j2534_core_request(req.which_command - 1, encoded,
                                    output.bytes_written, reply);
    pb_istream_t input = pb_istream_from_buffer(reply, len);
    memset(&res, 0, sizeof(res));
    TEST_ASSERT_TRUE(pb_decode(&input, opendiag_Response_fields, &res));
    return res.status;
}
static uint32_t connect_can(uint32_t flags) {
    command(opendiag_Request_connect_tag);
    req.command.connect = (opendiag_Connect){.device = dev,
                                             .protocol = J2534_CAN,
                                             .flags = flags,
                                             .baudrate = 500000,
                                             .connector = 1,
                                             .pins_count = 2,
                                             .pins = {6, 14}};
    TEST_ASSERT_EQUAL_INT(0, call());
    return res.id;
}
static uint32_t logical(uint32_t physical, bool extended) {
    command(opendiag_Request_logical_connect_tag);
    opendiag_LogicalConnect *c = &req.command.logical_connect;
    c->physical = physical;
    c->protocol = J2534_ISOTP;
    c->local_flags = extended ? J2534_ADDR : 0;
    c->remote_flags = c->local_flags | J2534_PAD;
    c->local_address.size = c->remote_address.size = extended ? 5 : 4;
    memcpy(c->local_address.bytes, "\0\0\x07\xe8\xf1", 5);
    memcpy(c->remote_address.bytes, "\0\0\x07\xe0\x10", 5);
    TEST_ASSERT_EQUAL_INT(0, call());
    return res.id;
}
static uint32_t filter(uint32_t ch, uint32_t type, uint8_t mask,
                       uint8_t pattern) {
    command(opendiag_Request_start_filter_tag);
    req.command.start_filter = (opendiag_Filter){.channel = ch, .type = type};
    req.command.start_filter.mask.size = req.command.start_filter.pattern.size =
        1;
    req.command.start_filter.mask.bytes[0] = mask;
    req.command.start_filter.pattern.bytes[0] = pattern;
    TEST_ASSERT_EQUAL_INT(0, call());
    return res.id;
}
static void queue(uint32_t ch, uint32_t proto, uint32_t flags,
                  const uint8_t *data, size_t len) {
    command(opendiag_Request_queue_tag);
    req.command.queue.channel = ch;
    req.command.queue.has_message = true;
    opendiag_Message *m = &req.command.queue.message;
    m->protocol = proto;
    m->handle = 42;
    m->tx_flags = flags;
    m->data.size = len;
    memcpy(m->data.bytes, data, len);
}
static uint32_t read_message(uint32_t ch) {
    command(opendiag_Request_read_tag);
    req.command.read.id = ch;
    return call();
}
void td_setup(void) {
    static bool initialized;
    if (!initialized) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());
        fake_port_register_all();
        initialized = true;
    }
    j2534_core_stop();
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());
    fake_clock_reset();
    fake_can_reset();
    fake_bus_reset_all();
    fake_port_reset();
    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_link_start(&j2534_frontend));
    vif_shell_bind();
    vif_link_set_port(fake_port_id(0));
    TEST_ASSERT_TRUE(j2534_frontend.start());
    command(opendiag_Request_open_tag);
    TEST_ASSERT_EQUAL_INT(0, call());
    dev = res.id;
}
void td_teardown(void) {
    j2534_frontend.stop();
    vif_bus_release_all(VIF_OWNER_LINK);
    vif_bus_release_all(VIF_OWNER_SHELL);
    vif_pin_release_all(VIF_OWNER_LINK);
    vif_pin_release_all(VIF_OWNER_SHELL);
    TEST_ASSERT_EQUAL_INT(0, idf_stub_lock_balance());
}
TEST(capabilities_are_explicit_about_limits) {
    command(opendiag_Request_capabilities_tag);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_TRUE(res.has_capabilities);
    TEST_ASSERT_FALSE(res.capabilities.full_j2534_compliance);
    TEST_ASSERT_EQUAL_INT(2, res.capabilities.logical_channels);
    TEST_ASSERT_EQUAL_INT(6, res.capabilities.protocols_count);
}
TEST(device_and_channel_handles_are_validated) {
    command(opendiag_Request_open_tag);
    TEST_ASSERT_EQUAL_INT(J2534_IN_USE, call());
    command(opendiag_Request_close_tag);
    req.command.close.id = dev + 1;
    TEST_ASSERT_EQUAL_INT(J2534_DEVICE, call());
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, read_message(0));
    command(opendiag_Request_close_tag);
    req.command.close.id = dev;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(J2534_NOT_OPEN, read_message(123));
}
TEST(conflicting_open_does_not_reconfigure_existing_driver) {
    uint32_t ch = connect_can(0);
    command(opendiag_Request_connect_tag);
    req.command.connect = (opendiag_Connect){.device = dev,
                                             .protocol = J2534_CAN,
                                             .baudrate = 250000,
                                             .connector = 1,
                                             .pins_count = 2,
                                             .pins = {6, 14}};
    TEST_ASSERT_EQUAL_INT(J2534_CONFLICT, call());
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
}
TEST(shell_claim_is_respected) {
    vif_bus_cfg_t cfg = {.bitrate = 500000};
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &cfg));
    command(opendiag_Request_connect_tag);
    req.command.connect = (opendiag_Connect){.device = dev,
                                             .protocol = J2534_CAN,
                                             .baudrate = 500000,
                                             .connector = 1,
                                             .pins_count = 2,
                                             .pins = {6, 14}};
    TEST_ASSERT_EQUAL_INT(J2534_CONFLICT, call());
    TEST_ASSERT_TRUE(vif_bus_is_open(VIF_OWNER_SHELL, VIF_BUS_CAN));
}
TEST(default_filter_blocks_and_block_overrides_pass) {
    uint32_t ch = connect_can(0);
    const uint8_t data[] = {1, 2, 3};
    fake_can_stage_stale(0x7e8, 3, data);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    filter(ch, 1, 0, 0);
    fake_can_stage_stale(0x7e8, 3, data);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(7, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM("\0\0\x07\xe8\x01\x02\x03", res.message.data.bytes,
                          7);
    TEST_ASSERT_EQUAL_INT(7, res.message.extra_data_index);
    filter(ch, 2, 0, 0);
    fake_can_stage_stale(0x7e8, 3, data);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
}
TEST(queued_is_not_transmitted_until_poll_and_completion_bypasses_filter) {
    uint32_t ch = connect_can(0);
    queue(ch, J2534_CAN, 0, (const uint8_t *)"\0\0\x07\xdf\x01\x02", 6);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(J2534_FULL, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_SUCCESS, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(42, res.message.handle);
    TEST_ASSERT_EQUAL_INT(0, res.message.data.size);
}
TEST(failed_send_produces_failed_indication) {
    uint32_t ch = connect_can(0);
    queue(ch, J2534_CAN, 0, (const uint8_t *)"\0\0\x07\xdf", 4);
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_can_fail_next_send();
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_FAILED, res.message.rx_status);
}
TEST(invalid_can_identifiers_and_protocols_are_rejected) {
    uint32_t ch = connect_can(0);
    queue(ch, J2534_CAN, 0, (const uint8_t *)"\0\0\x08\x00", 4);
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
    req.command.queue.message.protocol = J2534_ISOTP;
    TEST_ASSERT_EQUAL_INT(J2534_MSG_PROTOCOL, call());
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}
TEST(logical_single_frame_receive_is_independent_of_physical_filter) {
    uint32_t p = connect_can(0), ch = logical(p, false);
    const uint8_t data[] = {3, 0x41, 0x0c, 0x55, 0, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, data);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_ISOTP, res.message.protocol);
    TEST_ASSERT_EQUAL_MEM("\0\0\x07\xe8\x41\x0c\x55", res.message.data.bytes,
                          7);
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(p));
}
TEST(logical_multiframe_receive_emits_start_and_flow_control) {
    uint32_t ch = logical(connect_can(0), false);
    const uint8_t first[] = {0x10, 10, 1, 2, 3, 4, 5, 6};
    const uint8_t last[] = {0x21, 7, 8, 9, 10, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, first);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x7e0, fake_can_sent(0)->id);
    TEST_ASSERT_EQUAL_INT(0x30, fake_can_sent(0)->data[0]);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_START, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(0, res.message.extra_data_index);
    fake_can_stage_stale(0x7e8, 8, last);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(14, res.message.data.size);
    for (unsigned i = 0; i < 10; i++)
        TEST_ASSERT_EQUAL_INT(i + 1, res.message.data.bytes[i + 4]);
}
TEST(logical_transmit_waits_for_flow_control) {
    uint32_t ch = logical(connect_can(0), false);
    const uint8_t msg[] = {0, 0, 7, 0xe0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    queue(ch, J2534_ISOTP, J2534_PAD, msg, sizeof(msg));
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x10, fake_can_sent(0)->data[0]);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    const uint8_t fc[] = {0x30, 0, 0, 0, 0, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, fc);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x21, fake_can_sent(1)->data[0]);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_SUCCESS, res.message.rx_status);
}
TEST(parent_disconnect_invalidates_logical_handle) {
    uint32_t p = connect_can(0), ch = logical(p, false);
    command(opendiag_Request_disconnect_tag);
    req.command.disconnect.id = p;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, read_message(ch));
    TEST_ASSERT_FALSE(fake_can_is_up());
}
TEST(failed_close_retains_handle_for_retry) {
    command(opendiag_Request_connect_tag);
    req.command.connect = (opendiag_Connect){.device = dev,
                                             .protocol = J2534_VPW,
                                             .baudrate = 10400,
                                             .connector = 1,
                                             .pins_count = 1,
                                             .pins = {2}};
    TEST_ASSERT_EQUAL_INT(0, call());
    uint32_t ch = res.id;
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_FAIL);
    command(opendiag_Request_disconnect_tag);
    req.command.disconnect.id = ch;
    TEST_ASSERT_EQUAL_INT(J2534_FAILED, call());
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_OK);
    TEST_ASSERT_EQUAL_INT(0, call());
}
TEST(protobuf_rejects_truncation_and_wrong_command) {
    const uint8_t truncated[] = {0x22, 10, 1};
    size_t n = j2534_core_request(3, truncated, sizeof(truncated), reply);
    pb_istream_t in = pb_istream_from_buffer(reply, n);
    TEST_ASSERT_TRUE(pb_decode(&in, opendiag_Response_fields, &res));
    TEST_ASSERT_EQUAL_INT(J2534_MSG, res.status);
    const uint8_t open[] = {0x12, 0};
    n = j2534_core_request(3, open, sizeof(open), reply);
    in = pb_istream_from_buffer(reply, n);
    TEST_ASSERT_TRUE(pb_decode(&in, opendiag_Response_fields, &res));
    TEST_ASSERT_EQUAL_INT(J2534_MSG, res.status);
}

TEST(maximum_isotp_payload_survives_protobuf_and_reassembly) {
    uint32_t ch = logical(connect_can(0), false);
    uint8_t first[] = {0x1f, 0xff, 0, 1, 2, 3, 4, 5};
    fake_can_stage_stale(0x7e8, 8, first);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_START, res.message.rx_status);
    unsigned offset = 6, seq = 1;
    while (offset < 4095) {
        uint8_t cf[8] = {0};
        cf[0] = 0x20 | (seq++ & 15);
        unsigned n = 4095 - offset < 7 ? 4095 - offset : 7;
        for (unsigned i = 0; i < n; i++)
            cf[i + 1] = (uint8_t)(offset + i);
        fake_can_stage_stale(0x7e8, 8, cf);
        j2534_core_poll();
        offset += n;
    }
    command(opendiag_Request_read_tag);
    req.command.read.id = ch;
    TEST_ASSERT_EQUAL_INT(0, wire_call());
    TEST_ASSERT_EQUAL_INT(4099, res.message.data.size);
    for (unsigned i = 0; i < 4095; i++)
        TEST_ASSERT_EQUAL_INT((uint8_t)i, res.message.data.bytes[i + 4]);
}
TEST(extended_address_is_preserved) {
    uint32_t ch = logical(connect_can(0), true);
    const uint8_t sf[] = {0xf1, 2, 0x41, 0x00, 0, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, sf);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_ADDR, res.message.rx_status);
    TEST_ASSERT_EQUAL_MEM("\0\0\x07\xe8\xf1\x41\0", res.message.data.bytes, 7);
}
TEST(flow_control_timeout_reports_failure) {
    uint32_t ch = logical(connect_can(0), false);
    uint8_t data[14] = {0, 0, 7, 0xe0};
    queue(ch, J2534_ISOTP, J2534_PAD, data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    fake_clock_advance_ms(1001);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_FAILED, res.message.rx_status);
}
TEST(periodic_messages_stop_and_clear) {
    uint32_t ch = connect_can(0);
    command(opendiag_Request_start_periodic_tag);
    opendiag_Periodic *p = &req.command.start_periodic;
    p->channel = ch;
    p->interval_ms = 10;
    p->has_message = true;
    p->message.protocol = J2534_CAN;
    p->message.data.size = 5;
    memcpy(p->message.data.bytes, "\0\0\x07\xdf\x55", 5);
    TEST_ASSERT_EQUAL_INT(0, call());
    uint32_t id = res.id;
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    fake_clock_advance_ms(10);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    command(opendiag_Request_stop_periodic_tag);
    req.command.stop_periodic = (opendiag_Object){ch, id};
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_clock_advance_ms(20);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID, call());
}
TEST(clear_tx_cancels_queued_message) {
    uint32_t ch = connect_can(0);
    queue(ch, J2534_CAN, 0, (uint8_t *)"\0\0\x07\xdf", 4);
    TEST_ASSERT_EQUAL_INT(0, call());
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl.target = ch;
    req.command.ioctl.id = 7;
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}
TEST(receive_overflow_is_reported_after_preserving_queued_messages) {
    uint32_t ch = connect_can(0);
    filter(ch, 1, 0, 0);
    const uint8_t data[8] = {0};
    for (unsigned i = 0; i < 220; i++) {
        fake_can_stage_stale(0x100, 8, data);
        j2534_core_poll();
    }
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    fake_can_stage_stale(0x100, 8, data);
    j2534_core_poll();
    bool saw_overflow = false;
    while (read_message(ch) == J2534_OK)
        saw_overflow |= !!(res.message.rx_status & J2534_RX_OVERFLOW);
    TEST_ASSERT_TRUE(saw_overflow);
}
TEST(bytebus_crc_is_removed_and_checksum_disabled_is_preserved) {
    command(opendiag_Request_connect_tag);
    req.command.connect = (opendiag_Connect){.device = dev,
                                             .protocol = J2534_VPW,
                                             .baudrate = 10400,
                                             .connector = 1,
                                             .pins_count = 1,
                                             .pins = {2}};
    TEST_ASSERT_EQUAL_INT(0, call());
    uint32_t ch = res.id;
    filter(ch, 1, 0, 0);
    const uint8_t raw[] = {0x48, 0x6b, 0x10, 0x41, 0, 0xaa};
    fake_bus_stage_stale(FAKE_BUS_J1850_VPW, raw, sizeof(raw));
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(5, res.message.data.size);
    command(opendiag_Request_connect_tag);
    req.command.connect =
        (opendiag_Connect){.device = dev,
                           .protocol = J2534_ISO9141,
                           .flags = J2534_K_ONLY | J2534_CHECKSUM_DISABLED,
                           .baudrate = 10400,
                           .connector = 1,
                           .pins_count = 1,
                           .pins = {7}};
    TEST_ASSERT_EQUAL_INT(0, call());
    ch = res.id;
    filter(ch, 1, 0, 0);
    fake_bus_stage_stale(FAKE_BUS_KLINE, raw, sizeof(raw));
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(6, res.message.data.size);
}
TEST(voltage_request_cannot_drive_active_can_pins) {
    connect_can(0);
    command(opendiag_Request_voltage_tag);
    req.command.voltage = (opendiag_Voltage){dev, 1, 6, 12000};
    TEST_ASSERT_EQUAL_INT(J2534_PIN_IN_USE, call());
    TEST_ASSERT_FALSE(vif_any_pin_active());
    req.command.voltage.pin = 12;
    req.command.voltage.millivolts = 30000;
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, call());
    TEST_ASSERT_FALSE(vif_any_pin_active());
}
static uint32_t crc(const uint8_t *p, size_t n) {
    uint32_t v = 0xffffffff;
    while (n--) {
        v ^= *p++;
        for (unsigned i = 0; i < 8; i++)
            v = (v >> 1) ^ (0xedb88320u & (0u - (v & 1)));
    }
    return ~v;
}
static size_t frame(uint8_t *dst, unsigned op, unsigned flags, unsigned offset,
                    const uint8_t *data, unsigned len) {
    memset(dst, 0, 20 + len);
    memcpy(dst, "J253", 4);
    dst[4] = 1;
    dst[5] = op;
    dst[6] = flags;
    j2534_put32(dst + 8, 77);
    dst[12] = offset;
    dst[13] = offset >> 8;
    dst[14] = len;
    dst[15] = len >> 8;
    memcpy(dst + 16, data, len);
    j2534_put32(dst + 16 + len, crc(dst + 4, 12 + len));
    return 20 + len;
}
TEST(wire_fragmentation_does_not_execute_until_final_fragment) {
    uint32_t ch = logical(connect_can(0), false);
    uint8_t data[260] = {0, 0, 7, 0xe0};
    queue(ch, J2534_ISOTP, J2534_PAD, data, sizeof(data));
    pb_ostream_t enc = pb_ostream_from_buffer(encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(pb_encode(&enc, opendiag_Request_fields, &req));
    uint8_t f[212];
    size_t n = frame(f, 15, 1, 0, encoded, 192);
    fake_port_reset();
    for (size_t i = 0; i < n; i++)
        j2534_frontend.feed(f + i, 1);
    TEST_ASSERT_EQUAL_INT(20, fake_port_len(0));
    TEST_ASSERT_EQUAL_INT(0x84, (uint8_t)fake_port_text(0)[6]);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
    n = frame(f, 15, 0, 192, encoded + 192, enc.bytes_written - 192);
    j2534_frontend.feed(f, n);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}
TEST(wire_crc_rejects_corrupt_command_and_resynchronizes) {
    uint8_t f[212];
    const uint8_t close[] = {0x1a, 2, 8, 0};
    size_t n = frame(f, 2, 0, 0, close, sizeof(close));
    f[n - 1] ^= 1;
    fake_port_reset();
    j2534_frontend.feed(f, n);
    TEST_ASSERT_EQUAL_INT(0, fake_port_len(0));
    const uint8_t caps[] = {0x0a, 0};
    n = frame(f, 0, 0, 0, caps, sizeof(caps));
    j2534_frontend.feed((const uint8_t *)"garbageJ", 8);
    j2534_frontend.feed(f, n);
    TEST_ASSERT_TRUE(fake_port_len(0) > 20);
}

static uint32_t wire_call(void) {
    pb_ostream_t enc = pb_ostream_from_buffer(encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(pb_encode(&enc, opendiag_Request_fields, &req));
    unsigned op = req.which_command - 1;
    uint8_t f[212];
    size_t n = frame(f, op, 0, 0, encoded, enc.bytes_written);
    fake_port_reset();
    j2534_frontend.feed(f, n);
    size_t used = 0;
    for (;;) {
        const uint8_t *data = (const uint8_t *)fake_port_text(0);
        TEST_ASSERT_TRUE(fake_port_len(0) >= 20);
        unsigned len = data[14] | (unsigned)data[15] << 8;
        TEST_ASSERT_EQUAL_INT(used, data[12] | (unsigned)data[13] << 8);
        TEST_ASSERT_EQUAL_INT(20 + len, fake_port_len(0));
        TEST_ASSERT_EQUAL_INT(crc(data + 4, 12 + len),
                              j2534_u32(data + 16 + len));
        TEST_ASSERT_TRUE(used + len <= sizeof(reply));
        memcpy(reply + used, data + 16, len);
        used += len;
        if (!(data[6] & 1))
            break;
        n = frame(f, op, 2, used, encoded, 0);
        fake_port_reset();
        j2534_frontend.feed(f, n);
    }
    pb_istream_t input = pb_istream_from_buffer(reply, used);
    TEST_ASSERT_TRUE(pb_decode(&input, opendiag_Response_fields, &res));
    return res.status;
}
TEST(
    interleaved_logical_receptions_keep_their_own_buffers_and_flow_control_ids) {
    uint32_t one = logical(connect_can(0), false);
    req.command.logical_connect.local_address.bytes[3] = 0xe9;
    req.command.logical_connect.remote_address.bytes[3] = 0xe1;
    TEST_ASSERT_EQUAL_INT(0, call());
    uint32_t two = res.id;
    const uint8_t ff1[] = {0x10, 10, 1, 2, 3, 4, 5, 6},
                  ff2[] = {0x10, 10, 11, 12, 13, 14, 15, 16};
    const uint8_t cf1[] = {0x21, 7, 8, 9, 10, 0, 0, 0},
                  cf2[] = {0x21, 17, 18, 19, 20, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, ff1);
    fake_can_stage_stale(0x7e9, 8, ff2);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x7e0, fake_can_sent(0)->id);
    TEST_ASSERT_EQUAL_INT(0x7e1, fake_can_sent(1)->id);
    TEST_ASSERT_EQUAL_INT(0, read_message(one));
    TEST_ASSERT_EQUAL_INT(J2534_START, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(0, read_message(two));
    TEST_ASSERT_EQUAL_INT(J2534_START, res.message.rx_status);
    fake_can_stage_stale(0x7e9, 8, cf2);
    fake_can_stage_stale(0x7e8, 8, cf1);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(one));
    for (unsigned i = 0; i < 10; i++)
        TEST_ASSERT_EQUAL_INT(i + 1, res.message.data.bytes[i + 4]);
    TEST_ASSERT_EQUAL_INT(0, read_message(two));
    for (unsigned i = 0; i < 10; i++)
        TEST_ASSERT_EQUAL_INT(i + 11, res.message.data.bytes[i + 4]);
}

TEST(k_line_and_l_line_require_matching_resources_and_preserve_flags) {
    const uint32_t protocols[] = {J2534_ISO9141, J2534_ISO14230};
    for (unsigned index = 0; index < 2; index++) {
        for (unsigned k_only = 0; k_only < 2; k_only++) {
            command(opendiag_Request_connect_tag);
            req.command.connect = (opendiag_Connect){
                .device = dev,
                .protocol = protocols[index],
                .flags = J2534_CHECKSUM_DISABLED | (k_only ? J2534_K_ONLY : 0),
                .baudrate = 10400,
                .connector = 1,
                .pins_count = k_only ? 2 : 1,
                .pins = {7, 15},
            };
            TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
            req.command.connect.pins_count = k_only ? 1 : 2;
            TEST_ASSERT_EQUAL_INT(0, call());
            uint32_t channel = res.id;
            uint32_t value = 99;
            TEST_ASSERT_EQUAL_INT(
                0, vif_bus_param_get(VIF_OWNER_LINK, VIF_BUS_KLINE,
                                     BUS_P_K_LINE_ONLY, &value));
            TEST_ASSERT_EQUAL_INT(k_only, value);
            command(opendiag_Request_disconnect_tag);
            req.command.disconnect.id = channel;
            TEST_ASSERT_EQUAL_INT(0, call());
        }
    }
}
