/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "fake_board.h"
#include "fake_bus.h"
#include "fake_bus_tx_worker.h"
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
    fake_bus_tx_hold(false);
    fake_bus_tx_hold_completion(false);
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
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT, call());
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

#if OPENDIAG_HW_LS_OBD_9
TEST(pin_9_ground_works_during_kline_and_is_released_on_close) {
    command(opendiag_Request_connect_tag);
    req.command.connect = (opendiag_Connect){.device = dev,
                                             .protocol = J2534_ISO9141,
                                             .baudrate = 10400,
                                             .connector = 1,
                                             .pins_count = 2,
                                             .pins = {7, 15}};
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    command(opendiag_Request_voltage_tag);
    req.command.voltage = (opendiag_Voltage){dev, 1, 9, UINT32_MAX - 1};
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(9));
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(9));
    req.command.voltage.millivolts = UINT32_MAX - 1;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    command(opendiag_Request_close_tag);
    req.command.close.id = dev;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(9));
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

TEST(programming_voltage_keeps_pin_9_modes_interlocked) {
    command(opendiag_Request_voltage_tag);
    req.command.voltage = (opendiag_Voltage){dev, 1, 9, UINT32_MAX - 1};
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(J2534_PIN_IN_USE, call());
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(9));
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    req.command.voltage.millivolts = UINT32_MAX - 1;
    TEST_ASSERT_EQUAL_INT(J2534_PIN_IN_USE, call());
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(9));
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(9));
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    req.command.voltage.pin = 12;
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    req.command.voltage.pin = 9;
    req.command.voltage.millivolts = UINT32_MAX - 1;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(9));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(12));
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(J2534_VOLTAGE_IN_USE, call());
}
#else
TEST(programming_voltage_rejects_pin_9_ground_without_the_hardware_mod) {
    command(opendiag_Request_voltage_tag);
    req.command.voltage = (opendiag_Voltage){dev, 1, 9, UINT32_MAX - 1};
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
    TEST_ASSERT_EQUAL_INT(-1, fake_board_ls_state(9));
    TEST_ASSERT_FALSE(vif_any_pin_active());
}
#endif

TEST(programming_voltage_validates_each_pins_supported_modes) {
    command(opendiag_Request_voltage_tag);
    req.command.voltage = (opendiag_Voltage){dev, 1, 6, UINT32_MAX - 1};
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
    req.command.voltage.pin = 15;
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
    req.command.voltage.millivolts = UINT32_MAX - 1;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(15));
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(J2534_OK, call());
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(15));
    req.command.voltage.pin = 9;
    req.command.voltage.millivolts = 4999;
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT, call());
    req.command.voltage.millivolts = 20001;
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT, call());
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
TEST(passthru_at_command_is_ignored_before_binary_request) {
    const uint8_t text[] = "AT VIF PASSTHRU\r";
    fake_port_reset();
    for (size_t i = 0; i < sizeof(text) - 1; i++) {
        j2534_frontend.feed(text + i, 1);
    }
    TEST_ASSERT_EQUAL_STRING("", fake_port_text(0));
    command(opendiag_Request_capabilities_tag);
    TEST_ASSERT_EQUAL_INT(0, wire_call());
    TEST_ASSERT_TRUE(res.has_capabilities);
}

TEST(startup_recovers_after_partial_binary_frame_timeout) {
    uint8_t partial[212];
    const uint8_t data[192] = {0};
    frame(partial, 0, 0, 0, data, sizeof(data));
    j2534_frontend.feed(partial, 17);
    const uint8_t text[] = "AT VIF PASSTHRU\r";
    j2534_frontend.feed(text, sizeof(text) - 1);
    fake_clock_advance_ms(2100);
    j2534_frontend.feed(text, sizeof(text) - 1);
    command(opendiag_Request_capabilities_tag);
    TEST_ASSERT_EQUAL_INT(0, wire_call());
    TEST_ASSERT_TRUE(res.has_capabilities);
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
            TEST_ASSERT_EQUAL_INT(J2534_CONFLICT, call());
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

static uint32_t hds_channel(bool checksum) {
    command(opendiag_Request_connect_tag);
    req.command.connect =
        (opendiag_Connect){.device = dev,
                           .protocol = J2534_ISO9141,
                           .flags = checksum ? 0 : J2534_CHECKSUM_DISABLED,
                           .baudrate = 10400,
                           .connector = 1,
                           .pins_count = 2,
                           .pins = {7, 15}};
    TEST_ASSERT_EQUAL_INT(0, call());
    return res.id;
}

static void hds_init(uint32_t channel, size_t length, bool no_response) {
    command(opendiag_Request_fast_init_tag);
    req.command.fast_init.channel = channel;
    req.command.fast_init.no_response = no_response;
    req.command.fast_init.data.size = length;
    memset(req.command.fast_init.data.bytes, 0x55, length);
}

TEST(j2534_fast_init_advertises_capacity_and_bypasses_filters_for_first_reply) {
    command(opendiag_Request_capabilities_tag);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(260, res.capabilities.fast_init_max_data);
    uint32_t ch = hds_channel(false);
    const uint8_t reply[] = {0x10, 0x02};
    fake_bus_kline_init_reply(reply, sizeof(reply), 123456);
    hds_init(ch, 260, false);
    req.command.fast_init.tx_flags = 0x200;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_TRUE(res.has_message);
    TEST_ASSERT_EQUAL_INT(2, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM(reply, res.message.data.bytes, 2);
    TEST_ASSERT_EQUAL_INT(123456, res.message.timestamp_us);
    const bus_init_t *init = fake_bus_kline_last_init();
    TEST_ASSERT_TRUE(init->raw_response);
    TEST_ASSERT_EQUAL_INT(260, init->msg_len);
    TEST_ASSERT_EQUAL_INT(BUS_TX_WAIT_P3_MIN_ONLY, init->tx_flags);
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
}

TEST(j2534_fast_init_checksum_and_invalid_request_have_correct_boundaries) {
    uint32_t ch = hds_channel(true);
    const uint8_t reply[] = {0x12, 0x34, 0x46};
    fake_bus_kline_init_reply(reply, sizeof(reply), 42);
    hds_init(ch, 260, false);
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
    hds_init(ch, 259, false);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(2, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM(reply, res.message.data.bytes, 2);
    hds_init(ch, 1, true);
    req.command.fast_init.tx_flags = 0x100;
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
}

TEST(j2534_fast_init_optional_io_and_bus_errors) {
    uint32_t ch = hds_channel(false);
    hds_init(ch, 0, true);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_FALSE(res.has_message);
    TEST_ASSERT_TRUE(fake_bus_kline_last_init()->no_response);
    TEST_ASSERT_EQUAL_INT(0, fake_bus_kline_last_init()->msg_len);
    fake_bus_kline_connect_result(BUS_ERR_ECHO);
    hds_init(ch, 1, false);
    TEST_ASSERT_EQUAL_INT(J2534_FAILED, call());
    TEST_ASSERT_FALSE(res.has_message);
    fake_bus_kline_connect_result(BUS_ERR_INIT);
    TEST_ASSERT_EQUAL_INT(J2534_INIT, call());
}

TEST(j2534_fast_init_drains_pending_tx_and_stops_on_tx_failure) {
    uint32_t ch = hds_channel(false);
    const uint8_t pid00[] = {0x68, 0x6a, 0xf1, 1, 0, 0xc4};
    queue(ch, J2534_ISO9141, 0x200, pid00, sizeof(pid00));
    TEST_ASSERT_EQUAL_INT(0, call());
    hds_init(ch, 0, true);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_SUCCESS, res.message.rx_status);
    queue(ch, J2534_ISO9141, 0x200, pid00, sizeof(pid00));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_fail_next_send(FAKE_BUS_KLINE);
    hds_init(ch, 0, true);
    TEST_ASSERT_EQUAL_INT(J2534_TIMEOUT, call());
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_FAILED, res.message.rx_status);
}

TEST(j2534_native_filter_still_rejects_hds_legacy_tx_flags) {
    uint32_t ch = hds_channel(false);
    command(opendiag_Request_start_filter_tag);
    req.command.start_filter =
        (opendiag_Filter){.channel = ch, .type = 1, .flags = 0x200};
    req.command.start_filter.mask.size = req.command.start_filter.pattern.size =
        1;
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
}

static j2534_repeat_setup_t repeat_setup(uint32_t protocol,
                                         uint32_t condition) {
    j2534_repeat_setup_t setup = {
        .interval_ms = 20,
        .condition = condition,
        .message = {.protocol = protocol,
                    .len = 5,
                    .data = {0x68, 0x6a, 0xf1, 1, 0}},
        .mask = {.protocol = protocol, .len = 1, .data = {0xff}},
        .pattern = {.protocol = protocol, .len = 1, .data = {0x48}},
    };
    return setup;
}

static uint32_t start_repeat(uint32_t channel, j2534_repeat_setup_t *setup) {
    uint32_t id = 0;
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_start(channel, setup, &id));
    TEST_ASSERT_TRUE(id != 0);
    return id;
}

static bool repeat_active(uint32_t channel, uint32_t id) {
    bool active = false;
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_query(channel, id, &active));
    return active;
}

TEST(repeat_matching_precedes_filters_and_preserves_terminating_response) {
    uint32_t channel = hds_channel(true);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = start_repeat(channel, &setup);
    j2534_core_poll();
    filter(channel, 1, 0, 0);
    fake_clock_advance_ms(1);
    const uint8_t data[] = {0x48, 0x6b, 0x10, 0x41, 0, 4};
    fake_bus_stage_stale(FAKE_BUS_KLINE, data, sizeof(data));
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, id));
    TEST_ASSERT_EQUAL_INT(0, read_message(channel));
    TEST_ASSERT_EQUAL_INT(sizeof(data) - 1, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM(data, res.message.data.bytes, sizeof(data) - 1);
    fake_clock_advance_ms(100);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, id));
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID, j2534_core_repeat_stop(channel, id));
}

TEST(repeat_works_without_pass_filters_and_while_stops_in_silence) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 1);
    uint32_t id = start_repeat(channel, &setup);
    j2534_core_poll();
    fake_clock_advance_ms(21);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, id));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
    setup.condition = 0;
    id = start_repeat(channel, &setup);
    j2534_core_poll();
    fake_clock_advance_ms(1);
    const uint8_t response[] = {0x48, 0x6b, 0x10, 0xc3};
    fake_bus_stage_stale(FAKE_BUS_KLINE, response, sizeof(response));
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, id));
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(channel));
}

TEST(repeat_uses_end_relative_interval_and_copies_caller_storage) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    start_repeat(channel, &setup);
    memset(&setup, 0, sizeof(setup));
    fake_bus_send_duration(FAKE_BUS_KLINE, 7);
    j2534_core_poll();
    fake_clock_advance_ms(19);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
    fake_clock_advance_ms(2);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_bus_sent_count(FAKE_BUS_KLINE));
    size_t len = 0;
    const uint8_t *sent = fake_bus_sent(FAKE_BUS_KLINE, 1, &len);
    TEST_ASSERT_EQUAL_INT(0x68, sent[0]);
    TEST_ASSERT_EQUAL_INT(5, len);
}

TEST(repeat_ignores_tx_loopback_for_both_conditions) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t until = start_repeat(channel, &setup);
    setup.condition = 1;
    uint32_t while_id = start_repeat(channel, &setup);
    const uint8_t response[] = {0x48, 0x6b, 0x10, 0xc3};
    fake_bus_stage_response_status(FAKE_BUS_KLINE, response, sizeof(response),
                                   1, BUS_RX_TX_MSG_TYPE);
    j2534_core_poll();
    fake_clock_advance_ms(1);
    j2534_core_poll();
    TEST_ASSERT_TRUE(repeat_active(channel, until));
    fake_clock_advance_ms(21);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, while_id));
}

TEST(pending_repeat_can_be_stopped_without_transmitting) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = start_repeat(channel, &setup);
    fake_bus_tx_hold(true);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, id));
    fake_bus_tx_hold(false);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(response_cancels_a_repeat_waiting_for_bus_admission) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = start_repeat(channel, &setup);
    fake_bus_tx_hold(true);
    j2534_core_poll();
    fake_clock_advance_ms(1);
    const uint8_t response[] = {0x48, 0x6b, 0x10, 0xc3};
    fake_bus_stage_stale(FAKE_BUS_KLINE, response, sizeof(response));
    /* Let admission race the core's next receive drain. */
    fake_bus_tx_hold(false);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, id));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(repeat_prioritizes_due_jobs_over_ordinary_messages_without_starvation) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t first = start_repeat(channel, &setup);
    setup.message.data[4] = 1;
    uint32_t second = start_repeat(channel, &setup);
    const uint8_t ordinary[] = {0x68, 0x6a, 0xf1, 1, 2};
    queue(channel, J2534_ISO9141, 0, ordinary, sizeof(ordinary));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_send_duration(FAKE_BUS_KLINE, 25);
    j2534_core_poll();
    j2534_core_poll();
    size_t len;
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent(FAKE_BUS_KLINE, 0, &len)[4]);
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent(FAKE_BUS_KLINE, 1, &len)[4]);
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, first));
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, second));
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_bus_sent(FAKE_BUS_KLINE, 2, &len)[4]);
}

TEST(clear_periodics_does_not_clear_repeat_ids_or_pending_repeat) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = start_repeat(channel, &setup);
    fake_bus_tx_hold(true);
    j2534_core_poll();
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl =
        (opendiag_Ioctl){.target = channel, .id = BUS_IOCTL_CLEAR_PERIODIC};
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_tx_hold(false);
    j2534_core_poll();
    TEST_ASSERT_TRUE(repeat_active(channel, id));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(repeat_can_and_isotp_use_independent_jobs_and_native_framing) {
    uint32_t physical = connect_can(0);
    uint32_t channel = logical(physical, false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISOTP, 0);
    memcpy(setup.message.data, "\0\0\x07\xe0\x01", 5);
    setup.mask.data[0] = 0;
    setup.pattern.data[0] = 0;
    uint32_t logical_id = start_repeat(channel, &setup);
    setup.message.protocol = setup.mask.protocol = setup.pattern.protocol =
        J2534_CAN;
    uint32_t physical_id = start_repeat(physical, &setup);
    j2534_core_poll();
    j2534_core_poll();
    const struct can_frame *frame = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(frame);
    TEST_ASSERT_EQUAL_INT(0x7e0, frame->id);
    TEST_ASSERT_EQUAL_INT(1, frame->data[0]);
    fake_clock_advance_ms(1);
    const uint8_t response[] = {1, 0x41};
    fake_can_stage_stale(0x7e8, sizeof(response), response);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, logical_id));
    TEST_ASSERT_FALSE(repeat_active(physical, physical_id));
}

TEST(repeat_validates_shapes_and_retains_quota_until_stop) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = 0;
    setup.interval_ms = 4;
    TEST_ASSERT_EQUAL_INT(J2534_INTERVAL,
                          j2534_core_repeat_start(channel, &setup, &id));
    setup.interval_ms = 20;
    setup.condition = 2;
    TEST_ASSERT_EQUAL_INT(J2534_VALUE,
                          j2534_core_repeat_start(channel, &setup, &id));
    setup.condition = 0;
    setup.pattern.len = 2;
    TEST_ASSERT_EQUAL_INT(J2534_MSG,
                          j2534_core_repeat_start(channel, &setup, &id));
    setup.pattern.len = 1;
    for (unsigned i = 0; i < J2534_REPEAT_MAX; i++)
        id = start_repeat(channel, &setup);
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT,
                          j2534_core_repeat_start(channel, &setup, &id));
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, id));
    start_repeat(channel, &setup);
}

TEST(isotp_accepts_flow_control_after_tx_end_before_worker_returns) {
    uint32_t physical = connect_can(0);
    uint32_t channel = logical(physical, false);
    const uint8_t request[] = {0, 0, 7, 0xe0, 1, 2, 3, 4, 5, 6, 7, 8};
    queue(channel, J2534_ISOTP, J2534_PAD, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_tx_hold_completion(true);
    j2534_core_poll();
    const uint8_t flow[] = {0x30, 0, 0};
    fake_can_stage_stale(0x7e8, sizeof(flow), flow);
    j2534_core_poll();
    fake_bus_tx_hold_completion(false);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x21, fake_can_sent(1)->data[0]);
}

TEST(while_response_window_is_preserved_across_delayed_worker_completion) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 1);
    uint32_t id = start_repeat(channel, &setup);
    fake_bus_tx_hold_completion(true);
    j2534_core_poll();
    fake_clock_advance_ms(10);
    const uint8_t response[] = {0x48, 0x6b, 0x10, 0xc3};
    fake_bus_stage_stale(FAKE_BUS_KLINE, response, sizeof(response));
    j2534_core_poll();
    fake_clock_advance_ms(15);
    fake_bus_stage_stale(FAKE_BUS_KLINE, response, sizeof(response));
    j2534_core_poll();
    TEST_ASSERT_TRUE(repeat_active(channel, id));
    fake_bus_tx_hold_completion(false);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(stopped_repeat_completion_cannot_change_a_new_job_in_the_reused_slot) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t old_id = start_repeat(channel, &setup);
    fake_bus_tx_hold_completion(true);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, old_id));
    uint32_t new_id = start_repeat(channel, &setup);
    TEST_ASSERT_TRUE(old_id != new_id);
    fake_bus_tx_hold_completion(false);
    j2534_core_poll();
    TEST_ASSERT_TRUE(repeat_active(channel, new_id));
    TEST_ASSERT_EQUAL_INT(2, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(clear_tx_keeps_a_late_completion_from_clearing_new_ordinary_data) {
    uint32_t channel = hds_channel(false);
    const uint8_t first[] = {0x68, 0x6a, 0xf1, 1, 0};
    queue(channel, J2534_ISO9141, 0, first, sizeof(first));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_tx_hold_completion(true);
    j2534_core_poll();
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl =
        (opendiag_Ioctl){.target = channel, .id = BUS_IOCTL_CLEAR_TX_QUEUE};
    TEST_ASSERT_EQUAL_INT(0, call());
    const uint8_t second[] = {0x68, 0x6a, 0xf1, 1, 1};
    queue(channel, J2534_ISO9141, 0, second, sizeof(second));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_tx_hold_completion(false);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_bus_sent_count(FAKE_BUS_KLINE));
    size_t len;
    TEST_ASSERT_EQUAL_MEM(second, fake_bus_sent(FAKE_BUS_KLINE, 1, &len),
                          sizeof(second));
}

TEST(fast_init_joins_an_already_submitted_packet_without_sending_it_twice) {
    uint32_t channel = hds_channel(false);
    const uint8_t data[] = {0x68, 0x6a, 0xf1, 1, 0};
    queue(channel, J2534_ISO9141, 0, data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_tx_hold_completion(true);
    j2534_core_poll();
    fake_bus_tx_hold_completion(false);
    hds_init(channel, 0, true);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
}

TEST(clear_rx_still_applies_repeat_conditions_to_removed_responses) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = start_repeat(channel, &setup);
    j2534_core_poll();
    fake_clock_advance_ms(1);
    const uint8_t data[] = {0x48, 0x6b, 0x10, 0xc3};
    fake_bus_stage_stale(FAKE_BUS_KLINE, data, sizeof(data));
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl =
        (opendiag_Ioctl){.target = channel, .id = BUS_IOCTL_CLEAR_RX_QUEUE};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_FALSE(repeat_active(channel, id));
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(channel));
}

TEST(rx_loss_and_tx_failure_stop_repeats_without_releasing_ids) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t failed = start_repeat(channel, &setup);
    fake_bus_fail_next_send(FAKE_BUS_KLINE);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, failed));
    uint32_t lost = start_repeat(channel, &setup);
    const uint8_t data[] = {0x49, 0x6b, 0x10, 0xc4};
    fake_bus_stage_response_status(FAKE_BUS_KLINE, data, sizeof(data), 1,
                                   BUS_RX_BUFFER_OVERFLOW);
    j2534_core_poll();
    fake_clock_advance_ms(1);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, lost));
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, failed));
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, lost));
}

TEST(repeat_j1850_uses_the_same_scheduler_and_receive_conditions) {
    for (unsigned protocol = J2534_VPW; protocol <= J2534_PWM; protocol++) {
        command(opendiag_Request_connect_tag);
        req.command.connect = (opendiag_Connect){
            .device = dev,
            .protocol = protocol,
            .baudrate = protocol == J2534_PWM ? 41600 : 10400,
            .connector = 1,
            .pins_count = protocol == J2534_PWM ? 2 : 1,
            .pins = {2, 10},
        };
        TEST_ASSERT_EQUAL_INT(0, call());
        uint32_t channel = res.id;
        j2534_repeat_setup_t setup = repeat_setup(protocol, 0);
        uint32_t id = start_repeat(channel, &setup);
        j2534_core_poll();
        fake_clock_advance_ms(1);
        const uint8_t response[] = {0x48, 0x6b, 0x10, 0xc3};
        fake_bus_id_t bus =
            protocol == J2534_PWM ? FAKE_BUS_J1850_PWM : FAKE_BUS_J1850_VPW;
        fake_bus_stage_stale(bus, response, sizeof(response));
        j2534_core_poll();
        TEST_ASSERT_FALSE(repeat_active(channel, id));
        TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(bus));
        command(opendiag_Request_disconnect_tag);
        req.command.disconnect.id = channel;
        TEST_ASSERT_EQUAL_INT(0, call());
    }
}

TEST(repeat_can_29bit_masks_are_bit_patterns_not_transmit_identifiers) {
    uint32_t channel = connect_can(J2534_29BIT);
    j2534_repeat_setup_t setup = repeat_setup(J2534_CAN, 0);
    setup.message.flags = setup.mask.flags = setup.pattern.flags = J2534_29BIT;
    memcpy(setup.message.data, "\x18\xdb\x33\xf1\x01", 5);
    setup.mask.len = setup.pattern.len = 4;
    memset(setup.mask.data, 0xff, 4);
    memcpy(setup.pattern.data, "\x18\xda\xf1\x10", 4);
    uint32_t id = start_repeat(channel, &setup);
    j2534_core_poll();
    fake_clock_advance_ms(1);
    const uint8_t response[] = {0x41};
    fake_can_stage_stale(CAN_EFF_FLAG | 0x18daf110, sizeof(response), response);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, id));
}

TEST(
    repeat_isotp_extended_address_uses_single_frame_without_mutating_channel_tx) {
    uint32_t physical = connect_can(0);
    uint32_t channel = logical(physical, true);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISOTP, 0);
    setup.message.flags = J2534_ADDR | J2534_PAD;
    setup.mask.flags = setup.pattern.flags = J2534_ADDR;
    setup.message.len = 6;
    memcpy(setup.message.data, "\0\0\x07\xe0\x10\x01", 6);
    setup.mask.data[0] = setup.pattern.data[0] = 0;
    uint32_t id = start_repeat(channel, &setup);
    j2534_core_poll();
    const struct can_frame *frame = fake_can_sent(0);
    TEST_ASSERT_EQUAL_INT(0x10, frame->data[0]);
    TEST_ASSERT_EQUAL_INT(1, frame->data[1]);
    fake_clock_advance_ms(1);
    const uint8_t response[] = {0xf1, 1, 0x41};
    fake_can_stage_stale(0x7e8, sizeof(response), response);
    j2534_core_poll();
    TEST_ASSERT_FALSE(repeat_active(channel, id));
}

/* ------------------------------------------------------------------ *
 * Second-half coverage: receive status events, filters, scheduling,
 * connect and request error paths.
 * ------------------------------------------------------------------ */

static uint32_t connect_with(const opendiag_Connect *c) {
    command(opendiag_Request_connect_tag);
    req.command.connect = *c;
    return call();
}

static uint32_t connect_vpw(void) {
    opendiag_Connect c = {.device = dev,
                          .protocol = J2534_VPW,
                          .baudrate = 10400,
                          .connector = 1,
                          .pins_count = 1,
                          .pins = {2}};
    TEST_ASSERT_EQUAL_INT(0, connect_with(&c));
    return res.id;
}

static uint32_t logical_with_ids(uint32_t physical, uint8_t local_id,
                                 uint8_t remote_id) {
    command(opendiag_Request_logical_connect_tag);
    opendiag_LogicalConnect *c = &req.command.logical_connect;
    c->physical = physical;
    c->protocol = J2534_ISOTP;
    c->local_address.size = c->remote_address.size = 4;
    memcpy(c->local_address.bytes, "\0\0\x07\xe8", 4);
    memcpy(c->remote_address.bytes, "\0\0\x07\xe0", 4);
    c->local_address.bytes[3] = local_id;
    c->remote_address.bytes[3] = remote_id;
    TEST_ASSERT_EQUAL_INT(0, call());
    return res.id;
}

static uint32_t logical_request(uint32_t physical, uint32_t protocol,
                                uint32_t flags, uint8_t local_id,
                                uint32_t local_size) {
    command(opendiag_Request_logical_connect_tag);
    opendiag_LogicalConnect *c = &req.command.logical_connect;
    c->physical = physical;
    c->protocol = protocol;
    c->flags = flags;
    c->local_address.size = local_size;
    c->remote_address.size = 4;
    memcpy(c->local_address.bytes, "\0\0\x07\xe8", 4);
    memcpy(c->remote_address.bytes, "\0\0\x07\xe0", 4);
    c->local_address.bytes[3] = local_id;
    return call();
}

static uint32_t start_filter_raw(uint32_t ch, uint32_t type, uint32_t flags,
                                 size_t mask_len, size_t pattern_len) {
    command(opendiag_Request_start_filter_tag);
    req.command.start_filter =
        (opendiag_Filter){.channel = ch, .type = type, .flags = flags};
    req.command.start_filter.mask.size = mask_len;
    req.command.start_filter.pattern.size = pattern_len;
    return call();
}

static uint32_t start_periodic_can(uint32_t ch, uint32_t interval_ms,
                                   uint32_t protocol) {
    command(opendiag_Request_start_periodic_tag);
    opendiag_Periodic *p = &req.command.start_periodic;
    p->channel = ch;
    p->interval_ms = interval_ms;
    p->has_message = true;
    p->message.protocol = protocol;
    p->message.data.size = 5;
    memcpy(p->message.data.bytes, "\0\0\x07\xdf\x55", 5);
    return call();
}

static void kline_stage(uint32_t status, const uint8_t *data, size_t len) {
    fake_bus_stage_response_status(FAKE_BUS_KLINE, data, len, 0, status);
}

/* Queues a request, then polls twice: the first sends it and makes staged
 * replies live, the second receives them. */
static void kline_exchange(uint32_t ch) {
    static const uint8_t request[] = {0x68, 0x6a, 0xf1, 1, 0};
    queue(ch, J2534_ISO9141, 0, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    j2534_core_poll();
}

static void expect_tx_success(uint32_t ch) {
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_SUCCESS, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(42, res.message.handle);
}

TEST(kline_link_down_fails_queued_tx_reports_error_and_blocks_transmit) {
    uint32_t ch = hds_channel(false);
    static const uint8_t request[] = {0x68, 0x6a, 0xf1, 1, 0};
    static const uint8_t frame[] = {0x48, 0x6b, 0x10, 0xc3, 0x00};
    queue(ch, J2534_ISO9141, 0, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_stage_response_status(FAKE_BUS_KLINE, frame, sizeof(frame), 0,
                                   BUS_RX_LINK_DOWN);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
    queue(ch, J2534_ISO9141, 0, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_FAILED, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_ERROR, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(4, res.message.data.size);
    TEST_ASSERT_EQUAL_INT(1, res.message.data.bytes[0]);
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    queue(ch, J2534_ISO9141, 0, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(kline_break_and_start_of_message_are_delivered_as_events_without_filters) {
    uint32_t ch = hds_channel(false);
    static const uint8_t data[] = {0x48, 0x6b, 0x10, 0xc3, 0x00};
    kline_stage(BUS_RX_BREAK, data, sizeof(data));
    kline_exchange(ch);
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_BREAK, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(0, res.message.data.size);
    kline_stage(BUS_RX_START_OF_MSG, data, sizeof(data));
    kline_exchange(ch);
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_START, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(0, res.message.data.size);
}

TEST(kline_overflow_of_a_blocked_frame_is_reported_once_by_read) {
    uint32_t ch = hds_channel(false);
    static const uint8_t data[] = {0x48, 0x6b, 0x10, 0xc3, 0x00};
    kline_stage(BUS_RX_BUFFER_OVERFLOW, data, sizeof(data));
    kline_exchange(ch);
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(J2534_OVERFLOW, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
}

TEST(kline_checksum_failures_and_checksum_only_frames_are_dropped) {
    uint32_t ch = hds_channel(true);
    filter(ch, 1, 0, 0);
    static const uint8_t bad[] = {0x48, 0x6b, 0x10, 0xc3, 0x00};
    static const uint8_t checksum_only[] = {0x55};
    static const uint8_t good[] = {0x48, 0x6b, 0x10, 0xc3, 0xaa};
    kline_stage(BUS_RX_BAD_CHECKSUM, bad, sizeof(bad));
    kline_exchange(ch);
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    kline_stage(0, checksum_only, sizeof(checksum_only));
    kline_exchange(ch);
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    kline_stage(0, good, sizeof(good));
    kline_exchange(ch);
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(4, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM(good, res.message.data.bytes, 4);
}

TEST(can_remote_oversized_and_extended_frames_are_not_delivered_to_api) {
    uint32_t ch = connect_can(0);
    filter(ch, 1, 0, 0);
    static const uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    static const uint8_t want[12] = {0, 0, 7, 0xe8, 1, 2, 3, 4, 5, 6, 7, 8};
    fake_can_stage_stale(0x7e8 | CAN_RTR_FLAG, 0, d);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    fake_can_stage_stale(0x7e8, 9, d);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    fake_can_stage_stale(CAN_EFF_FLAG | 0x18daf110u, 8, d);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(J2534_EMPTY, read_message(ch));
    fake_can_stage_stale(0x7e8, 8, d);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(12, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM(want, res.message.data.bytes, 12);
}

TEST(flow_control_waits_for_bus_admission_before_the_next_segment) {
    uint32_t ch = logical(connect_can(0), false);
    static const uint8_t first[] = {0x10, 10, 1, 2, 3, 4, 5, 6};
    fake_bus_tx_hold(true);
    fake_can_stage_stale(0x7e8, 8, first);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
    fake_bus_tx_hold(false);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x7e0, fake_can_sent(0)->id);
    TEST_ASSERT_EQUAL_INT(0x30, fake_can_sent(0)->data[0]);
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_START, res.message.rx_status);
}

TEST(stopped_repeat_with_a_held_transmit_never_reaches_the_bus) {
    uint32_t channel = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    uint32_t id = start_repeat(channel, &setup);
    fake_bus_tx_hold(true);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_stop(channel, id));
    j2534_core_poll();
    fake_bus_tx_hold(false);
    j2534_core_poll();
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID, j2534_core_repeat_stop(channel, id));
}

TEST(logical_repeats_are_sent_in_due_order_not_slot_order) {
    uint32_t physical = connect_can(0);
    uint32_t older_slot = logical_with_ids(physical, 0xe8, 0xe0);
    uint32_t newer_slot = logical_with_ids(physical, 0xe9, 0xe1);
    j2534_repeat_setup_t first = repeat_setup(J2534_ISOTP, 0);
    j2534_repeat_setup_t second = repeat_setup(J2534_ISOTP, 0);
    memcpy(first.message.data, "\0\0\x07\xe0\x01", 5);
    memcpy(second.message.data, "\0\0\x07\xe1\x01", 5);
    /* The job in the higher slot is started first, so it is due first even
     * though the lower slot is scanned first. */
    start_repeat(newer_slot, &second);
    start_repeat(older_slot, &first);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x7e1, fake_can_sent(0)->id);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    TEST_ASSERT_EQUAL_INT(0x7e0, fake_can_sent(1)->id);
}

TEST(periodic_start_checks_message_interval_protocol_and_quota) {
    uint32_t ch = connect_can(0);
    command(opendiag_Request_start_periodic_tag);
    opendiag_Periodic *p = &req.command.start_periodic;
    p->channel = ch;
    p->interval_ms = 10;
    p->has_message = false;
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
    TEST_ASSERT_EQUAL_INT(J2534_INTERVAL, start_periodic_can(ch, 4, J2534_CAN));
    TEST_ASSERT_EQUAL_INT(J2534_INTERVAL,
                          start_periodic_can(ch, 65536, J2534_CAN));
    TEST_ASSERT_EQUAL_INT(J2534_MSG_PROTOCOL,
                          start_periodic_can(ch, 10, J2534_ISO9141));
    uint32_t ids[J2534_PERIODIC_MAX];
    for (unsigned i = 0; i < J2534_PERIODIC_MAX; i++) {
        TEST_ASSERT_EQUAL_INT(0, start_periodic_can(ch, 10, J2534_CAN));
        ids[i] = res.id;
    }
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT, start_periodic_can(ch, 10, J2534_CAN));
    command(opendiag_Request_stop_periodic_tag);
    req.command.stop_periodic = (opendiag_Object){ch, ids[0]};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(0, start_periodic_can(ch, 10, J2534_CAN));
}

TEST(queue_without_a_message_is_rejected_before_any_transmit) {
    uint32_t ch = connect_can(0);
    command(opendiag_Request_queue_tag);
    req.command.queue.channel = ch;
    req.command.queue.has_message = false;
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(filter_table_limits_validate_types_shapes_and_stop_by_id) {
    uint32_t p = connect_can(0);
    uint32_t lg = logical(p, false);
    TEST_ASSERT_EQUAL_INT(J2534_FILTER_TYPE, start_filter_raw(lg, 1, 0, 1, 1));
    TEST_ASSERT_EQUAL_INT(J2534_FILTER_TYPE, start_filter_raw(p, 3, 0, 1, 1));
    TEST_ASSERT_EQUAL_INT(J2534_MSG, start_filter_raw(p, 1, 0, 0, 0));
    TEST_ASSERT_EQUAL_INT(J2534_MSG, start_filter_raw(p, 1, 0, 1, 2));
    TEST_ASSERT_EQUAL_INT(J2534_MSG, start_filter_raw(p, 1, J2534_ADDR, 1, 1));
    TEST_ASSERT_EQUAL_INT(J2534_MSG, start_filter_raw(p, 1, J2534_29BIT, 1, 1));
    uint32_t ids[J2534_FILTER_MAX];
    for (unsigned i = 0; i < J2534_FILTER_MAX; i++) {
        TEST_ASSERT_EQUAL_INT(0, start_filter_raw(p, 1, 0, 1, 1));
        ids[i] = res.id;
    }
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT, start_filter_raw(p, 1, 0, 1, 1));
    command(opendiag_Request_stop_filter_tag);
    req.command.stop_filter = (opendiag_Object){p, ids[0]};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(J2534_FILTER_ID, call());
    TEST_ASSERT_EQUAL_INT(0, start_filter_raw(p, 1, 0, 1, 1));
}

TEST(repeat_start_rejects_unknown_channel_null_setup_and_protocol_mismatch) {
    uint32_t id = 0;
    uint32_t k = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL,
                          j2534_core_repeat_start(123, &setup, &id));
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, j2534_core_repeat_start(k, NULL, &id));
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, j2534_core_repeat_start(k, &setup, NULL));
    setup.mask.protocol = J2534_CAN;
    TEST_ASSERT_EQUAL_INT(J2534_MSG_PROTOCOL,
                          j2534_core_repeat_start(k, &setup, &id));
    setup = repeat_setup(J2534_ISO9141, 0);
    setup.message.protocol = J2534_CAN;
    TEST_ASSERT_EQUAL_INT(J2534_MSG_PROTOCOL,
                          j2534_core_repeat_start(k, &setup, &id));
    TEST_ASSERT_EQUAL_INT(0, id);
}

TEST(repeat_start_rejects_flags_the_channel_cannot_match) {
    uint32_t id = 0;
    uint32_t k = hds_channel(false);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISO9141, 0);
    setup.mask.flags = setup.pattern.flags = J2534_29BIT;
    TEST_ASSERT_EQUAL_INT(J2534_FLAGS, j2534_core_repeat_start(k, &setup, &id));
    uint32_t can = connect_can(0);
    setup = repeat_setup(J2534_CAN, 0);
    memcpy(setup.message.data, "\0\0\x07\xdf\x55", 5);
    setup.mask.flags = setup.pattern.flags = J2534_29BIT;
    TEST_ASSERT_EQUAL_INT(J2534_FLAGS,
                          j2534_core_repeat_start(can, &setup, &id));
    uint32_t lg = logical(can, false);
    setup = repeat_setup(J2534_ISOTP, 0);
    memcpy(setup.message.data, "\0\0\x07\xe0\x01", 5);
    setup.mask.flags = setup.pattern.flags = J2534_ADDR;
    TEST_ASSERT_EQUAL_INT(J2534_FLAGS, j2534_core_repeat_start(lg, &setup, &id));
    TEST_ASSERT_EQUAL_INT(0, id);
}

TEST(repeat_start_limits_mask_length_to_the_protocol_frame) {
    uint32_t vpw = connect_vpw();
    uint32_t id = 0;
    j2534_repeat_setup_t setup = repeat_setup(J2534_VPW, 0);
    setup.mask.len = setup.pattern.len = 12;
    TEST_ASSERT_EQUAL_INT(J2534_MSG, j2534_core_repeat_start(vpw, &setup, &id));
    setup.mask.len = setup.pattern.len = 11;
    TEST_ASSERT_EQUAL_INT(0, j2534_core_repeat_start(vpw, &setup, &id));
    TEST_ASSERT_TRUE(id != 0);
}

TEST(repeat_query_and_stop_check_channel_argument_and_job_kind) {
    bool active = true;
    uint32_t k = hds_channel(false);
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL,
                          j2534_core_repeat_query(123, 1, &active));
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, j2534_core_repeat_query(k, 1, NULL));
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID,
                          j2534_core_repeat_query(k, 999, &active));
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, j2534_core_repeat_stop(123, 1));
    command(opendiag_Request_start_periodic_tag);
    opendiag_Periodic *p = &req.command.start_periodic;
    p->channel = k;
    p->interval_ms = 10;
    p->has_message = true;
    p->message.protocol = J2534_ISO9141;
    p->message.data.size = 5;
    memcpy(p->message.data.bytes, "\x68\x6a\xf1\x01\x00", 5);
    TEST_ASSERT_EQUAL_INT(0, call());
    uint32_t periodic = res.id;
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID,
                          j2534_core_repeat_query(k, periodic, &active));
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID, j2534_core_repeat_stop(k, periodic));
    command(opendiag_Request_stop_periodic_tag);
    req.command.stop_periodic = (opendiag_Object){k, periodic};
    TEST_ASSERT_EQUAL_INT(0, call());
}

TEST(repeat_wire_requests_start_query_and_stop_by_id) {
    uint32_t ch = hds_channel(false);
    command(opendiag_Request_start_repeat_tag);
    opendiag_Repeat *r = &req.command.start_repeat;
    r->channel = ch;
    r->interval_ms = 20;
    r->has_message = true;
    r->message.protocol = J2534_ISO9141;
    r->message.data.size = 5;
    memcpy(r->message.data.bytes, "\x68\x6a\xf1\x01\x00", 5);
    r->has_pattern = true;
    r->pattern.protocol = J2534_ISO9141;
    r->pattern.data.size = 1;
    r->pattern.data.bytes[0] = 0x48;
    TEST_ASSERT_EQUAL_INT(J2534_MSG, call());
    r->has_mask = true;
    r->mask.protocol = J2534_ISO9141;
    r->mask.data.size = 1;
    r->mask.data.bytes[0] = 0xff;
    TEST_ASSERT_EQUAL_INT(0, call());
    uint32_t id = res.id;
    TEST_ASSERT_TRUE(id != 0);
    command(opendiag_Request_query_repeat_tag);
    req.command.query_repeat = (opendiag_Object){ch, id};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_TRUE(res.repeat_active);
    command(opendiag_Request_stop_repeat_tag);
    req.command.stop_repeat = (opendiag_Object){ch, id};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID, call());
    command(opendiag_Request_query_repeat_tag);
    req.command.query_repeat = (opendiag_Object){ch, id};
    TEST_ASSERT_EQUAL_INT(J2534_MSG_ID, call());
}

TEST(logical_disconnect_accepts_only_logical_handles) {
    uint32_t p = connect_can(0);
    uint32_t ch = logical(p, false);
    command(opendiag_Request_logical_disconnect_tag);
    req.command.logical_disconnect.id = p;
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, call());
    TEST_ASSERT_TRUE(fake_can_is_up());
    req.command.logical_disconnect.id = ch;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, read_message(ch));
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(select_reports_only_readable_live_channels_and_rejects_bad_requests) {
    uint32_t can = connect_can(0);
    uint32_t k = hds_channel(false);
    filter(can, 1, 0, 0);
    command(opendiag_Request_select_tag);
    req.command.select = (opendiag_Select){.type = 2, .channels_count = 1,
                                           .channels = {can}};
    TEST_ASSERT_EQUAL_INT(J2534_SELECT, call());
    req.command.select.type = 1;
    req.command.select.channels[0] = 123;
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, call());
    static const uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    fake_can_stage_stale(0x7e8, 8, d);
    j2534_core_poll();
    req.command.select = (opendiag_Select){.type = 1, .channels_count = 2,
                                           .channels = {k, can}};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(1, res.channels_count);
    TEST_ASSERT_EQUAL_INT(can, res.channels[0]);
}

TEST(connect_rejects_bad_device_connector_baud_protocol_and_flags) {
    opendiag_Connect c = {.device = dev + 1,
                          .protocol = J2534_CAN,
                          .baudrate = 500000,
                          .connector = 1,
                          .pins_count = 2,
                          .pins = {6, 14}};
    TEST_ASSERT_EQUAL_INT(J2534_DEVICE, connect_with(&c));
    c.device = dev;
    c.connector = 2;
    TEST_ASSERT_EQUAL_INT(J2534_PIN, connect_with(&c));
    c.connector = 1;
    c.baudrate = 100000;
    TEST_ASSERT_EQUAL_INT(J2534_BAUD, connect_with(&c));
    c.baudrate = 500000;
    c.flags = J2534_CHECKSUM_DISABLED;
    TEST_ASSERT_EQUAL_INT(J2534_FLAGS, connect_with(&c));
    c = (opendiag_Connect){.device = dev,
                           .protocol = J2534_PWM,
                           .baudrate = 10400,
                           .connector = 1,
                           .pins_count = 2,
                           .pins = {2, 10}};
    TEST_ASSERT_EQUAL_INT(J2534_BAUD, connect_with(&c));
    c = (opendiag_Connect){.device = dev,
                           .protocol = J2534_VPW,
                           .baudrate = 41600,
                           .connector = 1,
                           .pins_count = 1,
                           .pins = {2}};
    TEST_ASSERT_EQUAL_INT(J2534_BAUD, connect_with(&c));
    c = (opendiag_Connect){.device = dev,
                           .protocol = J2534_ISO9141,
                           .baudrate = 1199,
                           .connector = 1,
                           .pins_count = 2,
                           .pins = {7, 15}};
    TEST_ASSERT_EQUAL_INT(J2534_BAUD, connect_with(&c));
    c.baudrate = 15626;
    TEST_ASSERT_EQUAL_INT(J2534_BAUD, connect_with(&c));
    c = (opendiag_Connect){.device = dev,
                           .protocol = J2534_ISOTP,
                           .baudrate = 500000,
                           .connector = 1,
                           .pins_count = 2,
                           .pins = {6, 14}};
    TEST_ASSERT_EQUAL_INT(J2534_PROTOCOL, connect_with(&c));
    TEST_ASSERT_FALSE(vif_any_pin_active());
    connect_can(0);
}

TEST(connect_refuses_pwm_while_the_j1850_group_is_claimed_by_vpw) {
    connect_can(0);
    hds_channel(false);
    connect_vpw();
    opendiag_Connect pwm = {.device = dev,
                            .protocol = J2534_PWM,
                            .baudrate = 41600,
                            .connector = 1,
                            .pins_count = 2,
                            .pins = {2, 10}};
    /* PWM and VPW share one J1850 resource group, so this is a conflict,
     * not a slot limit: three physical slots equal three bus groups. */
    TEST_ASSERT_EQUAL_INT(J2534_CONFLICT, connect_with(&pwm));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_J1850_PWM));
}

TEST(connect_open_failure_releases_the_claim_and_reports_the_board_code) {
    opendiag_Connect k = {.device = dev,
                          .protocol = J2534_ISO9141,
                          .baudrate = 10400,
                          .connector = 1,
                          .pins_count = 2,
                          .pins = {7, 15}};
    fake_bus_open_result(FAKE_BUS_KLINE, ESP_ERR_NOT_SUPPORTED);
    TEST_ASSERT_EQUAL_INT(J2534_CONFLICT, connect_with(&k));
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_KLINE));
    fake_bus_open_result(FAKE_BUS_KLINE, ESP_OK);
    TEST_ASSERT_EQUAL_INT(0, connect_with(&k));
    TEST_ASSERT_TRUE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_KLINE));
}

TEST(connect_kline_configuration_failure_disconnects_and_reports_value) {
    opendiag_Connect k = {.device = dev,
                          .protocol = J2534_ISO9141,
                          .baudrate = 10400,
                          .connector = 1,
                          .pins_count = 2,
                          .pins = {7, 15}};
    fake_bus_on_kline_only(NULL, BUS_ERR_UNSUPPORTED);
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, connect_with(&k));
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_KLINE));
    fake_bus_on_kline_only(NULL, 0);
    TEST_ASSERT_EQUAL_INT(0, connect_with(&k));
    TEST_ASSERT_TRUE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_KLINE));
}

TEST(logical_connect_rejects_parent_protocol_flags_and_descriptor) {
    uint32_t can = connect_can(0);
    uint32_t k = hds_channel(false);
    TEST_ASSERT_EQUAL_INT(J2534_LOGICAL,
                          logical_request(k, J2534_ISOTP, 0, 0xe8, 4));
    TEST_ASSERT_EQUAL_INT(J2534_PROTOCOL,
                          logical_request(can, J2534_CAN, 0, 0xe8, 4));
    TEST_ASSERT_EQUAL_INT(J2534_FLAGS,
                          logical_request(can, J2534_ISOTP, 2, 0xe8, 4));
    TEST_ASSERT_EQUAL_INT(J2534_DESCRIPTOR,
                          logical_request(can, J2534_ISOTP, 0, 0xe8, 3));
    TEST_ASSERT_EQUAL_INT(0, logical_request(can, J2534_ISOTP, 0, 0xe8, 4));
    TEST_ASSERT_EQUAL_INT(J2534_NOT_UNIQUE,
                          logical_request(can, J2534_ISOTP, 0, 0xe8, 4));
    TEST_ASSERT_EQUAL_INT(0, logical_request(can, J2534_ISOTP, 0, 0xe9, 4));
    TEST_ASSERT_EQUAL_INT(J2534_LIMIT,
                          logical_request(can, J2534_ISOTP, 0, 0xea, 4));
}

TEST(fast_init_rejects_can_channels_and_short_or_missing_replies) {
    uint32_t can = connect_can(0);
    hds_init(can, 1, false);
    TEST_ASSERT_EQUAL_INT(J2534_IOCTL, call());
    uint32_t ch = hds_channel(true);
    static const uint8_t one[] = {0x12};
    fake_bus_kline_init_reply(one, 1, 0);
    hds_init(ch, 1, false);
    TEST_ASSERT_EQUAL_INT(J2534_INIT, call());
}

TEST(fast_init_without_checksum_needs_at_least_one_reply_byte) {
    uint32_t ch = hds_channel(false);
    static const uint8_t none[1] = {0};
    fake_bus_kline_init_reply(none, 0, 0);
    hds_init(ch, 1, false);
    TEST_ASSERT_EQUAL_INT(J2534_INIT, call());
}

TEST(fast_init_waits_for_a_held_transmit_and_reports_timeout) {
    uint32_t ch = hds_channel(false);
    static const uint8_t request[] = {0x68, 0x6a, 0xf1, 1, 0};
    queue(ch, J2534_ISO9141, 0, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    fake_bus_tx_hold_completion(true);
    j2534_core_poll();
    hds_init(ch, 0, true);
    TEST_ASSERT_EQUAL_INT(J2534_TIMEOUT, call());
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
    fake_bus_tx_hold_completion(false);
    j2534_core_poll();
    expect_tx_success(ch);
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(read_voltage_ioctls_check_device_connector_pin_and_data) {
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl = (opendiag_Ioctl){.target = dev + 1, .id = 3,
                                         .connector = 1, .pin = 16};
    TEST_ASSERT_EQUAL_INT(J2534_DEVICE, call());
    req.command.ioctl.target = dev;
    req.command.ioctl.data.size = 1;
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, call());
    req.command.ioctl.data.size = 0;
    req.command.ioctl.config_count = 1;
    TEST_ASSERT_EQUAL_INT(J2534_VALUE, call());
    req.command.ioctl.config_count = 0;
    req.command.ioctl.connector = 0;
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
    req.command.ioctl.connector = 1;
    req.command.ioctl.pin = 15;
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
    req.command.ioctl = (opendiag_Ioctl){.target = dev, .id = 14, .connector = 1};
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
}

TEST(read_vbatt_reports_the_battery_reading) {
    fake_board_set_vbatt_mv(12345);
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl = (opendiag_Ioctl){.target = dev, .id = 3,
                                         .connector = 1, .pin = 16};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(12345, res.millivolts);
    fake_board_set_vbatt_mv(12600);
}

TEST(read_voltage_without_a_reading_fails) {
    fake_board_set_vbatt_mv(-1);
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl = (opendiag_Ioctl){.target = dev, .id = 3,
                                         .connector = 1, .pin = 16};
    TEST_ASSERT_EQUAL_INT(J2534_FAILED, call());
    fake_board_set_vbatt_mv(12600);
}

TEST(programming_voltage_reading_is_zero) {
    command(opendiag_Request_ioctl_tag);
    req.command.ioctl = (opendiag_Ioctl){.target = dev, .id = 14};
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(0, res.millivolts);
}

TEST(programming_voltage_pin_interlocks_and_shell_owned_pins) {
    command(opendiag_Request_voltage_tag);
    req.command.voltage = (opendiag_Voltage){dev, 1, 1, 12000};
    TEST_ASSERT_EQUAL_INT(J2534_PIN, call());
    req.command.voltage.pin = 12;
    TEST_ASSERT_EQUAL_INT(0, call());
    req.command.voltage.pin = 11;
    TEST_ASSERT_EQUAL_INT(J2534_VOLTAGE_IN_USE, call());
    req.command.voltage.pin = 12;
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(0, call());
    req.command.voltage.pin = 11;
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(0, call());
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(VIF_OWNER_SHELL, 13,
                                              VIF_PIN_VOLTAGE, 12000));
    req.command.voltage.pin = 13;
    req.command.voltage.millivolts = 12000;
    TEST_ASSERT_EQUAL_INT(J2534_PIN_IN_USE, call());
    req.command.voltage.millivolts = UINT32_MAX;
    TEST_ASSERT_EQUAL_INT(J2534_PIN_IN_USE, call());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(VIF_OWNER_SHELL, 13,
                                              VIF_PIN_OFF, 0));
}

TEST(version_and_last_error_text_are_bound_to_the_device_and_last_failure) {
    command(opendiag_Request_version_tag);
    req.command.version.id = dev + 1;
    TEST_ASSERT_EQUAL_INT(J2534_DEVICE, call());
    req.command.version.id = dev;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_STRING(
        "OpenDIAG host-test; protobuf wire 3; J2534 05.00 subset", res.text);
    command(opendiag_Request_close_tag);
    req.command.close.id = dev + 1;
    TEST_ASSERT_EQUAL_INT(J2534_DEVICE, call());
    command(opendiag_Request_last_error_tag);
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_STRING("J2534 status 0x1a", res.text);
}

TEST(close_reports_a_failed_bus_release_and_keeps_the_device_for_retry) {
    connect_vpw();
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_FAIL);
    command(opendiag_Request_close_tag);
    req.command.close.id = dev;
    TEST_ASSERT_EQUAL_INT(J2534_FAILED, call());
    command(opendiag_Request_open_tag);
    TEST_ASSERT_EQUAL_INT(J2534_IN_USE, call());
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_OK);
    command(opendiag_Request_close_tag);
    req.command.close.id = dev;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_J1850_VPW));
}

TEST(open_reports_a_failed_release_of_a_claim_left_by_a_stopped_core) {
    connect_vpw();
    j2534_core_stop();
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_FAIL);
    command(opendiag_Request_open_tag);
    TEST_ASSERT_EQUAL_INT(J2534_FAILED, call());
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_OK);
    TEST_ASSERT_EQUAL_INT(0, call());
    dev = res.id;
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_J1850_VPW));
}

TEST(read_on_a_channel_whose_bus_close_failed_reports_channel_error) {
    opendiag_Connect v = {.device = dev,
                          .protocol = J2534_VPW,
                          .baudrate = 10400,
                          .connector = 1,
                          .pins_count = 1,
                          .pins = {2}};
    TEST_ASSERT_EQUAL_INT(0, connect_with(&v));
    uint32_t ch = res.id;
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_FAIL);
    command(opendiag_Request_disconnect_tag);
    req.command.disconnect.id = ch;
    TEST_ASSERT_EQUAL_INT(J2534_FAILED, call());
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, read_message(ch));
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_OK);
    command(opendiag_Request_disconnect_tag);
    req.command.disconnect.id = ch;
    TEST_ASSERT_EQUAL_INT(0, call());
    TEST_ASSERT_EQUAL_INT(J2534_CHANNEL, read_message(ch));
}

TEST(logical_transmit_is_aborted_by_an_overflow_flow_control) {
    uint32_t ch = logical(connect_can(0), false);
    static const uint8_t msg[] = {0, 0, 7, 0xe0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    queue(ch, J2534_ISOTP, J2534_PAD, msg, sizeof(msg));
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    static const uint8_t overflow[] = {0x32, 0, 0, 0, 0, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, overflow);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(ch));
    TEST_ASSERT_EQUAL_INT(J2534_TX_FAILED, res.message.rx_status);
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

static uint32_t legacy_logical(uint32_t physical, uint8_t local_id,
                               uint8_t remote_id) {
    command(opendiag_Request_logical_connect_tag);
    opendiag_LogicalConnect *c = &req.command.logical_connect;
    c->physical = physical;
    c->protocol = J2534_ISOTP;
    c->legacy_channel = true;
    c->local_address.size = c->remote_address.size = 4;
    memcpy(c->local_address.bytes, "\0\0\x07\xe8", 4);
    memcpy(c->remote_address.bytes, "\0\0\x07\xe0", 4);
    c->local_address.bytes[3] = local_id;
    c->remote_address.bytes[3] = remote_id;
    TEST_ASSERT_EQUAL_INT(0, call());
    return res.id;
}

TEST(legacy_reception_on_one_logical_also_feeds_its_peers_receives) {
    uint32_t physical = connect_can(0);
    uint32_t reader = legacy_logical(physical, 0xe8, 0xe0);
    uint32_t peer = legacy_logical(physical, 0xe9, 0xe1);
    j2534_repeat_setup_t setup = repeat_setup(J2534_ISOTP, 0);
    memcpy(setup.message.data, "\0\0\x07\xe1\x01", 5);
    setup.mask.len = setup.pattern.len = 4;
    memset(setup.mask.data, 0xff, 4);
    memcpy(setup.pattern.data, "\0\0\x07\xe8", 4);
    uint32_t id = start_repeat(peer, &setup);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
    TEST_ASSERT_TRUE(repeat_active(peer, id));
    static const uint8_t response[] = {0x01, 0x41, 0, 0, 0, 0, 0, 0};
    fake_can_stage_stale(0x7e8, 8, response);
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, read_message(reader));
    TEST_ASSERT_EQUAL_INT(5, res.message.data.size);
    TEST_ASSERT_EQUAL_MEM("\0\0\x07\xe8\x41", res.message.data.bytes, 5);
    TEST_ASSERT_FALSE(repeat_active(peer, id));
}

TEST(poll_after_the_device_is_closed_transmits_nothing) {
    uint32_t ch = connect_can(0);
    static const uint8_t request[] = {0, 0, 7, 0xdf};
    queue(ch, J2534_CAN, 0, request, sizeof(request));
    TEST_ASSERT_EQUAL_INT(0, call());
    command(opendiag_Request_close_tag);
    req.command.close.id = dev;
    TEST_ASSERT_EQUAL_INT(0, call());
    j2534_core_poll();
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
}
