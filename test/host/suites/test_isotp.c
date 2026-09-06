/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "isotp.h"
#include "td_test.h"

static isotp_config_t config;
static isotp_rx_t rx;
static isotp_tx_t tx;
static uint8_t payload[4096], received[4096];
static isotp_segment_t segment;
static struct can_frame frame;

void td_setup(void) {
    config = isotp_config_default(0x7e0, 0x7e8);
    for (unsigned i = 0; i < sizeof(payload); i++)
        payload[i] = i ^ (i >> 8);
    memset(received, 0xcc, sizeof(received));
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_init(&rx, &config, received, 4095));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_init(&tx, &config));
}

static int feed(const uint8_t *data, unsigned dlc, uint32_t time) {
    frame = (struct can_frame){.id = 0x7e8, .dlc = dlc};
    memcpy(frame.data, data, dlc);
    return isotp_rx_feed(&rx, &frame, time, &segment);
}

static void first(unsigned length, uint32_t time) {
    uint8_t ff[8] = {0x10 | (length >> 8), length & 0xff, 1, 2, 3, 4, 5, 6};
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(ff, 8, time));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME,
                          isotp_rx_flow_control(&rx, &frame, time));
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_confirm(&rx, true, time));
}

static void start_tx(unsigned length, uint32_t time) {
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_start(&tx, payload, length, time));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, time));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_confirm(&tx, true, time));
}

static int flow(unsigned fs, unsigned bs, unsigned stmin, uint32_t time) {
    frame = (struct can_frame){
        .id = 0x7e8, .dlc = 3, .data = {0x30 | fs, bs, stmin}};
    return isotp_tx_flow_control(&tx, &frame, time);
}

TEST(receive_single_frame_strips_padding) {
    uint8_t sf[] = {3, 0x41, 0, 0x12, 0xaa, 0xaa, 0xaa, 0xaa};
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, feed(sf, 8, 0));
    TEST_ASSERT_TRUE(segment.started && segment.complete);
    TEST_ASSERT_EQUAL_INT(3, segment.length);
    TEST_ASSERT_EQUAL_MEM(sf + 1, received, 3);
    TEST_ASSERT_EQUAL_INT(0xcc, received[3]);
    TEST_ASSERT_FALSE(segment.padding_error);
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, feed(sf, 4, 0));
    TEST_ASSERT_TRUE(segment.padding_error);
}

TEST(invalid_lengths_and_unknown_types_are_ignored) {
    const uint8_t invalid[][8] = {
        {0}, {8}, {0x10, 7}, {0x10, 0, 0, 0, 0x10, 0}, {0x40}, {0x21}, {0x30}};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++)
        TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(invalid[i], 8, 0));
    uint8_t ff[8] = {0x10, 20};
    for (unsigned dlc = 0; dlc < 8; dlc++)
        TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(ff, dlc, 0));
    uint8_t sf[8] = {7};
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(sf, 4, 0));
}

TEST(wrong_address_and_frame_format_do_not_affect_receive_state) {
    first(20, 0);
    frame = (struct can_frame){.id = 0x7e9, .dlc = 8, .data = {0x22}};
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_feed(&rx, &frame, 1, &segment));
    frame.id = 0x7e8 | CAN_EFF_FLAG;
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_feed(&rx, &frame, 1, &segment));
    frame.id = 0x7e8 | CAN_RTR_FLAG;
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_feed(&rx, &frame, 1, &segment));
    frame.id = 0x7e8;
    frame.dlc = 9;
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_feed(&rx, &frame, 1, &segment));
    TEST_ASSERT_EQUAL_INT(6, rx.offset);
}

TEST(wrong_sequence_aborts_and_orphan_frames_are_ignored) {
    first(20, 0);
    uint8_t cf[8] = {0x22};
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_SEQUENCE, feed(cf, 8, 1));
    TEST_ASSERT_EQUAL_INT(ISOTP_RX_IDLE, rx.state);
    cf[0] = 0x21;
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(cf, 8, 2));
}

TEST(short_nonfinal_cf_is_ignored_and_last_cf_is_trimmed) {
    first(16, 0);
    uint8_t cf[8] = {0x21, 7, 8, 9, 10, 11, 12, 13};
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(cf, 7, 1));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(cf, 8, 2));
    cf[0] = 0x22;
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(cf, 3, 3));
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, feed(cf, 4, 4));
    TEST_ASSERT_EQUAL_INT(3, segment.length);
    TEST_ASSERT_EQUAL_INT(13, segment.offset);
    TEST_ASSERT_TRUE(segment.padding_error);
    TEST_ASSERT_EQUAL_INT(0xcc, received[16]);
}

TEST(new_valid_sf_or_ff_replaces_incomplete_reception) {
    first(20, 0);
    uint8_t sf[8] = {0};
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(sf, 8, 1));
    TEST_ASSERT_EQUAL_INT(ISOTP_RX_CF, rx.state);
    sf[0] = 1;
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, feed(sf, 8, 2));
    TEST_ASSERT_TRUE(segment.replaced);
    first(20, 3);
    uint8_t ff[8] = {0x10, 8};
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(ff, 8, 4));
    TEST_ASSERT_TRUE(segment.replaced);
    TEST_ASSERT_EQUAL_INT(8, segment.total);
}

TEST(receiver_capacity_overflow_sends_only_overflow_fc) {
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_init(&rx, &config, received, 8));
    uint8_t ff[8] = {0x10, 9};
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_OVERFLOW, feed(ff, 8, 0));
    TEST_ASSERT_EQUAL_INT(0xcc, received[0]);
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_rx_flow_control(&rx, &frame, 1));
    TEST_ASSERT_EQUAL_INT(0x32, frame.data[0]);
    TEST_ASSERT_EQUAL_INT(0x7e0, frame.id);
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_confirm(&rx, true, 2));
    TEST_ASSERT_EQUAL_INT(ISOTP_RX_IDLE, rx.state);
}

TEST(receiver_block_size_requires_new_fc_only_between_blocks) {
    config.block_size = 1;
    config.stmin = 0xf3;
    config.pad_value = 0xaa;
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_init(&rx, &config, NULL, 4095));
    first(20, 0);
    TEST_ASSERT_EQUAL_MEM("\x30\x01\xf3\xaa\xaa\xaa\xaa\xaa", frame.data, 8);
    uint8_t cf[8] = {0x21};
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(cf, 8, 1));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_rx_flow_control(&rx, &frame, 2));
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, isotp_rx_flow_control(&rx, &frame, 2));
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_confirm(&rx, true, 3));
    cf[0] = 0x22;
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, feed(cf, 8, 4));
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, isotp_rx_flow_control(&rx, &frame, 5));
}

TEST(receiver_times_from_fc_confirmation_and_handles_clock_wrap) {
    first(20, UINT32_MAX - 100);
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_check_timeout(&rx, 999898));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT_CR,
                          isotp_rx_check_timeout(&rx, 999899));
    uint8_t ff[8] = {0x10, 20};
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(ff, 8, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_rx_flow_control(&rx, &frame, 100));
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_confirm(&rx, true, 900000));
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_check_timeout(&rx, 1899999));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT_CR,
                          isotp_rx_check_timeout(&rx, 1900000));
}

TEST(receiver_failed_or_unconfirmed_fc_aborts) {
    uint8_t ff[8] = {0x10, 20};
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(ff, 8, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_rx_flow_control(&rx, &frame, 1));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_SEND, isotp_rx_confirm(&rx, false, 2));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, feed(ff, 8, 3));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT_A,
                          isotp_rx_check_timeout(&rx, 1000003));
}

TEST(functional_addressing_only_allows_single_frames) {
    config.functional = true;
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_init(&rx, &config, received, 4095));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_init(&tx, &config));
    uint8_t ff[8] = {0x10, 20};
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(ff, 8, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_ARGUMENT,
                          isotp_tx_start(&tx, payload, 8, 0));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_start(&tx, payload, 7, 0));
}

TEST(transmit_single_frame_completes_only_after_confirmation) {
    config.pad = false;
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_init(&tx, &config));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_start(&tx, payload, 3, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 0));
    TEST_ASSERT_EQUAL_INT(4, frame.dlc);
    TEST_ASSERT_EQUAL_MEM("\x03\x00\x01\x02", frame.data, 4);
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, isotp_tx_next(&tx, &frame, 10));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_BUSY, isotp_tx_start(&tx, payload, 3, 10));
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, isotp_tx_confirm(&tx, true, 20));
}

TEST(transmit_length_limits) {
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_ARGUMENT,
                          isotp_tx_start(&tx, payload, 0, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_ARGUMENT,
                          isotp_tx_start(&tx, payload, 4096, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_ARGUMENT, isotp_tx_start(&tx, NULL, 1, 0));
    start_tx(4095, 0);
    TEST_ASSERT_EQUAL_INT(0x1f, frame.data[0]);
    TEST_ASSERT_EQUAL_INT(0xff, frame.data[1]);
}

TEST(transmit_stmin_is_measured_from_completion_and_across_blocks) {
    start_tx(40, 0);
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(0, 1, 0xf3, 100));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 100));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_confirm(&tx, true, 500));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(0, 0, 0xf3, 600));
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, isotp_tx_next(&tx, &frame, 799));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 800));
    TEST_ASSERT_EQUAL_INT(0x22, frame.data[0]);
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_confirm(&tx, true, 1500));
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, isotp_tx_next(&tx, &frame, 1799));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 1800));
}

TEST(all_stmin_encodings_use_specified_units_or_reserved_fallback) {
    for (unsigned i = 0; i <= 255; i++) {
        unsigned expected = i <= 0x7f                  ? i * 1000
                            : (i >= 0xf1 && i <= 0xf9) ? (i - 0xf0) * 100
                                                       : 127000;
        TEST_ASSERT_EQUAL_INT(expected, isotp_stmin_us(i));
    }
}

TEST(wait_restarts_bs_timeout_and_cts_resets_wait_counter) {
    config.wait_limit = 1;
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_init(&tx, &config));
    start_tx(40, 0);
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(1, 255, 255, 900000));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_check_timeout(&tx, 1800000));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(0, 1, 0, 1800001));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 1800001));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_confirm(&tx, true, 1800002));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(1, 0, 0, 1800003));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_WAIT_LIMIT, flow(1, 0, 0, 1800004));
}

TEST(flow_control_errors_abort_transmit) {
    start_tx(20, 0);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_OVERFLOW, flow(2, 255, 255, 1));
    start_tx(20, 2);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_FLOW_STATUS, flow(3, 0, 0, 3));
    config.wait_limit = 0;
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_init(&tx, &config));
    start_tx(20, 4);
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_WAIT_LIMIT, flow(1, 0, 0, 5));
}

TEST(short_or_wrong_address_fc_does_not_extend_deadline) {
    start_tx(20, 0);
    frame = (struct can_frame){.id = 0x7e9, .dlc = 3, .data = {0x30}};
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED,
                          isotp_tx_flow_control(&tx, &frame, 800000));
    frame.id = 0x7e8;
    frame.dlc = 2;
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED,
                          isotp_tx_flow_control(&tx, &frame, 900000));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT_BS,
                          isotp_tx_check_timeout(&tx, 1000000));
}

TEST(transmit_failures_and_confirmation_timeout_are_reported) {
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_start(&tx, payload, 3, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 0));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_SEND, isotp_tx_confirm(&tx, false, 1));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_start(&tx, payload, 3, 2));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 2));
    TEST_ASSERT_EQUAL_INT(ISOTP_ERR_TIMEOUT_A,
                          isotp_tx_check_timeout(&tx, 1000002));
}

TEST(normal_fixed_priority_mask_preserves_frame_format_check) {
    config.rx_id = CAN_EFF_FLAG | 0x18daf110;
    config.rx_mask = 0x03ffffff;
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_init(&rx, &config, received, 4095));
    frame = (struct can_frame){
        .id = CAN_EFF_FLAG | 0x0cdaf110, .dlc = 2, .data = {1, 0x55}};
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE,
                          isotp_rx_feed(&rx, &frame, 0, &segment));
    TEST_ASSERT_EQUAL_INT(0x55, received[0]);
}

TEST(two_independent_streams_reassemble_interleaved_ecus) {
    isotp_rx_t other;
    uint8_t buffer[20];
    isotp_config_t c = isotp_config_default(0x7e1, 0x7e9);
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_init(&other, &c, buffer, sizeof(buffer)));
    first(20, 0);
    frame = (struct can_frame){.id = 0x7e9, .dlc = 8, .data = {0x10, 8}};
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME,
                          isotp_rx_feed(&other, &frame, 1, &segment));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME,
                          isotp_rx_flow_control(&other, &frame, 1));
    TEST_ASSERT_EQUAL_INT(0, isotp_rx_confirm(&other, true, 1));
    frame =
        (struct can_frame){.id = 0x7e9, .dlc = 3, .data = {0x21, 0xaa, 0xbb}};
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE,
                          isotp_rx_feed(&other, &frame, 2, &segment));
    TEST_ASSERT_EQUAL_INT(6, rx.offset);
    TEST_ASSERT_EQUAL_MEM("\xaa\xbb", buffer + 6, 2);
}

TEST(roundtrip_all_lengths_both_address_modes_and_padding_modes) {
    for (unsigned extended = 0; extended < 2; extended++) {
        for (unsigned pad = 0; pad < 2; pad++) {
            for (unsigned length = 1; length <= 4095; length++) {
                config.extended_address = extended;
                config.tx_address = 0x10;
                config.rx_address = 0xf1;
                config.pad = pad;
                config.block_size = 17;
                config.stmin = 0xf1;
                isotp_config_t peer = config;
                peer.tx_id = config.rx_id;
                peer.rx_id = config.tx_id;
                peer.tx_address = config.rx_address;
                peer.rx_address = config.tx_address;
                TEST_ASSERT_EQUAL_INT(0, isotp_tx_init(&tx, &config));
                TEST_ASSERT_EQUAL_INT(
                    0, isotp_rx_init(&rx, &peer, received, 4095));
                TEST_ASSERT_EQUAL_INT(0,
                                      isotp_tx_start(&tx, payload, length, 0));
                unsigned now = 0, frames = 0;
                do {
                    now += 200;
                    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME,
                                          isotp_tx_next(&tx, &frame, now));
                    int sent = isotp_tx_confirm(&tx, true, now + 10);
                    int got = isotp_rx_feed(&rx, &frame, now + 10, &segment);
                    TEST_ASSERT_EQUAL_INT(
                        sent == ISOTP_COMPLETE ? ISOTP_COMPLETE : ISOTP_FRAME,
                        got);
                    frames++;
                    if (got == ISOTP_COMPLETE)
                        break;
                    int fc = isotp_rx_flow_control(&rx, &frame, now + 20);
                    if (fc == ISOTP_FRAME) {
                        TEST_ASSERT_EQUAL_INT(
                            0, isotp_rx_confirm(&rx, true, now + 30));
                        TEST_ASSERT_EQUAL_INT(
                            ISOTP_FRAME,
                            isotp_tx_flow_control(&tx, &frame, now + 30));
                    }
                    TEST_ASSERT_TRUE(frames < 700);
                } while (tx.state != ISOTP_TX_IDLE);
                TEST_ASSERT_TRUE(segment.complete);
                TEST_ASSERT_EQUAL_MEM(payload, received, length);
                if (length == 4095)
                    TEST_ASSERT_TRUE(frames > 255);
            }
        }
    }
}

TEST(reserved_stmin_fallback_persists_until_the_message_finishes) {
    start_tx(40, 0);
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(0, 1, 0x80, 1));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 1));
    TEST_ASSERT_EQUAL_INT(0, isotp_tx_confirm(&tx, true, 2));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, flow(0, 0, 0, 3));
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, isotp_tx_next(&tx, &frame, 127001));
    TEST_ASSERT_EQUAL_INT(ISOTP_FRAME, isotp_tx_next(&tx, &frame, 127002));
}

TEST(extended_address_is_checked_before_pci) {
    config.extended_address = true;
    config.rx_address = 0xf1;
    config.tx_address = 0x10;
    TEST_ASSERT_EQUAL_INT(
        0, isotp_rx_init(&rx, &config, received, sizeof(received)));
    uint8_t sf[8] = {0xf2, 1, 0x55};
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(sf, 3, 0));
    sf[0] = 0xf1;
    TEST_ASSERT_EQUAL_INT(ISOTP_COMPLETE, feed(sf, 3, 1));
    TEST_ASSERT_EQUAL_INT(0x55, received[0]);
    sf[1] = 7;
    TEST_ASSERT_EQUAL_INT(ISOTP_IGNORED, feed(sf, 8, 2));
}
