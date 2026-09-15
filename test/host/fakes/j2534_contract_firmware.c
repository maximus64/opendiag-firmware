/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>

#include "freertos/semphr.h"
#include "freertos/task.h"
#include "fake_bus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_port.h"
#include "j2534.h"

int contract_stop(void) {
    j2534_frontend.stop();
    esp_err_t result = vif_bus_release_all(VIF_OWNER_LINK);
    if (vif_pin_release_all(VIF_OWNER_LINK) != ESP_OK)
        return -1;
    return result == ESP_OK && idf_stub_lock_balance() == 0 ? 0 : -1;
}

int contract_reset(void) {
    static bool initialized;

    if (initialized && contract_stop())
        return -1;
    if (!initialized) {
        if (comm_iface_init() != ESP_OK)
            return -1;
        fake_port_register_all();
        initialized = true;
    }
    if (vif_init() != ESP_OK)
        return -1;
    fake_clock_reset();
    fake_can_reset();
    fake_bus_reset_all();
    fake_port_reset();
    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    if (vif_link_start(&j2534_frontend) != ESP_OK)
        return -1;
    vif_link_set_port(fake_port_id(0));
    return j2534_frontend.start() ? 0 : -1;
}

static int byte_bus(unsigned protocol) {
    switch (protocol) {
    case J2534_ISO9141:
        return FAKE_BUS_KLINE;
    case J2534_PWM:
        return FAKE_BUS_J1850_PWM;
    case J2534_VPW:
        return FAKE_BUS_J1850_VPW;
    default:
        return -1;
    }
}

int contract_sent_count(unsigned protocol) {
    if (protocol == J2534_CAN)
        return fake_can_sent_count();
    int bus = byte_bus(protocol);
    return bus < 0 ? -1 : fake_bus_sent_count(bus);
}

int contract_sent(unsigned protocol, unsigned index, uint8_t *data,
                  size_t cap) {
    if (protocol == J2534_CAN) {
        const struct can_frame *frame = fake_can_sent(index);
        if (!frame || cap < 4u + frame->dlc)
            return -1;
        for (unsigned i = 0; i < 4; i++)
            data[i] = frame->id >> (24 - 8 * i);
        memcpy(data + 4, frame->data, frame->dlc);
        return 4 + frame->dlc;
    }
    int bus = byte_bus(protocol);
    size_t len;
    const uint8_t *sent = bus < 0 ? NULL : fake_bus_sent(bus, index, &len);
    if (!sent || cap < len)
        return -1;
    memcpy(data, sent, len);
    return len;
}

int contract_inject(unsigned protocol, const uint8_t *data, size_t len) {
    if (protocol == J2534_CAN) {
        if (len < 4 || len > 12)
            return -1;
        uint32_t id = 0;
        for (unsigned i = 0; i < 4; i++)
            id = id << 8 | data[i];
        fake_can_stage_stale(id, len - 4, data + 4);
        return 0;
    }
    int bus = byte_bus(protocol);
    if (bus < 0 || !len || len > 32)
        return -1;
    fake_bus_stage_stale(bus, data, len);
    return 0;
}
