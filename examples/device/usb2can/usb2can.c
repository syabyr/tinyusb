#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "bsp/board_api.h"
#include "inc/hw_types.h"
#include "system_TM4C123.h"
#include "tusb.h"

#include "usb2can.h"

#define USB2CAN_DEBUG 1

enum {
    BLINK_NOT_MOUNTED = 250u,
    BLINK_MOUNTED = 1000u,
    BLINK_SUSPENDED = 2500u,
    BLINK_IDENTIFY = 100u,
};

static uint32_t _blink_interval_ms = BLINK_NOT_MOUNTED;
static struct gs_host_config _host_config;
static struct gs_device_bittiming _pending_bittiming;
static struct gs_device_mode _pending_mode;
static struct gs_identify_mode _pending_identify;
static struct gs_device_termination_state _pending_termination;
static gs_host_frame_t _pending_host_frames[8];
static uint8_t _pending_bittiming_channel;
static uint8_t _pending_mode_channel;
static uint8_t _pending_termination_channel;
static uint8_t _pending_host_frame_head;
static uint8_t _pending_host_frame_tail;
static bool _bittiming_update_pending;
static bool _mode_update_pending;
static bool _termination_update_pending;
static bool _vendor_rx_armed;

static const struct gs_device_config _device_config = {
    .reserved1 = 0u,
    .reserved2 = 0u,
    .reserved3 = 0u,
    .icount = USB2CAN_CAN_CHANNELS - 1u,
    .sw_version = USB2CAN_SW_VERSION,
    .hw_version = USB2CAN_HW_VERSION,
};

static struct gs_device_bt_const _bt_const = {
    .feature = USB2CAN_SUPPORTED_FEATURES,
    .fclk_can = 0u,
    .tseg1_min = 1u,
    .tseg1_max = 16u,
    .tseg2_min = 1u,
    .tseg2_max = 8u,
    .sjw_max = 4u,
    .brp_min = 1u,
    .brp_max = 1024u,
    .brp_inc = 1u,
};

#if USB2CAN_DEBUG
static void usb2can_debug_request(char const *tag, tusb_control_request_t const *request) {
    printf("[%s] bm=%02x bReq=%u wValue=%u wIndex=%u wLength=%u\r\n",
           tag,
           request->bmRequestType,
           request->bRequest,
           request->wValue,
           request->wIndex,
           request->wLength);
}

static void usb2can_debug_channel(char const *tag, uint8_t channel) {
    printf("[%s] channel=%u\r\n", tag, channel);
}

static void usb2can_debug_text(char const *tag, char const *text) {
    printf("[%s] %s\r\n", tag, text);
}

static void usb2can_debug_u32(char const *tag, uint32_t value0, uint32_t value1) {
    printf("[%s] %lu %lu\r\n", tag, (unsigned long) value0, (unsigned long) value1);
}
#else
static void usb2can_debug_request(char const *tag, tusb_control_request_t const *request) {
    (void) tag;
    (void) request;
}

static void usb2can_debug_channel(char const *tag, uint8_t channel) {
    (void) tag;
    (void) channel;
}

static void usb2can_debug_text(char const *tag, char const *text) {
    (void) tag;
    (void) text;
}

static void usb2can_debug_u32(char const *tag, uint32_t value0, uint32_t value1) {
    (void) tag;
    (void) value0;
    (void) value1;
}
#endif

static void led_blinking_task(void) {
    static uint32_t start_ms;
    static bool led_state;
    uint32_t interval = usb2can_identify_active() ? BLINK_IDENTIFY : _blink_interval_ms;

    if ((tusb_time_millis_api() - start_ms) < interval) {
        return;
    }

    start_ms += interval;
    led_state = !led_state;
    board_led_write(led_state);
}

static bool usb2can_channel_valid(uint16_t channel) {
    return channel < USB2CAN_CAN_CHANNELS;
}

static bool usb2can_request_is_interface(tusb_control_request_t const *request) {
    return request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE;
}

static bool usb2can_request_is_device_scope(tusb_control_request_t const *request) {
    return usb2can_request_is_interface(request) &&
           request->wValue == 1u &&
           request->wIndex == USB2CAN_USB_INTERFACE;
}

static bool usb2can_request_get_channel(tusb_control_request_t const *request, uint8_t *channel) {
    if (!usb2can_request_is_interface(request)) {
        usb2can_debug_text("channel", "recipient is not interface");
        return false;
    }

    if (request->wIndex == 0u && usb2can_channel_valid(request->wValue)) {
        *channel = (uint8_t) request->wValue;
        usb2can_debug_channel("channel-wValue", *channel);
        return true;
    }

    if (request->wValue == 0u && usb2can_channel_valid(request->wIndex)) {
        *channel = (uint8_t) request->wIndex;
        usb2can_debug_channel("channel-wIndex", *channel);
        return true;
    }

    usb2can_debug_request("channel-invalid", request);
    return false;
}

static bool usb2can_request_is_timestamp_scope(tusb_control_request_t const *request) {
    return usb2can_request_is_interface(request) &&
           request->wValue == 0u &&
           request->wIndex == 0u;
}

static bool usb2can_request_is_bt_const_scope(tusb_control_request_t const *request) {
    if (!usb2can_request_is_interface(request)) {
        usb2can_debug_text("bt-const", "recipient is not interface");
        return false;
    }

    if (usb2can_channel_valid(request->wValue) || usb2can_channel_valid(request->wIndex)) {
        usb2can_debug_request("bt-const-channel", request);
        return true;
    }

    usb2can_debug_request("bt-const-fallback", request);
    return request->wLength == sizeof(_bt_const);
}

static uint32_t usb2can_timestamp_us(void) {
    return tusb_time_millis_api() * 1000u;
}

static bool usb2can_enqueue_host_frame(gs_host_frame_t const *frame) {
    uint8_t next_head = (uint8_t) ((_pending_host_frame_head + 1u) % TU_ARRAY_SIZE(_pending_host_frames));

    if (next_head == _pending_host_frame_tail) {
        usb2can_debug_text("host-enqueue", "queue full");
        return false;
    }

    _pending_host_frames[_pending_host_frame_head] = *frame;
    _pending_host_frame_head = next_head;
    usb2can_debug_u32("host-enqueue", frame->channel, frame->can_id);
    return true;
}

static bool usb2can_dequeue_host_frame(gs_host_frame_t *frame) {
    if (_pending_host_frame_head == _pending_host_frame_tail) {
        return false;
    }

    *frame = _pending_host_frames[_pending_host_frame_tail];
    _pending_host_frame_tail = (uint8_t) ((_pending_host_frame_tail + 1u) % TU_ARRAY_SIZE(_pending_host_frames));
    usb2can_debug_u32("host-dequeue", frame->channel, frame->can_id);
    return true;
}

static void usb2can_vendor_rx_arm(void) {
    if (!tud_mounted()) {
        _vendor_rx_armed = false;
        return;
    }

    if (_vendor_rx_armed) {
        return;
    }

    if (tud_vendor_read_xfer()) {
        _vendor_rx_armed = true;
        usb2can_debug_text("vendor-rx-arm", "queued");
    }
}

static void usb2can_vendor_tx_task(void) {
    gs_host_frame_t frame;

    while (tud_mounted() && tud_vendor_write_available() >= sizeof(frame)) {
        if (!usb2can_can_dequeue_frame(&frame)) {
            break;
        }

        usb2can_debug_u32("vendor-tx", frame.echo_id, frame.can_id);
        tud_vendor_write(&frame, sizeof(frame));
    }
}

static void usb2can_vendor_rx_task(void) {
    usb2can_vendor_rx_arm();
}

static void usb2can_process_pending_control_ops(void) {
    if (_bittiming_update_pending) {
        usb2can_debug_u32("bittiming-apply", _pending_bittiming_channel, _pending_bittiming.brp);
        (void) usb2can_can_configure_bittiming(_pending_bittiming_channel, &_pending_bittiming);
        _bittiming_update_pending = false;
    }

    if (_mode_update_pending) {
        usb2can_debug_u32("mode-apply", _pending_mode_channel, _pending_mode.mode);
        (void) usb2can_can_set_mode(_pending_mode_channel, &_pending_mode);
        _mode_update_pending = false;
    }

    if (_termination_update_pending) {
        usb2can_debug_u32("term-apply", _pending_termination_channel, _pending_termination.state);
        (void) usb2can_can_set_termination(_pending_termination_channel, &_pending_termination);
        _termination_update_pending = false;
    }
}

static void usb2can_process_pending_host_frames(void) {
    gs_host_frame_t frame;

    while (usb2can_dequeue_host_frame(&frame)) {
        usb2can_debug_u32("vendor-frame", frame.channel, frame.can_id);
        usb2can_debug_u32("vendor-send", frame.can_dlc, frame.flags);

        if (!usb2can_can_send_frame(&frame)) {
            usb2can_debug_text("vendor-send", "send failed");
        }
    }
}

static bool usb2can_control_in_request(uint8_t rhport, tusb_control_request_t const *request) {
    static struct gs_device_state state;
    static struct gs_device_termination_state termination;
    static uint32_t timestamp_us;
    uint8_t channel;

    switch (request->bRequest) {
        case GS_USB_BREQ_DEVICE_CONFIG:
            usb2can_debug_request("in-device-config", request);
            if (!usb2can_request_is_device_scope(request)) {
                return false;
            }

            return tud_control_xfer(rhport, request, (void *) (uintptr_t) &_device_config, sizeof(_device_config));

        case GS_USB_BREQ_BT_CONST:
            usb2can_debug_request("in-bt-const", request);
            if (!usb2can_request_is_bt_const_scope(request)) {
                usb2can_debug_text("in-bt-const", "scope rejected");
                return false;
            }

            usb2can_debug_text("in-bt-const", "replying with bt_const");
            return tud_control_xfer(rhport, request, (void *) (uintptr_t) &_bt_const, sizeof(_bt_const));

        case GS_USB_BREQ_TIMESTAMP:
            usb2can_debug_request("in-timestamp", request);
            if (!usb2can_request_is_timestamp_scope(request)) {
                return false;
            }

            timestamp_us = usb2can_timestamp_us();
            return tud_control_xfer(rhport, request, &timestamp_us, sizeof(timestamp_us));

        case GS_USB_BREQ_GET_STATE:
            usb2can_debug_request("in-get-state", request);
            if (!usb2can_request_get_channel(request, &channel)) {
                return false;
            }

            usb2can_can_get_state(channel, &state);
            return tud_control_xfer(rhport, request, &state, sizeof(state));

        case GS_USB_BREQ_GET_TERMINATION:
            usb2can_debug_request("in-get-term", request);
            if (!usb2can_request_get_channel(request, &channel)) {
                return false;
            }

            usb2can_can_get_termination(channel, &termination);
            return tud_control_xfer(rhport, request, &termination, sizeof(termination));

        default:
            return false;
    }
}

static bool usb2can_control_out_setup(uint8_t rhport, tusb_control_request_t const *request) {
    uint8_t channel;

    usb2can_debug_request("out-setup", request);

    switch (request->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT:
            if (!usb2can_request_is_device_scope(request) || request->wLength != sizeof(_host_config)) {
                return false;
            }

            return tud_control_xfer(rhport, request, &_host_config, sizeof(_host_config));

        case GS_USB_BREQ_BITTIMING:
            if (!usb2can_request_get_channel(request, &channel) || request->wLength != sizeof(_pending_bittiming)) {
                return false;
            }

            return tud_control_xfer(rhport, request, &_pending_bittiming, sizeof(_pending_bittiming));

        case GS_USB_BREQ_MODE:
            if (!usb2can_request_get_channel(request, &channel) || request->wLength != sizeof(_pending_mode)) {
                return false;
            }

            return tud_control_xfer(rhport, request, &_pending_mode, sizeof(_pending_mode));

        case GS_USB_BREQ_IDENTIFY:
            if (!usb2can_request_get_channel(request, &channel) || request->wLength != sizeof(_pending_identify)) {
                return false;
            }

            return tud_control_xfer(rhport, request, &_pending_identify, sizeof(_pending_identify));

        case GS_USB_BREQ_SET_TERMINATION:
            if (!usb2can_request_get_channel(request, &channel) || request->wLength != sizeof(_pending_termination)) {
                return false;
            }

            return tud_control_xfer(rhport, request, &_pending_termination, sizeof(_pending_termination));

        default:
            return false;
    }
}

static bool usb2can_control_out_ack(tusb_control_request_t const *request) {
    uint8_t channel;

    usb2can_debug_request("out-ack", request);

    switch (request->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT:
            return _host_config.byte_order == 0x0000beefu;

        case GS_USB_BREQ_BITTIMING:
            if (!usb2can_request_get_channel(request, &channel)) {
                return false;
            }

            _pending_bittiming_channel = channel;
            _bittiming_update_pending = true;
            usb2can_debug_u32("bittiming-queue", channel, _pending_bittiming.brp);
            return true;

        case GS_USB_BREQ_MODE:
            if (!usb2can_request_get_channel(request, &channel)) {
                return false;
            }

            usb2can_debug_u32("mode-ack", _pending_mode.mode, _pending_mode.flags);
            _pending_mode_channel = channel;
            _mode_update_pending = true;
            return true;

        case GS_USB_BREQ_IDENTIFY:
            if (!usb2can_request_get_channel(request, &channel)) {
                return false;
            }

            usb2can_can_identify(_pending_identify.mode != 0u);
            return true;

        case GS_USB_BREQ_SET_TERMINATION:
            if (!usb2can_request_get_channel(request, &channel)) {
                return false;
            }

            _pending_termination_channel = channel;
            _termination_update_pending = true;
            usb2can_debug_u32("term-queue", channel, _pending_termination.state);
            return true;

        default:
            return false;
    }
}

int main(void) {
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };

    board_init();
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    board_init_after_tusb();

    _bt_const.fclk_can = SystemCoreClock;

    while (1) {
        tud_task();
        usb2can_process_pending_control_ops();
        usb2can_process_pending_host_frames();
        usb2can_can_poll();
        usb2can_vendor_tx_task();
        usb2can_vendor_rx_task();
        led_blinking_task();
    }
}

void tud_mount_cb(void) {
    _blink_interval_ms = BLINK_MOUNTED;
    _vendor_rx_armed = false;
}

void tud_umount_cb(void) {
    _blink_interval_ms = BLINK_NOT_MOUNTED;
    _vendor_rx_armed = false;
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
    _blink_interval_ms = BLINK_SUSPENDED;
}

void tud_resume_cb(void) {
    _blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    usb2can_debug_request(stage == CONTROL_STAGE_SETUP ? "vendor-setup" : (stage == CONTROL_STAGE_ACK ? "vendor-ack" : "vendor-data"), request);

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
        usb2can_debug_text("vendor", "not a vendor request");
        return false;
    }

    if (stage == CONTROL_STAGE_SETUP) {
        if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
            return usb2can_control_in_request(rhport, request);
        }

        return usb2can_control_out_setup(rhport, request);
    }

    if (stage == CONTROL_STAGE_ACK && request->bmRequestType_bit.direction == TUSB_DIR_OUT) {
        return usb2can_control_out_ack(request);
    }

    return true;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint32_t bufsize) {
    (void) itf;
    uint32_t offset = 0;

    _vendor_rx_armed = false;

    usb2can_debug_u32("vendor-rx", bufsize, sizeof(gs_host_frame_t));

    while (buffer != NULL && (offset + sizeof(gs_host_frame_t)) <= bufsize) {
        gs_host_frame_t frame;
        uint32_t count = sizeof(frame);

        memcpy(&frame, buffer + offset, sizeof(frame));
        offset += sizeof(frame);

        usb2can_debug_u32("vendor-rx-read", count, sizeof(frame));

        if (!usb2can_enqueue_host_frame(&frame)) {
            usb2can_debug_text("vendor-rx", "host queue full");
        }
    }

    if (buffer == NULL || (bufsize % sizeof(gs_host_frame_t)) != 0u) {
        usb2can_debug_text("vendor-rx", "partial frame");
    }

    usb2can_vendor_rx_arm();
}