#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_can.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "system_TM4C123.h"

#include "bsp/board_api.h"

#include "usb2can.h"

#define USB2CAN_DEBUG 1

#ifndef GPIO_PD0_CAN0RX
#define GPIO_PD0_CAN0RX 0x00030002u
#endif

#ifndef GPIO_PD1_CAN0TX
#define GPIO_PD1_CAN0TX 0x00030402u
#endif

#ifndef GPIO_PF0_CAN1RX
#define GPIO_PF0_CAN1RX 0x00050001u
#endif

#ifndef GPIO_PF1_CAN1TX
#define GPIO_PF1_CAN1TX 0x00050401u
#endif

#define USB2CAN_CAN_QUEUE_DEPTH 16u
#define USB2CAN_CAN_RX_OBJECT   1u
#define USB2CAN_CAN_TX_OBJECT   2u
#define USB2CAN_CAN_TX_OBJECT_MASK (1u << (USB2CAN_CAN_TX_OBJECT - 1u))
#define USB2CAN_SELF_TEST_MODE_FLAGS (GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_ONE_SHOT)

typedef struct {
  uint32_t base;
  uint32_t peripheral;
  uint32_t gpio_peripheral;
  uint32_t gpio_base;
  uint32_t pin_rx;
  uint32_t pin_tx;
  uint32_t pin_cfg_rx;
  uint32_t pin_cfg_tx;
  uint32_t bitrate;
  uint32_t mode_flags;
  gs_host_frame_t pending_tx_echo;
  uint8_t tx_pending_report_polls;
  bool initialized;
  bool self_test_enabled;
  bool started;
  bool tx_echo_pending;
} usb2can_channel_state_t;

static usb2can_channel_state_t _channels[USB2CAN_CAN_CHANNELS] = {
  {
    .base = CAN0_BASE,
    .peripheral = SYSCTL_PERIPH_CAN0,
    .gpio_peripheral = SYSCTL_PERIPH_GPIOD,
    .gpio_base = GPIO_PORTD_BASE,
    .pin_rx = GPIO_PIN_0,
    .pin_tx = GPIO_PIN_1,
    .pin_cfg_rx = GPIO_PD0_CAN0RX,
    .pin_cfg_tx = GPIO_PD1_CAN0TX,
    .bitrate = 500000u,
    .mode_flags = 0u,
    .tx_pending_report_polls = 0u,
    .initialized = false,
    .self_test_enabled = false,
    .started = false,
    .tx_echo_pending = false,
  },
  {
    .base = CAN1_BASE,
    .peripheral = SYSCTL_PERIPH_CAN1,
    .gpio_peripheral = SYSCTL_PERIPH_GPIOF,
    .gpio_base = GPIO_PORTF_BASE,
    .pin_rx = GPIO_PIN_0,
    .pin_tx = GPIO_PIN_1,
    .pin_cfg_rx = GPIO_PF0_CAN1RX,
    .pin_cfg_tx = GPIO_PF1_CAN1TX,
    .bitrate = 500000u,
    .mode_flags = 0u,
    .tx_pending_report_polls = 0u,
    .initialized = false,
    .self_test_enabled = false,
    .started = false,
    .tx_echo_pending = false,
  }
};

static gs_host_frame_t _rx_queue[USB2CAN_CAN_QUEUE_DEPTH];
static uint8_t _rx_queue_head;
static uint8_t _rx_queue_tail;
static bool _identify_on;
static bool _can_initialized;

#if USB2CAN_DEBUG
static void usb2can_can_debug(char const *tag, uint32_t value0, uint32_t value1) {
  printf("[%s] %lu %lu\r\n", tag, (unsigned long) value0, (unsigned long) value1);
}
#else
static void usb2can_can_debug(char const *tag, uint32_t value0, uint32_t value1) {
  (void) tag;
  (void) value0;
  (void) value1;
}
#endif

static uint32_t usb2can_system_clock_hz(void) {
  return SystemCoreClock;
}

static void usb2can_unlock_pf0(void) {
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0u;
}

static void usb2can_configure_channel_pins(usb2can_channel_state_t *channel) {
  SysCtlPeripheralEnable(channel->gpio_peripheral);
  while (!SysCtlPeripheralReady(channel->gpio_peripheral)) {
  }

  if (channel->gpio_base == GPIO_PORTF_BASE) {
    usb2can_unlock_pf0();
  }

  GPIOPinConfigure(channel->pin_cfg_rx);
  GPIOPinConfigure(channel->pin_cfg_tx);
  GPIOPinTypeCAN(channel->gpio_base, channel->pin_rx | channel->pin_tx);
}

static void usb2can_apply_bitrate(usb2can_channel_state_t *channel) {
  CANInit(channel->base);
  CANBitRateSet(channel->base, usb2can_system_clock_hz(), channel->bitrate);
}

static void usb2can_apply_mode_flags(usb2can_channel_state_t *channel) {
  uint32_t control = HWREG(channel->base + CAN_O_CTL) | CAN_CTL_INIT;
  uint32_t test = 0u;

  HWREG(channel->base + CAN_O_CTL) = control;
  CANRetrySet(channel->base, (channel->mode_flags & GS_CAN_MODE_ONE_SHOT) == 0u);

  if (channel->mode_flags & GS_CAN_MODE_LOOP_BACK) {
    test |= CAN_TST_LBACK;
  }

  if (channel->mode_flags & GS_CAN_MODE_LISTEN_ONLY) {
    test |= CAN_TST_SILENT;
  }

  if (test != 0u) {
    HWREG(channel->base + CAN_O_TST) = test;
    HWREG(channel->base + CAN_O_CTL) = control | CAN_CTL_TEST;
  } else {
    HWREG(channel->base + CAN_O_TST) = 0u;
    HWREG(channel->base + CAN_O_CTL) = control & ~CAN_CTL_TEST;
  }
}

static void usb2can_configure_rx_object(usb2can_channel_state_t *channel) {
  tCANMsgObject message_object;

  memset(&message_object, 0, sizeof(message_object));
  message_object.ulMsgID = 0u;
  message_object.ulMsgIDMask = 0u;
  message_object.ulFlags = MSG_OBJ_USE_ID_FILTER;
  message_object.ulMsgLen = 8u;
  CANMessageSet(channel->base, USB2CAN_CAN_RX_OBJECT, &message_object, MSG_OBJ_TYPE_RX);
}

static void usb2can_init_channel(usb2can_channel_state_t *channel, uint8_t channel_index) {
  if (channel->initialized) {
    return;
  }

  usb2can_can_debug("can-init-ch", channel_index, channel->base);

  SysCtlPeripheralEnable(channel->peripheral);
  while (!SysCtlPeripheralReady(channel->peripheral)) {
  }
  usb2can_can_debug("can-init-periph", channel_index, channel->peripheral);

  usb2can_configure_channel_pins(channel);
  usb2can_can_debug("can-init-pins", channel_index, channel->gpio_base);

  usb2can_apply_bitrate(channel);
  usb2can_can_debug("can-init-rate", channel_index, channel->bitrate);

  usb2can_configure_rx_object(channel);
  usb2can_can_debug("can-init-rx", channel_index, USB2CAN_CAN_RX_OBJECT);

  channel->initialized = true;
  _can_initialized = true;
}

static bool usb2can_lazy_init_channel(uint8_t channel_index) {
  if (channel_index >= USB2CAN_CAN_CHANNELS) {
    return false;
  }

  usb2can_init_channel(&_channels[channel_index], channel_index);
  return true;
}

static void usb2can_stop_channel(usb2can_channel_state_t *channel) {
  CANDisable(channel->base);
  CANRetrySet(channel->base, true);
  HWREG(channel->base + CAN_O_CTL) |= CAN_CTL_INIT;
  HWREG(channel->base + CAN_O_TST) = 0u;
  HWREG(channel->base + CAN_O_CTL) &= ~CAN_CTL_TEST;
  channel->tx_echo_pending = false;
  channel->tx_pending_report_polls = 0u;
  channel->started = false;
}

static void usb2can_start_channel(usb2can_channel_state_t *channel) {
  usb2can_apply_bitrate(channel);
  usb2can_apply_mode_flags(channel);
  usb2can_configure_rx_object(channel);
  CANEnable(channel->base);
  channel->tx_echo_pending = false;
  channel->tx_pending_report_polls = 0u;
  channel->started = true;
}

static bool usb2can_queue_frame(const gs_host_frame_t *frame) {
  uint8_t next_head = (uint8_t) ((_rx_queue_head + 1u) % USB2CAN_CAN_QUEUE_DEPTH);

  if (next_head == _rx_queue_tail) {
    return false;
  }

  _rx_queue[_rx_queue_head] = *frame;
  _rx_queue_head = next_head;
  return true;
}

static uint32_t usb2can_calc_bitrate(const struct gs_device_bittiming *timing) {
  uint32_t tq_count;

  if (timing->brp == 0u) {
    return 0u;
  }

  tq_count = 1u + timing->prop_seg + timing->phase_seg1 + timing->phase_seg2;
  if (tq_count == 0u) {
    return 0u;
  }

  return usb2can_system_clock_hz() / (timing->brp * tq_count);
}

static uint32_t usb2can_tx_flags_from_can_id(uint32_t can_id) {
  uint32_t flags = MSG_OBJ_NO_FLAGS;

  if (can_id & USB2CAN_CAN_EFF_FLAG) {
    flags |= MSG_OBJ_EXTENDED_ID;
  }

  if (can_id & USB2CAN_CAN_RTR_FLAG) {
    flags |= MSG_OBJ_REMOTE_FRAME;
  }

  return flags;
}

static bool usb2can_channel_self_test_enabled(const usb2can_channel_state_t *channel) {
#if USB2CAN_FORCE_SELF_TEST
  (void) channel;
  return true;
#else
  if (channel->self_test_enabled) {
    return true;
  }

  if ((channel->mode_flags & GS_CAN_MODE_LOOP_BACK) != 0u) {
    return true;
  }

  return (channel->mode_flags & USB2CAN_SELF_TEST_MODE_FLAGS) == USB2CAN_SELF_TEST_MODE_FLAGS;
#endif
}

static bool usb2can_queue_self_test_frames(const gs_host_frame_t *frame) {
  gs_host_frame_t echo_frame = *frame;
  gs_host_frame_t rx_frame = *frame;

  echo_frame.flags = 0u;
  rx_frame.echo_id = GS_HOST_FRAME_ECHO_ID_RX;
  rx_frame.flags = 0u;

  if (!usb2can_queue_frame(&echo_frame)) {
    return false;
  }

  if (!usb2can_queue_frame(&rx_frame)) {
    _rx_queue_head = (uint8_t) ((_rx_queue_head + USB2CAN_CAN_QUEUE_DEPTH - 1u) % USB2CAN_CAN_QUEUE_DEPTH);
    return false;
  }

  return true;
}

void usb2can_can_init(void) {
  uint8_t channel_index;

  usb2can_can_debug("can-init", USB2CAN_CAN_CHANNELS, 0u);

  for (channel_index = 0; channel_index < USB2CAN_CAN_CHANNELS; channel_index++) {
    usb2can_init_channel(&_channels[channel_index], channel_index);
  }
}

bool usb2can_can_configure_bittiming(uint8_t channel_index, const struct gs_device_bittiming *timing) {
  usb2can_channel_state_t *channel;
  uint32_t bitrate;

  if (channel_index >= USB2CAN_CAN_CHANNELS) {
    return false;
  }

  bitrate = usb2can_calc_bitrate(timing);
  if (bitrate == 0u) {
    return false;
  }

  channel = &_channels[channel_index];
  channel->bitrate = bitrate;
  usb2can_can_debug("bitrate-store", channel_index, bitrate);

  return true;
}

bool usb2can_can_set_mode(uint8_t channel_index, const struct gs_device_mode *mode) {
  usb2can_channel_state_t *channel;
  uint32_t mode_flags;

  if (!usb2can_lazy_init_channel(channel_index)) {
    return false;
  }

  channel = &_channels[channel_index];
  mode_flags = mode->flags;

  if (mode->mode == GS_CAN_MODE_START) {
    if (!usb2can_lazy_init_channel(channel_index)) {
      return false;
    }

    if ((mode_flags & ~USB2CAN_SUPPORTED_MODE_FLAGS) != 0u) {
      usb2can_can_debug("mode-flags-mask", channel_index, mode_flags);
    }

    channel->mode_flags = mode_flags & USB2CAN_SUPPORTED_MODE_FLAGS;
    usb2can_can_debug("mode-store", channel_index, channel->mode_flags);
    usb2can_can_debug("self-test-mode", channel_index, usb2can_channel_self_test_enabled(channel) ? 1u : 0u);
    usb2can_start_channel(channel);
  } else if (mode->mode == GS_CAN_MODE_RESET) {
    usb2can_stop_channel(channel);
  } else {
    usb2can_can_debug("mode-invalid", channel_index, mode->mode);
    return false;
  }

  return true;
}

bool usb2can_can_set_termination(uint8_t channel_index, const struct gs_device_termination_state *termination) {
  usb2can_channel_state_t *channel;
  bool enabled;

  if (!usb2can_lazy_init_channel(channel_index)) {
    return false;
  }

  channel = &_channels[channel_index];
  enabled = termination->state == USB2CAN_TERMINATION_ENABLED;
  channel->self_test_enabled = enabled;
  usb2can_can_debug("self-test-set", channel_index, enabled ? 1u : 0u);

  if (channel->started) {
    usb2can_stop_channel(channel);
    usb2can_start_channel(channel);
  }

  return termination->state == USB2CAN_TERMINATION_DISABLED || enabled;
}

void usb2can_can_get_termination(uint8_t channel_index, struct gs_device_termination_state *termination) {
  memset(termination, 0, sizeof(*termination));

  if (channel_index >= USB2CAN_CAN_CHANNELS) {
    termination->state = USB2CAN_TERMINATION_DISABLED;
    return;
  }

  termination->state = _channels[channel_index].self_test_enabled ? USB2CAN_TERMINATION_ENABLED : USB2CAN_TERMINATION_DISABLED;
  usb2can_can_debug("self-test-get", channel_index, termination->state);
}

void usb2can_can_identify(bool identify_on) {
  _identify_on = identify_on;
  board_led_write(identify_on);
}

void usb2can_can_get_state(uint8_t channel_index, struct gs_device_state *state) {
  usb2can_channel_state_t *channel;
  unsigned long rxerr = 0u;
  unsigned long txerr = 0u;

  memset(state, 0, sizeof(*state));

  usb2can_can_debug("get-state", channel_index, _can_initialized ? 1u : 0u);

  if (!_can_initialized) {
    state->state = GS_CAN_STATE_STOPPED;
    return;
  }

  if (channel_index >= USB2CAN_CAN_CHANNELS) {
    state->state = GS_CAN_STATE_STOPPED;
    return;
  }

  channel = &_channels[channel_index];
  if (!channel->started) {
    state->state = GS_CAN_STATE_STOPPED;
    return;
  }

  CANErrCntrGet(channel->base, &rxerr, &txerr);
  state->rxerr = (uint32_t) rxerr;
  state->txerr = (uint32_t) txerr;

  switch (CANStatusGet(channel->base, CAN_STS_CONTROL) & (CAN_STATUS_BUS_OFF | CAN_STATUS_EPASS | CAN_STATUS_EWARN)) {
    case CAN_STATUS_BUS_OFF:
      state->state = GS_CAN_STATE_BUS_OFF;
      break;

    case CAN_STATUS_EPASS:
      state->state = GS_CAN_STATE_ERROR_PASSIVE;
      break;

    case CAN_STATUS_EWARN:
      state->state = GS_CAN_STATE_ERROR_WARNING;
      break;

    default:
      state->state = GS_CAN_STATE_ERROR_ACTIVE;
      break;
  }
}

bool usb2can_can_send_frame(const gs_host_frame_t *frame) {
  usb2can_channel_state_t *channel;
  tCANMsgObject message_object;
  uint8_t message_data[8];
  uint8_t length;
  uint32_t tx_request;

  if (!usb2can_lazy_init_channel(frame->channel)) {
    usb2can_can_debug("tx-init-fail", frame->channel, 0u);
    return false;
  }

  channel = &_channels[frame->channel];
  if (!channel->started) {
    usb2can_can_debug("tx-not-started", frame->channel, channel->bitrate);
    return false;
  }

  usb2can_can_debug("tx-frame", frame->channel, frame->can_id);

  if (usb2can_channel_self_test_enabled(channel)) {
    bool queued = usb2can_queue_self_test_frames(frame);

    usb2can_can_debug("tx-self-test", frame->channel, queued ? 1u : 0u);
    return queued;
  }

  tx_request = CANStatusGet(channel->base, CAN_STS_TXREQUEST);
  if (channel->tx_echo_pending || ((tx_request & USB2CAN_CAN_TX_OBJECT_MASK) != 0u)) {
    usb2can_can_debug("tx-busy", frame->channel, tx_request);
    return false;
  }

  length = frame->can_dlc;
  if (length > 8u) {
    length = 8u;
  }

  memcpy(message_data, frame->data, length);

  memset(&message_object, 0, sizeof(message_object));
  message_object.ulMsgID = frame->can_id & USB2CAN_CAN_EFF_MASK;
  message_object.ulFlags = usb2can_tx_flags_from_can_id(frame->can_id);
  message_object.ulMsgLen = length;
  message_object.pucMsgData = message_data;
  usb2can_can_debug("tx-submit", frame->channel, message_object.ulMsgLen);
  CANMessageSet(channel->base, USB2CAN_CAN_TX_OBJECT, &message_object, MSG_OBJ_TYPE_TX);
  tx_request = CANStatusGet(channel->base, CAN_STS_TXREQUEST);
  usb2can_can_debug("tx-submitted", frame->channel, tx_request);

  channel->pending_tx_echo = *frame;
  channel->pending_tx_echo.flags = 0u;
  channel->tx_pending_report_polls = 0u;
  channel->tx_echo_pending = true;
  return true;
}

void usb2can_can_poll(void) {
  uint8_t channel_index;

  for (channel_index = 0; channel_index < USB2CAN_CAN_CHANNELS; channel_index++) {
    usb2can_channel_state_t *channel = &_channels[channel_index];

    if (!channel->initialized || !channel->started) {
      continue;
    }

    if (channel->tx_echo_pending) {
      uint32_t tx_request = CANStatusGet(channel->base, CAN_STS_TXREQUEST);

      if ((tx_request & USB2CAN_CAN_TX_OBJECT_MASK) == 0u) {
        usb2can_can_debug("tx-echo", channel_index, tx_request);
        if (usb2can_queue_frame(&channel->pending_tx_echo)) {
          channel->tx_echo_pending = false;
          channel->tx_pending_report_polls = 0u;
        }
      } else if (channel->tx_pending_report_polls < 20u) {
        channel->tx_pending_report_polls++;
      } else {
        unsigned long rxerr = 0u;
        unsigned long txerr = 0u;
        uint32_t control = CANStatusGet(channel->base, CAN_STS_CONTROL);
        uint32_t ctl = HWREG(channel->base + CAN_O_CTL);
        uint32_t tst = HWREG(channel->base + CAN_O_TST);
        uint32_t msgval = CANStatusGet(channel->base, CAN_STS_MSGVAL);

        CANErrCntrGet(channel->base, &rxerr, &txerr);
        usb2can_can_debug("tx-wait", channel_index, tx_request);
        usb2can_can_debug("tx-state", control, (uint32_t) txerr);
        usb2can_can_debug("tx-regs", ctl, tst);
        usb2can_can_debug("tx-msgval", msgval, (uint32_t) rxerr);
        channel->tx_pending_report_polls = 0u;
      }
    }

    if ((CANStatusGet(channel->base, CAN_STS_NEWDAT) & 0x1u) != 0u) {
      gs_host_frame_t frame;
      tCANMsgObject message_object;
      uint8_t message_data[8];

      memset(&frame, 0, sizeof(frame));
      memset(&message_object, 0, sizeof(message_object));

      message_object.pucMsgData = message_data;
      message_object.ulMsgLen = sizeof(message_data);
      CANMessageGet(channel->base, USB2CAN_CAN_RX_OBJECT, &message_object, true);

      frame.echo_id = GS_HOST_FRAME_ECHO_ID_RX;
      frame.can_id = message_object.ulMsgID;
      frame.can_dlc = (uint8_t) message_object.ulMsgLen;
      frame.channel = channel_index;
      frame.flags = 0u;

      if (message_object.ulFlags & MSG_OBJ_EXTENDED_ID) {
        frame.can_id |= USB2CAN_CAN_EFF_FLAG;
      }

      if (message_object.ulFlags & MSG_OBJ_REMOTE_FRAME) {
        frame.can_id |= USB2CAN_CAN_RTR_FLAG;
      }

      if (frame.can_dlc > sizeof(frame.data)) {
        frame.can_dlc = sizeof(frame.data);
      }

      memcpy(frame.data, message_data, frame.can_dlc);
      usb2can_queue_frame(&frame);
      usb2can_configure_rx_object(channel);
    }
  }
}

bool usb2can_can_dequeue_frame(gs_host_frame_t *frame) {
  if (_rx_queue_head == _rx_queue_tail) {
    return false;
  }

  *frame = _rx_queue[_rx_queue_tail];
  _rx_queue_tail = (uint8_t) ((_rx_queue_tail + 1u) % USB2CAN_CAN_QUEUE_DEPTH);
  return true;
}

bool usb2can_identify_active(void) {
  return _identify_on;
}