#ifndef USB2CAN_H_
#define USB2CAN_H_

#include <stdbool.h>
#include <stdint.h>

#define USB2CAN_USB_VID          0x1209u
#define USB2CAN_USB_PID          0x2323u
#define USB2CAN_USB_BCD          0x0100u

#define USB2CAN_SW_VERSION       0x00010000u
#define USB2CAN_HW_VERSION       0x00010000u

#define USB2CAN_USB_INTERFACE    0u
#define USB2CAN_CAN_CHANNELS     2u
#define USB2CAN_FORCE_SELF_TEST  1u

#define USB2CAN_CAN_EFF_FLAG     0x80000000u
#define USB2CAN_CAN_RTR_FLAG     0x40000000u
#define USB2CAN_CAN_ERR_FLAG     0x20000000u
#define USB2CAN_CAN_EFF_MASK     0x1fffffffu

#define GS_CAN_FEATURE_LISTEN_ONLY     (1u << 0)
#define GS_CAN_FEATURE_LOOP_BACK       (1u << 1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE   (1u << 2)
#define GS_CAN_FEATURE_ONE_SHOT        (1u << 3)
#define GS_CAN_FEATURE_HW_TIMESTAMP    (1u << 4)
#define GS_CAN_FEATURE_IDENTIFY        (1u << 5)
#define GS_CAN_FEATURE_USER_ID         (1u << 6)
#define GS_CAN_FEATURE_PAD_PKTS        (1u << 7)
#define GS_CAN_FEATURE_FD              (1u << 8)
#define GS_CAN_FEATURE_BT_CONST_EXT    (1u << 10)
#define GS_CAN_FEATURE_TERMINATION     (1u << 11)
#define GS_CAN_FEATURE_BERR_REPORTING  (1u << 12)
#define GS_CAN_FEATURE_GET_STATE       (1u << 13)

#define GS_CAN_MODE_RESET              0u
#define GS_CAN_MODE_START              1u

#define GS_CAN_MODE_NORMAL             0u
#define GS_CAN_MODE_LISTEN_ONLY        (1u << 0)
#define GS_CAN_MODE_LOOP_BACK          (1u << 1)
#define GS_CAN_MODE_TRIPLE_SAMPLE      (1u << 2)
#define GS_CAN_MODE_ONE_SHOT           (1u << 3)
#define GS_CAN_MODE_HW_TIMESTAMP       (1u << 4)
#define GS_CAN_MODE_PAD_PKTS           (1u << 7)
#define GS_CAN_MODE_FD                 (1u << 8)
#define GS_CAN_MODE_BERR_REPORTING     (1u << 12)

#define USB2CAN_TERMINATION_DISABLED   0u
#define USB2CAN_TERMINATION_ENABLED    120u

#define USB2CAN_SUPPORTED_FEATURES     (GS_CAN_FEATURE_LISTEN_ONLY | GS_CAN_FEATURE_LOOP_BACK | GS_CAN_FEATURE_ONE_SHOT | GS_CAN_FEATURE_IDENTIFY | GS_CAN_FEATURE_TERMINATION)
#define USB2CAN_SUPPORTED_MODE_FLAGS   (GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_ONE_SHOT)

#define GS_CAN_STATE_ERROR_ACTIVE      0u
#define GS_CAN_STATE_ERROR_WARNING     1u
#define GS_CAN_STATE_ERROR_PASSIVE     2u
#define GS_CAN_STATE_BUS_OFF           3u
#define GS_CAN_STATE_STOPPED           4u
#define GS_CAN_STATE_SLEEPING          5u

#define GS_HOST_FRAME_ECHO_ID_RX       0xffffffffu

enum gs_usb_breq {
	GS_USB_BREQ_HOST_FORMAT = 0,
	GS_USB_BREQ_BITTIMING,
	GS_USB_BREQ_MODE,
	GS_USB_BREQ_BERR,
	GS_USB_BREQ_BT_CONST,
	GS_USB_BREQ_DEVICE_CONFIG,
	GS_USB_BREQ_TIMESTAMP,
	GS_USB_BREQ_IDENTIFY,
	GS_USB_BREQ_GET_USER_ID,
	GS_USB_BREQ_SET_USER_ID,
	GS_USB_BREQ_DATA_BITTIMING,
	GS_USB_BREQ_BT_CONST_EXT,
	GS_USB_BREQ_SET_TERMINATION,
	GS_USB_BREQ_GET_TERMINATION,
	GS_USB_BREQ_GET_STATE,
};

struct gs_host_config {
	uint32_t byte_order;
} __attribute__((packed));

struct gs_device_config {
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t icount;
	uint32_t sw_version;
	uint32_t hw_version;
} __attribute__((packed));

struct gs_device_mode {
	uint32_t mode;
	uint32_t flags;
} __attribute__((packed));

struct gs_device_state {
	uint32_t state;
	uint32_t rxerr;
	uint32_t txerr;
} __attribute__((packed));

struct gs_device_bittiming {
	uint32_t prop_seg;
	uint32_t phase_seg1;
	uint32_t phase_seg2;
	uint32_t sjw;
	uint32_t brp;
} __attribute__((packed));

struct gs_identify_mode {
	uint32_t mode;
} __attribute__((packed));

struct gs_device_termination_state {
	uint32_t state;
} __attribute__((packed));

struct gs_device_bt_const {
	uint32_t feature;
	uint32_t fclk_can;
	uint32_t tseg1_min;
	uint32_t tseg1_max;
	uint32_t tseg2_min;
	uint32_t tseg2_max;
	uint32_t sjw_max;
	uint32_t brp_min;
	uint32_t brp_max;
	uint32_t brp_inc;
} __attribute__((packed));

typedef struct {
	uint32_t echo_id;
	uint32_t can_id;
	uint8_t can_dlc;
	uint8_t channel;
	uint8_t flags;
	uint8_t reserved;
	uint8_t data[8];
} __attribute__((packed)) gs_host_frame_t;

void usb2can_can_init(void);
void usb2can_can_poll(void);
bool usb2can_can_configure_bittiming(uint8_t channel_index, const struct gs_device_bittiming *timing);
bool usb2can_can_set_mode(uint8_t channel_index, const struct gs_device_mode *mode);
bool usb2can_can_send_frame(const gs_host_frame_t *frame);
bool usb2can_can_dequeue_frame(gs_host_frame_t *frame);
void usb2can_can_get_state(uint8_t channel_index, struct gs_device_state *state);
void usb2can_can_identify(bool identify_on);
bool usb2can_identify_active(void);
bool usb2can_can_set_termination(uint8_t channel_index, const struct gs_device_termination_state *termination);
void usb2can_can_get_termination(uint8_t channel_index, struct gs_device_termination_state *termination);

#endif