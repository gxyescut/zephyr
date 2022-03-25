#ifndef __USB_EPFIFO_H_
#define __USB_EPFIFO_H_

#include <zephyr.h>

#define CSR_USB_BASE 0xe0004800L
#define CSR_USB_PULLUP_OUT_ADDR 0xe0004800L
#define CSR_USB_PULLUP_OUT_SIZE 1
static inline unsigned char usb_pullup_out_read(void) {
	unsigned char r = litex_read8(0xe0004800L);
	return r;
}
static inline void usb_pullup_out_write(unsigned char value) {
	litex_write8(value, 0xe0004800L);
}
#define CSR_USB_EP_0_OUT_EV_STATUS_ADDR 0xe0004804L
#define CSR_USB_EP_0_OUT_EV_STATUS_SIZE 1
static inline unsigned char usb_ep_0_out_ev_status_read(void) {
	unsigned char r = litex_read8(0xe0004804L);
	return r;
}
static inline void usb_ep_0_out_ev_status_write(unsigned char value) {
	litex_write8(value, 0xe0004804L);
}
#define CSR_USB_EP_0_OUT_EV_PENDING_ADDR 0xe0004808L
#define CSR_USB_EP_0_OUT_EV_PENDING_SIZE 1
static inline unsigned char usb_ep_0_out_ev_pending_read(void) {
	unsigned char r = litex_read8(0xe0004808L);
	return r;
}
static inline void usb_ep_0_out_ev_pending_write(unsigned char value) {
	litex_write8(value, 0xe0004808L);
}
#define CSR_USB_EP_0_OUT_EV_ENABLE_ADDR 0xe000480cL
#define CSR_USB_EP_0_OUT_EV_ENABLE_SIZE 1
static inline unsigned char usb_ep_0_out_ev_enable_read(void) {
	unsigned char r = litex_read8(0xe000480cL);
	return r;
}
static inline void usb_ep_0_out_ev_enable_write(unsigned char value) {
	litex_write8(value, 0xe000480cL);
}
#define CSR_USB_EP_0_OUT_LAST_TOK_ADDR 0xe0004810L
#define CSR_USB_EP_0_OUT_LAST_TOK_SIZE 1
static inline unsigned char usb_ep_0_out_last_tok_read(void) {
	unsigned char r = litex_read8(0xe0004810L);
	return r;
}
#define CSR_USB_EP_0_OUT_RESPOND_ADDR 0xe0004814L
#define CSR_USB_EP_0_OUT_RESPOND_SIZE 1
static inline unsigned char usb_ep_0_out_respond_read(void) {
	unsigned char r = litex_read8(0xe0004814L);
	return r;
}
static inline void usb_ep_0_out_respond_write(unsigned char value) {
	litex_write8(value, 0xe0004814L);
}
#define CSR_USB_EP_0_OUT_DTB_ADDR 0xe0004818L
#define CSR_USB_EP_0_OUT_DTB_SIZE 1
static inline unsigned char usb_ep_0_out_dtb_read(void) {
	unsigned char r = litex_read8(0xe0004818L);
	return r;
}
static inline void usb_ep_0_out_dtb_write(unsigned char value) {
	litex_write8(value, 0xe0004818L);
}
#define CSR_USB_EP_0_OUT_OBUF_HEAD_ADDR 0xe000481cL
#define CSR_USB_EP_0_OUT_OBUF_HEAD_SIZE 1
static inline unsigned char usb_ep_0_out_obuf_head_read(void) {
	unsigned char r = litex_read8(0xe000481cL);
	return r;
}
static inline void usb_ep_0_out_obuf_head_write(unsigned char value) {
	litex_write8(value, 0xe000481cL);
}
#define CSR_USB_EP_0_OUT_OBUF_EMPTY_ADDR 0xe0004820L
#define CSR_USB_EP_0_OUT_OBUF_EMPTY_SIZE 1
static inline unsigned char usb_ep_0_out_obuf_empty_read(void) {
	unsigned char r = litex_read8(0xe0004820L);
	return r;
}
#define CSR_USB_EP_0_IN_EV_STATUS_ADDR 0xe0004824L
#define CSR_USB_EP_0_IN_EV_STATUS_SIZE 1
static inline unsigned char usb_ep_0_in_ev_status_read(void) {
	unsigned char r = litex_read8(0xe0004824L);
	return r;
}
static inline void usb_ep_0_in_ev_status_write(unsigned char value) {
	litex_write8(value, 0xe0004824L);
}
#define CSR_USB_EP_0_IN_EV_PENDING_ADDR 0xe0004828L
#define CSR_USB_EP_0_IN_EV_PENDING_SIZE 1
static inline unsigned char usb_ep_0_in_ev_pending_read(void) {
	unsigned char r = litex_read8(0xe0004828L);
	return r;
}
static inline void usb_ep_0_in_ev_pending_write(unsigned char value) {
	litex_write8(value, 0xe0004828L);
}
#define CSR_USB_EP_0_IN_EV_ENABLE_ADDR 0xe000482cL
#define CSR_USB_EP_0_IN_EV_ENABLE_SIZE 1
static inline unsigned char usb_ep_0_in_ev_enable_read(void) {
	unsigned char r = litex_read8(0xe000482cL);
	return r;
}
static inline void usb_ep_0_in_ev_enable_write(unsigned char value) {
	litex_write8(value, 0xe000482cL);
}
#define CSR_USB_EP_0_IN_LAST_TOK_ADDR 0xe0004830L
#define CSR_USB_EP_0_IN_LAST_TOK_SIZE 1
static inline unsigned char usb_ep_0_in_last_tok_read(void) {
	unsigned char r = litex_read8(0xe0004830L);
	return r;
}
#define CSR_USB_EP_0_IN_RESPOND_ADDR 0xe0004834L
#define CSR_USB_EP_0_IN_RESPOND_SIZE 1
static inline unsigned char usb_ep_0_in_respond_read(void) {
	unsigned char r = litex_read8(0xe0004834L);
	return r;
}
static inline void usb_ep_0_in_respond_write(unsigned char value) {
	litex_write8(value, 0xe0004834L);
}
#define CSR_USB_EP_0_IN_DTB_ADDR 0xe0004838L
#define CSR_USB_EP_0_IN_DTB_SIZE 1
static inline unsigned char usb_ep_0_in_dtb_read(void) {
	unsigned char r = litex_read8(0xe0004838L);
	return r;
}
static inline void usb_ep_0_in_dtb_write(unsigned char value) {
	litex_write8(value, 0xe0004838L);
}
#define CSR_USB_EP_0_IN_IBUF_HEAD_ADDR 0xe000483cL
#define CSR_USB_EP_0_IN_IBUF_HEAD_SIZE 1
static inline unsigned char usb_ep_0_in_ibuf_head_read(void) {
	unsigned char r = litex_read8(0xe000483cL);
	return r;
}
static inline void usb_ep_0_in_ibuf_head_write(unsigned char value) {
	litex_write8(value, 0xe000483cL);
}
#define CSR_USB_EP_0_IN_IBUF_EMPTY_ADDR 0xe0004840L
#define CSR_USB_EP_0_IN_IBUF_EMPTY_SIZE 1
static inline unsigned char usb_ep_0_in_ibuf_empty_read(void) {
	unsigned char r = litex_read8(0xe0004840L);
	return r;
}
#define CSR_USB_EP_1_IN_EV_STATUS_ADDR 0xe0004844L
#define CSR_USB_EP_1_IN_EV_STATUS_SIZE 1
static inline unsigned char usb_ep_1_in_ev_status_read(void) {
	unsigned char r = litex_read8(0xe0004844L);
	return r;
}
static inline void usb_ep_1_in_ev_status_write(unsigned char value) {
	litex_write8(value, 0xe0004844L);
}
#define CSR_USB_EP_1_IN_EV_PENDING_ADDR 0xe0004848L
#define CSR_USB_EP_1_IN_EV_PENDING_SIZE 1
static inline unsigned char usb_ep_1_in_ev_pending_read(void) {
	unsigned char r = litex_read8(0xe0004848L);
	return r;
}
static inline void usb_ep_1_in_ev_pending_write(unsigned char value) {
	litex_write8(value, 0xe0004848L);
}
#define CSR_USB_EP_1_IN_EV_ENABLE_ADDR 0xe000484cL
#define CSR_USB_EP_1_IN_EV_ENABLE_SIZE 1
static inline unsigned char usb_ep_1_in_ev_enable_read(void) {
	unsigned char r = litex_read8(0xe000484cL);
	return r;
}
static inline void usb_ep_1_in_ev_enable_write(unsigned char value) {
	litex_write8(value, 0xe000484cL);
}
#define CSR_USB_EP_1_IN_LAST_TOK_ADDR 0xe0004850L
#define CSR_USB_EP_1_IN_LAST_TOK_SIZE 1
static inline unsigned char usb_ep_1_in_last_tok_read(void) {
	unsigned char r = litex_read8(0xe0004850L);
	return r;
}
#define CSR_USB_EP_1_IN_RESPOND_ADDR 0xe0004854L
#define CSR_USB_EP_1_IN_RESPOND_SIZE 1
static inline unsigned char usb_ep_1_in_respond_read(void) {
	unsigned char r = litex_read8(0xe0004854L);
	return r;
}
static inline void usb_ep_1_in_respond_write(unsigned char value) {
	litex_write8(value, 0xe0004854L);
}
#define CSR_USB_EP_1_IN_DTB_ADDR 0xe0004858L
#define CSR_USB_EP_1_IN_DTB_SIZE 1
static inline unsigned char usb_ep_1_in_dtb_read(void) {
	unsigned char r = litex_read8(0xe0004858L);
	return r;
}
static inline void usb_ep_1_in_dtb_write(unsigned char value) {
	litex_write8(value, 0xe0004858L);
}
#define CSR_USB_EP_1_IN_IBUF_HEAD_ADDR 0xe000485cL
#define CSR_USB_EP_1_IN_IBUF_HEAD_SIZE 1
static inline unsigned char usb_ep_1_in_ibuf_head_read(void) {
	unsigned char r = litex_read8(0xe000485cL);
	return r;
}
static inline void usb_ep_1_in_ibuf_head_write(unsigned char value) {
	litex_write8(value, 0xe000485cL);
}
#define CSR_USB_EP_1_IN_IBUF_EMPTY_ADDR 0xe0004860L
#define CSR_USB_EP_1_IN_IBUF_EMPTY_SIZE 1
static inline unsigned char usb_ep_1_in_ibuf_empty_read(void) {
	unsigned char r = litex_read8(0xe0004860L);
	return r;
}
#define CSR_USB_EP_2_OUT_EV_STATUS_ADDR 0xe0004864L
#define CSR_USB_EP_2_OUT_EV_STATUS_SIZE 1
static inline unsigned char usb_ep_2_out_ev_status_read(void) {
	unsigned char r = litex_read8(0xe0004864L);
	return r;
}
static inline void usb_ep_2_out_ev_status_write(unsigned char value) {
	litex_write8(value, 0xe0004864L);
}
#define CSR_USB_EP_2_OUT_EV_PENDING_ADDR 0xe0004868L
#define CSR_USB_EP_2_OUT_EV_PENDING_SIZE 1
static inline unsigned char usb_ep_2_out_ev_pending_read(void) {
	unsigned char r = litex_read8(0xe0004868L);
	return r;
}
static inline void usb_ep_2_out_ev_pending_write(unsigned char value) {
	litex_write8(value, 0xe0004868L);
}
#define CSR_USB_EP_2_OUT_EV_ENABLE_ADDR 0xe000486cL
#define CSR_USB_EP_2_OUT_EV_ENABLE_SIZE 1
static inline unsigned char usb_ep_2_out_ev_enable_read(void) {
	unsigned char r = litex_read8(0xe000486cL);
	return r;
}
static inline void usb_ep_2_out_ev_enable_write(unsigned char value) {
	litex_write8(value, 0xe000486cL);
}
#define CSR_USB_EP_2_OUT_LAST_TOK_ADDR 0xe0004870L
#define CSR_USB_EP_2_OUT_LAST_TOK_SIZE 1
static inline unsigned char usb_ep_2_out_last_tok_read(void) {
	unsigned char r = litex_read8(0xe0004870L);
	return r;
}
#define CSR_USB_EP_2_OUT_RESPOND_ADDR 0xe0004874L
#define CSR_USB_EP_2_OUT_RESPOND_SIZE 1
static inline unsigned char usb_ep_2_out_respond_read(void) {
	unsigned char r = litex_read8(0xe0004874L);
	return r;
}
static inline void usb_ep_2_out_respond_write(unsigned char value) {
	litex_write8(value, 0xe0004874L);
}
#define CSR_USB_EP_2_OUT_DTB_ADDR 0xe0004878L
#define CSR_USB_EP_2_OUT_DTB_SIZE 1
static inline unsigned char usb_ep_2_out_dtb_read(void) {
	unsigned char r = litex_read8(0xe0004878L);
	return r;
}
static inline void usb_ep_2_out_dtb_write(unsigned char value) {
	litex_write8(value, 0xe0004878L);
}
#define CSR_USB_EP_2_OUT_OBUF_HEAD_ADDR 0xe000487cL
#define CSR_USB_EP_2_OUT_OBUF_HEAD_SIZE 1
static inline unsigned char usb_ep_2_out_obuf_head_read(void) {
	unsigned char r = litex_read8(0xe000487cL);
	return r;
}
static inline void usb_ep_2_out_obuf_head_write(unsigned char value) {
	litex_write8(value, 0xe000487cL);
}
#define CSR_USB_EP_2_OUT_OBUF_EMPTY_ADDR 0xe0004880L
#define CSR_USB_EP_2_OUT_OBUF_EMPTY_SIZE 1
static inline unsigned char usb_ep_2_out_obuf_empty_read(void) {
	unsigned char r = litex_read8(0xe0004880L);
	return r;
}
#define CSR_USB_EP_2_IN_EV_STATUS_ADDR 0xe0004884L
#define CSR_USB_EP_2_IN_EV_STATUS_SIZE 1
static inline unsigned char usb_ep_2_in_ev_status_read(void) {
	unsigned char r = litex_read8(0xe0004884L);
	return r;
}
static inline void usb_ep_2_in_ev_status_write(unsigned char value) {
	litex_write8(value, 0xe0004884L);
}
#define CSR_USB_EP_2_IN_EV_PENDING_ADDR 0xe0004888L
#define CSR_USB_EP_2_IN_EV_PENDING_SIZE 1
static inline unsigned char usb_ep_2_in_ev_pending_read(void) {
	unsigned char r = litex_read8(0xe0004888L);
	return r;
}
static inline void usb_ep_2_in_ev_pending_write(unsigned char value) {
	litex_write8(value, 0xe0004888L);
}
#define CSR_USB_EP_2_IN_EV_ENABLE_ADDR 0xe000488cL
#define CSR_USB_EP_2_IN_EV_ENABLE_SIZE 1
static inline unsigned char usb_ep_2_in_ev_enable_read(void) {
	unsigned char r = litex_read8(0xe000488cL);
	return r;
}
static inline void usb_ep_2_in_ev_enable_write(unsigned char value) {
	litex_write8(value, 0xe000488cL);
}
#define CSR_USB_EP_2_IN_LAST_TOK_ADDR 0xe0004890L
#define CSR_USB_EP_2_IN_LAST_TOK_SIZE 1
static inline unsigned char usb_ep_2_in_last_tok_read(void) {
	unsigned char r = litex_read8(0xe0004890L);
	return r;
}
#define CSR_USB_EP_2_IN_RESPOND_ADDR 0xe0004894L
#define CSR_USB_EP_2_IN_RESPOND_SIZE 1
static inline unsigned char usb_ep_2_in_respond_read(void) {
	unsigned char r = litex_read8(0xe0004894L);
	return r;
}
static inline void usb_ep_2_in_respond_write(unsigned char value) {
	litex_write8(value, 0xe0004894L);
}
#define CSR_USB_EP_2_IN_DTB_ADDR 0xe0004898L
#define CSR_USB_EP_2_IN_DTB_SIZE 1
static inline unsigned char usb_ep_2_in_dtb_read(void) {
	unsigned char r = litex_read8(0xe0004898L);
	return r;
}
static inline void usb_ep_2_in_dtb_write(unsigned char value) {
	litex_write8(value, 0xe0004898L);
}
#define CSR_USB_EP_2_IN_IBUF_HEAD_ADDR 0xe000489cL
#define CSR_USB_EP_2_IN_IBUF_HEAD_SIZE 1
static inline unsigned char usb_ep_2_in_ibuf_head_read(void) {
	unsigned char r = litex_read8(0xe000489cL);
	return r;
}
static inline void usb_ep_2_in_ibuf_head_write(unsigned char value) {
	litex_write8(value, 0xe000489cL);
}
#define CSR_USB_EP_2_IN_IBUF_EMPTY_ADDR 0xe00048a0L
#define CSR_USB_EP_2_IN_IBUF_EMPTY_SIZE 1
static inline unsigned char usb_ep_2_in_ibuf_empty_read(void) {
	unsigned char r = litex_read8(0xe00048a0L);
	return r;
}
#define CSR_USB_ADDRESS_ADDR 0xe00048a4L
#define CSR_USB_ADDRESS_SIZE 1
static inline unsigned char usb_address_read(void) {
	unsigned char r = litex_read8(0xe00048a4L);
	return r;
}
static inline void usb_address_write(unsigned char value) {
	litex_write8(value, 0xe00048a4L);
}

#endif /* __USB_EPFIFO_H_ */
