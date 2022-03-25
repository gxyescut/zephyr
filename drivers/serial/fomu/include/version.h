#ifndef __VERSION_H
#define __VERSION_H

#include <zephyr.h>

#define CSR_VERSION_BASE 0xe0007000L
#define CSR_VERSION_MAJOR_ADDR 0xe0007000L
#define CSR_VERSION_MAJOR_SIZE 1
static inline unsigned char version_major_read(void) {
	unsigned char r = litex_read8(0xe0007000L);
	return r;
}
#define CSR_VERSION_MINOR_ADDR 0xe0007004L
#define CSR_VERSION_MINOR_SIZE 1
static inline unsigned char version_minor_read(void) {
	unsigned char r = litex_read8(0xe0007004L);
	return r;
}
#define CSR_VERSION_REVISION_ADDR 0xe0007008L
#define CSR_VERSION_REVISION_SIZE 1
static inline unsigned char version_revision_read(void) {
	unsigned char r = litex_read8(0xe0007008L);
	return r;
}
#define CSR_VERSION_GITREV_ADDR 0xe000700cL
#define CSR_VERSION_GITREV_SIZE 4
static inline unsigned int version_gitrev_read(void) {
	unsigned int r = litex_read32(0xe000700cL);
	return r;
}
#define CSR_VERSION_GITEXTRA_ADDR 0xe000701cL
#define CSR_VERSION_GITEXTRA_SIZE 2
static inline unsigned short int version_gitextra_read(void) {
	unsigned short int r = litex_read16(0xe000701cL);
	return r;
}
#define CSR_VERSION_DIRTY_ADDR 0xe0007024L
#define CSR_VERSION_DIRTY_SIZE 1
static inline unsigned char version_dirty_read(void) {
	unsigned char r = litex_read8(0xe0007024L);
	return r;
}
#define CSR_VERSION_MODEL_ADDR 0xe0007028L
#define CSR_VERSION_MODEL_SIZE 1
static inline unsigned char version_model_read(void) {
	unsigned char r = litex_read8(0xe0007028L);
	return r;
}

#endif
