#ifndef __QUICK_FEATHER_PWM_GEN_H
#define __QUICK_FEATHER_PWM_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <eoss3_dev.h>

typedef struct {
	__IO u32_t PWM_DEVICE_ID;	/* Address offset: 0x00 */
	__IO u32_t PWM_DEVICE_REV;	/* Address offset: 0x04 */
	__IO u32_t PWM_EN;		/* Address offset: 0x08 */
	__IO u32_t PWM0_WIDTH;	/* Address offset: 0x0c */
	__IO u32_t PWM0_PERIOD;	/* Address offset: 0x10 */
	__IO u32_t PWM1_WIDTH;	/* Address offset: 0x14 */
	__IO u32_t PWM1_PERIOD;	/* Address offset: 0x18 */
	__IO u32_t PWM2_WIDTH;	/* Address offset: 0x1c */
	__IO u32_t PWM2_PERIOD;	/* Address offset: 0x20 */
} PWM_GEN_TypeDef;

#define PWM_GEN_ID	((uint32_t) (0x50574D30))
#define PWM_GEN_REV	((uint32_t) (0x00000100))

#ifdef __cplusplus
}
#endif

#endif /* __EOSS3_HAL_PADS_H */
