#define FASTVDMA_IRQ_WR_EN   BIT(0)
#define FASTVDMA_IRQ_RD_EN   BIT(1)

#define FASTVDMA_CTRL_DISABLE 0x0
#define FASTVDMA_CTRL_WR_EN   BIT(0)
#define FASTVDMA_CTRL_RD_EN   BIT(1)
#define FASTVDMA_CTRL_WR_SYNC BIT(2)
#define FASTVDMA_CTRL_RD_SYNC BIT(3)
#define FASTVDMA_CTRL_WR_LOOP BIT(4)
#define FASTVDMA_CTRL_RD_LOOP BIT(5)

#define FASTVDMA_STATUS_WR_BUSY BIT(0)
#define FASTVDMA_STATUS_RD_BUSY BIT(1)

#define FASTVDMA_IRQ_WR_EN    BIT(0)
#define FASTVDMA_IRQ_RD_EN    BIT(1)

#define FASTVDMA_IRQ_WR_TRIG  BIT(0)
#define FASTVDMA_IRQ_RD_TRIG  BIT(1)

#define EV_ENABLE	BIT(0)

#define REG_SIZE 1

struct fastvdma_dev_cfg {
	volatile uint32_t *base_addr;
	volatile uint32_t *control_addr;
	volatile uint32_t *status_addr;
	volatile uint32_t *irq_mask_addr;
	volatile uint32_t *irq_status_addr;
	volatile uint32_t *read_base_addr;
	volatile uint32_t *read_length_addr;
	volatile uint32_t *read_count_addr;
	volatile uint32_t *read_stride_addr;
	volatile uint32_t *write_base_addr;
	volatile uint32_t *write_length_addr;
	volatile uint32_t *write_count_addr;
	volatile uint32_t *write_stride_addr;
	volatile uint32_t *ev_status_addr;
	volatile uint32_t *ev_pending_addr;
	volatile uint32_t *ev_enable_addr;
	dma_callback_t cb;
	enum dma_channel_direction dir;
	void *user_data;
	void (*irq_config)(void);
};
