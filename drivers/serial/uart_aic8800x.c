/*
 * Copyright (c) 2022 Talpa Chen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/drivers/uart.h>

#define DT_DRV_COMPAT aicsemi_aic8800x_uart

#define GET_GPIO_REGS(port)		(((struct aic8800x_uart_config *)port->config)->uart_regs)

/* TXRXD */
#define UART_TXRXD_DATA_POS             (0)
#define UART_TXRXD_DATA_WIDTH           (8)
#define UART_TXRXD_DATA_MASK            (((0x1ul << UART_TXRXD_DATA_WIDTH) - 1) << UART_TXRXD_DATA_POS)

/* DIV0 */
#define UART_DIV0_DIV_POS               (0)
#define UART_DIV0_DIV_WIDTH             (8)
#define UART_DIV0_DIV_MASK              (((0x1ul << UART_DIV0_DIV_WIDTH) - 1) << UART_DIV0_DIV_POS)

/* IRQCTL */
#define UART_IRQCTL_RXIRQEN_POS         (0)
#define UART_IRQCTL_RXIRQEN_WIDTH       (1)
#define UART_IRQCTL_RXIRQEN_MASK        (((0x1ul << UART_IRQCTL_RXIRQEN_WIDTH) - 1) << UART_IRQCTL_RXIRQEN_POS)
#define UART_IRQCTL_TXIRQEN_POS         (1)
#define UART_IRQCTL_TXIRQEN_WIDTH       (1)
#define UART_IRQCTL_TXIRQEN_MASK        (((0x1ul << UART_IRQCTL_TXIRQEN_WIDTH) - 1) << UART_IRQCTL_TXIRQEN_POS)
#define UART_IRQCTL_LSIRQEN_POS         (2)
#define UART_IRQCTL_LSIRQEN_WIDTH       (1)
#define UART_IRQCTL_LSIRQEN_MASK        (((0x1ul << UART_IRQCTL_LSIRQEN_WIDTH) - 1) << UART_IRQCTL_LSIRQEN_POS)
#define UART_IRQCTL_MSIRQEN_POS         (3)
#define UART_IRQCTL_MSIRQEN_WIDTH       (1)
#define UART_IRQCTL_MSIRQEN_MASK        (((0x1ul << UART_IRQCTL_MSIRQEN_WIDTH) - 1) << UART_IRQCTL_MSIRQEN_POS)
#define UART_IRQCTL_PTIRQEN_POS         (4)
#define UART_IRQCTL_PTIRQEN_WIDTH       (1)
#define UART_IRQCTL_PTIRQEN_MASK        (((0x1ul << UART_IRQCTL_PTIRQEN_WIDTH) - 1) << UART_IRQCTL_PTIRQEN_POS)

/* DIV1 */
#define UART_DIV1_DIV_POS               (0)
#define UART_DIV1_DIV_WIDTH             (8)
#define UART_DIV1_DIV_MASK              (((0x1ul << UART_DIV1_DIV_WIDTH) - 1) << UART_DIV1_DIV_POS)

/* IRQTYP */
#define UART_IRQTYP_TYPE_POS            (0)
#define UART_IRQTYP_TYPE_WIDTH          (4)
#define UART_IRQTYP_TYPE_MASK           (((0x1ul << UART_IRQTYP_TYPE_WIDTH) - 1) << UART_IRQTYP_TYPE_POS)
#   define UART_IRQTYP_RX_ERROR_INT     0x6
#   define UART_IRQTYP_RX_INT           0x4
#   define UART_IRQTYP_TIMEOUT_INT      0xC
#   define UART_IRQTYP_TX_INT           0x2
#   define UART_IRQTYP_MODEM_INT        0x0
#   define UART_IRQTYP_NO_INT           0x1

/* DBUFCFG */
#define UART_DBUFCFG_DBUFEN_POS         (0)
#define UART_DBUFCFG_DBUFEN_WIDTH       (1)
#define UART_DBUFCFG_DBUFEN_MASK        (((0x1ul << UART_DBUFCFG_DBUFEN_WIDTH) - 1) << UART_DBUFCFG_DBUFEN_POS)
#define UART_DBUFCFG_RXDRST_POS         (1)
#define UART_DBUFCFG_RXDRST_WIDTH       (1)
#define UART_DBUFCFG_RXDRST_MASK        (((0x1ul << UART_DBUFCFG_RXDRST_WIDTH) - 1) << UART_DBUFCFG_RXDRST_POS)
#define UART_DBUFCFG_TXDRST_POS         (2)
#define UART_DBUFCFG_TXDRST_WIDTH       (1)
#define UART_DBUFCFG_TXDRST_MASK        (((0x1ul << UART_DBUFCFG_TXDRST_WIDTH) - 1) << UART_DBUFCFG_TXDRST_POS)

/* DFMTCFG */
#define UART_DFMTCFG_DLS_POS            (0)     /* 0 -> 5bits, 1 -> 6bits, 2 -> 7bits, 3 -> 8bits */
#define UART_DFMTCFG_DLS_WIDTH          (2)
#define UART_DFMTCFG_DLS_MASK           (((0x1ul << UART_DFMTCFG_DLS_WIDTH) - 1) << UART_DFMTCFG_DLS_POS)
#define UART_DFMTCFG_STOP_POS           (2)     /* 0 -> 1bits, 1 -> 2bits */
#define UART_DFMTCFG_STOP_WIDTH         (1)
#define UART_DFMTCFG_STOP_MASK          (((0x1ul << UART_DFMTCFG_STOP_WIDTH) - 1) << UART_DFMTCFG_STOP_POS)
#define UART_DFMTCFG_PEN_POS            (3)     /* 0 -> off parity, 1 -> on parity */
#define UART_DFMTCFG_PEN_WIDTH          (1)
#define UART_DFMTCFG_PEN_MASK           (((0x1ul << UART_DFMTCFG_PEN_WIDTH) - 1) << UART_DFMTCFG_PEN_POS)
#define UART_DFMTCFG_EPS_POS            (4)     /* 0 -> odd parity, 1 -> even parity */
#define UART_DFMTCFG_EPS_WIDTH          (1)
#define UART_DFMTCFG_EPS_MASK           (((0x1ul << UART_DFMTCFG_EPS_WIDTH) - 1) << UART_DFMTCFG_EPS_POS)
#define UART_DFMTCFG_BRK_POS            (6)     /* break */
#define UART_DFMTCFG_BRK_WIDTH          (1)
#define UART_DFMTCFG_BRK_MASK           (((0x1ul << UART_DFMTCFG_BRK_WIDTH) - 1) << UART_DFMTCFG_BRK_POS)
#define UART_DFMTCFG_DIVAE_POS          (7)     /* divisor register access, 1 -> enable */
#define UART_DFMTCFG_DIVAE_WIDTH        (1)
#define UART_DFMTCFG_DIVAE_MASK         (((0x1ul << UART_DFMTCFG_DIVAE_WIDTH) - 1) << UART_DFMTCFG_DIVAE_POS)
#define UART_DFMTCFG_DIVMS_POS          (8)     /* divisor mode */
#define UART_DFMTCFG_DIVMS_WIDTH        (1)
#define UART_DFMTCFG_DIVMS_MASK         (((0x1ul << UART_DFMTCFG_DIVMS_WIDTH) - 1) << UART_DFMTCFG_DIVMS_POS)

/* MDMCFG */
#define UART_MDMCFG_DTR_POS             (0)
#define UART_MDMCFG_DTR_WIDTH           (1)
#define UART_MDMCFG_DTR_MASK            (((0x1ul << UART_MDMCFG_DTR_WIDTH) - 1) << UART_MDMCFG_DTR_POS)
#define UART_MDMCFG_RTS_POS             (1)
#define UART_MDMCFG_RTS_WIDTH           (1)
#define UART_MDMCFG_RTS_MASK            (((0x1ul << UART_MDMCFG_RTS_WIDTH) - 1) << UART_MDMCFG_RTS_POS)
#define UART_MDMCFG_OUT1_POS            (2)
#define UART_MDMCFG_OUT1_WIDTH          (1)
#define UART_MDMCFG_OUT1_MASK           (((0x1ul << UART_MDMCFG_OUT1_WIDTH) - 1) << UART_MDMCFG_OUT1_POS)
#define UART_MDMCFG_OUT2_POS            (3)
#define UART_MDMCFG_OUT2_WIDTH          (1)
#define UART_MDMCFG_OUT2_MASK           (((0x1ul << UART_MDMCFG_OUT2_WIDTH) - 1) << UART_MDMCFG_OUT2_POS)
#define UART_MDMCFG_LOOPBACK_POS        (4)
#define UART_MDMCFG_LOOPBACK_WIDTH      (1)
#define UART_MDMCFG_LOOPBACK_MASK       (((0x1ul << UART_MDMCFG_LOOPBACK_WIDTH) - 1) << UART_MDMCFG_LOOPBACK_POS)
#define UART_MDMCFG_AFCE_POS            (5)
#define UART_MDMCFG_AFCE_WIDTH          (1)
#define UART_MDMCFG_AFCE_MASK           (((0x1ul << UART_MDMCFG_AFCE_WIDTH) - 1) << UART_MDMCFG_AFCE_POS)
#define UART_MDMCFG_SIRE_POS            (6)
#define UART_MDMCFG_SIRE_WIDTH          (1)
#define UART_MDMCFG_SIRE_MASK           (((0x1ul << UART_MDMCFG_SIRE_WIDTH) - 1) << UART_MDMCFG_SIRE_POS)
#define UART_MDMCFG_AUTO_DET_POS        (7)
#define UART_MDMCFG_AUTO_DET_WIDTH      (1)
#define UART_MDMCFG_AUTO_DET_MASK       (((0x1ul << UART_MDMCFG_AUTO_DET_WIDTH) - 1) << UART_MDMCFG_AUTO_DET_POS)
#define UART_MDMCFG_CLK_P_POS           (8)     /* clk select ??? */
#define UART_MDMCFG_CLK_P_WIDTH         (1)
#define UART_MDMCFG_CLK_P_MASK          (((0x1ul << UART_MDMCFG_CLK_P_WIDTH) - 1) << UART_MDMCFG_CLK_P_POS)

/* IRQSTS */
#define UART_IRQSTS_DR_POS              (0)
#define UART_IRQSTS_DR_WIDTH            (1)
#define UART_IRQSTS_DR_MASK             (((0x1ul << UART_IRQSTS_DR_WIDTH) - 1) << UART_IRQSTS_DR_POS)
#define UART_IRQSTS_OE_POS              (1)
#define UART_IRQSTS_OE_WIDTH            (1)
#define UART_IRQSTS_OE_MASK             (((0x1ul << UART_IRQSTS_OE_WIDTH) - 1) << UART_IRQSTS_OE_POS)
#define UART_IRQSTS_THRE_POS            (5)
#define UART_IRQSTS_THRE_WIDTH          (1)
#define UART_IRQSTS_THRE_MASK           (((0x1ul << UART_IRQSTS_THRE_WIDTH) - 1) << UART_IRQSTS_THRE_POS)
#define UART_IRQSTS_TEMT_POS            (6)
#define UART_IRQSTS_TEMT_WIDTH          (1)
#define UART_IRQSTS_TEMT_MASK           (((0x1ul << UART_IRQSTS_TEMT_WIDTH) - 1) << UART_IRQSTS_TEMT_POS)
#define UART_IRQSTS_REE_POS             (7)
#define UART_IRQSTS_REE_WIDTH           (1)
#define UART_IRQSTS_REE_MASK            (((0x1ul << UART_IRQSTS_REE_WIDTH) - 1) << UART_IRQSTS_REE_POS)
#define UART_IRQSTS_DTDR_POS            (8)
#define UART_IRQSTS_DTDR_WIDTH          (1)
#define UART_IRQSTS_DTDR_MASK           (((0x1ul << UART_IRQSTS_DTDR_WIDTH) - 1) << UART_IRQSTS_DTDR_POS)

/* MDMSTS */
// NULL

/* DBUFSTS */
#define UART_DBUFSTS_TX_COUNT_POS           (0)
#define UART_DBUFSTS_TX_COUNT_WIDTH         (8)
#define UART_DBUFSTS_TX_COUNT_MASK          (((0x1ul << UART_DBUFSTS_TX_COUNT_WIDTH) - 1) << UART_DBUFSTS_TX_COUNT_POS)
#define UART_DBUFSTS_RX_COUNT_POS           (9)
#define UART_DBUFSTS_RX_COUNT_WIDTH         (8)
#define UART_DBUFSTS_RX_COUNT_MASK          (((0x1ul << UART_DBUFSTS_RX_COUNT_WIDTH) - 1) << UART_DBUFSTS_RX_COUNT_POS)
#define UART_DBUFSTS_TX_DBUF_EMPTY_POS      (18)
#define UART_DBUFSTS_TX_DBUF_EMPTY_WIDTH    (1)
#define UART_DBUFSTS_TX_DBUF_EMPTY_MASK     (((0x1ul << UART_DBUFSTS_TX_DBUF_EMPTY_WIDTH) - 1) << UART_DBUFSTS_TX_DBUF_EMPTY_POS)
#define UART_DBUFSTS_TX_DBUF_FULL_POS       (19)
#define UART_DBUFSTS_TX_DBUF_FULL_WIDTH     (1)
#define UART_DBUFSTS_TX_DBUF_FULL_MASK      (((0x1ul << UART_DBUFSTS_TX_DBUF_FULL_WIDTH) - 1) << UART_DBUFSTS_TX_DBUF_FULL_POS)
#define UART_DBUFSTS_RX_DBUF_EMPTY_POS      (20)
#define UART_DBUFSTS_RX_DBUF_EMPTY_WIDTH    (1)
#define UART_DBUFSTS_RX_DBUF_EMPTY_MASK     (((0x1ul << UART_DBUFSTS_RX_DBUF_EMPTY_WIDTH) - 1) << UART_DBUFSTS_RX_DBUF_EMPTY_POS)
#define UART_DBUFSTS_RX_DBUF_FULL_POS       (21)
#define UART_DBUFSTS_RX_DBUF_FULL_WIDTH     (1)
#define UART_DBUFSTS_RX_DBUF_FULL_MASK      (((0x1ul << UART_DBUFSTS_RX_DBUF_FULL_WIDTH) - 1) << UART_DBUFSTS_RX_DBUF_FULL_POS)

/* DBUFTH */
#define UART_DBUFTH_RXTRIGTH_POS        (0)
#define UART_DBUFTH_RXTRIGTH_WIDTH      (8)
#define UART_DBUFTH_RXTRIGTH_MASK       (((0x1ul << UART_DBUFTH_TXTRIGTH_WIDTH) - 1) << UART_DBUFTH_TXTRIGTH_POS)
#define UART_DBUFTH_TXTRIGTH_POS        (9)
#define UART_DBUFTH_TXTRIGTH_WIDTH      (8)
#define UART_DBUFTH_TXTRIGTH_MASK       (((0x1ul << UART_DBUFTH_RXTRIGTH_WIDTH) - 1) << UART_DBUFTH_RXTRIGTH_POS)

/* DIV2 */
#define UART_DIV2_DIV_POS               (0)
#define UART_DIV2_DIV_WIDTH             (8)
#define UART_DIV2_DIV_MASK              (((0x1ul << UART_DIV2_DIV_WIDTH) - 1) << UART_DIV2_DIV_POS)


struct aic8800x_uart_regs {
    union {
        volatile uint32_t TXRXD;
        volatile uint32_t DIV0;
    };
    union {
        volatile uint32_t IRQCTL;
        volatile uint32_t DIV1;
    };
    union {
        volatile uint32_t IRQTYP;
        volatile uint32_t DBUFCFG;
    };
    volatile uint32_t DFMTCFG;
    
    volatile uint32_t MDMCFG;
    volatile uint32_t IRQSTS;
    volatile uint32_t MDMSTS;
    volatile uint32_t RESERVED0;

    volatile uint32_t DBUFSTS;
    volatile uint32_t DBUFTH;
    volatile uint32_t DIV2;
};

struct aic8800x_uart_config {
	struct aic8800x_uart_regs *const uart_regs;
};

struct aic8800x_uart_data {
	uint32_t dummy;
};

static int uart_aic8800x_init(const struct device *dev)
{
	struct aic8800x_uart_regs *uart_regs = GET_GPIO_REGS(dev);
	//struct aic8800x_uart_data *const data = dev->data;
	return 0;
}

static int uart_aic8800x_poll_in(const struct device *dev, unsigned char *c)
{
	struct aic8800x_uart_regs *uart_regs = GET_GPIO_REGS(dev);

	if ((uart_regs->DBUFSTS & UART_DBUFSTS_RX_DBUF_EMPTY_MASK) != 0) {
		*c = uart_regs->TXRXD;
	} else {
		return -EPERM;
	}

	return 0;
}

static void uart_aic8800x_poll_out(const struct device *dev, unsigned char c)
{
	struct aic8800x_uart_regs *uart_regs = GET_GPIO_REGS(dev);

	while ((uart_regs->DBUFSTS & UART_DBUFSTS_TX_DBUF_FULL_MASK) != 0) {
	}

	uart_regs->TXRXD = c;
}

static int uart_aic8800x_err_check(const struct device *dev)
{
	// TODO
	return 0;
}

static const struct uart_driver_api uart_aic8800x_driver_api = {
	.poll_in = uart_aic8800x_poll_in,
	.poll_out = uart_aic8800x_poll_out,
	.err_check = uart_aic8800x_err_check,
};

#define AIC8800X_UART_IRQ_HANDLER(idx)										\
	static void usart_aic8800_config_func_##idx(const struct device *dev)	\
	{																		\
	}
#define AIC8800X_UART_IRQ_HANDLER_FUNC_INIT(idx)							\
	.irq_config_func = usart_aic8800_config_func_##idx

#define AIC8800X_UART_INIT(idx)							\
	AIC8800X_UART_IRQ_HANDLER(idx)						\
	static struct aic8800x_uart_data uart_aic8800x_data_##idx = {			\
	};									\
	static const struct aic8800x_uart_config uart_aic8800x_config_##idx = {		\
		.uart_regs = DT_INST_REG_ADDR(idx),					\
	};									\
	DEVICE_DT_INST_DEFINE(idx, &uart_aic8800x_init,				\
			      NULL,						\
			      &uart_aic8800x_data_##idx,				\
			      &uart_aic8800x_config_##idx, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &uart_aic8800x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AIC8800X_UART_INIT)
