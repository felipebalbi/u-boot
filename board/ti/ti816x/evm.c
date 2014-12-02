/*
 * evm.c
 *
 * Copyright (C) 2013, Adeneo Embedded <www.adeneo-embedded.com>
 * Antoine Tenart, <atenart@adeneo-embedded.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/cpu.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <miiphy.h>
#include <cpsw.h>

DECLARE_GLOBAL_DATA_PTR;

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_DRAM_1 + 0x100;
	return 0;
}

#ifdef CONFIG_SPL_BUILD

static struct module_pin_mux mmc_pin_mux[] = {
	{ OFFSET(pincntl157), PULLDOWN_EN | PULLUDDIS | MODE(0x0) },
	{ OFFSET(pincntl158), PULLDOWN_EN | PULLUDEN | MODE(0x0) },
	{ OFFSET(pincntl159), PULLUP_EN | PULLUDDIS | MODE(0x0) },
	{ OFFSET(pincntl160), PULLUP_EN | PULLUDDIS | MODE(0x0) },
	{ OFFSET(pincntl161), PULLUP_EN | PULLUDDIS | MODE(0x0) },
	{ OFFSET(pincntl162), PULLUP_EN | PULLUDDIS | MODE(0x0) },
	{ OFFSET(pincntl163), PULLUP_EN | PULLUDDIS | MODE(0x0) },
	{ -1 },
};

static struct module_pin_mux rgmii1_pin_mux[] = {
	{ OFFSET(pincntl232), MODE(0x01) },
	{ OFFSET(pincntl233), PULLUP_EN | MODE(0x01) },
	{ OFFSET(pincntl234), PULLUP_EN | MODE(0x01) },
	{ OFFSET(pincntl235), MODE(0x01) },
	{ OFFSET(pincntl236), MODE(0x01) },
	{ OFFSET(pincntl237), MODE(0x01) },
	{ OFFSET(pincntl238), MODE(0x01) },
	{ OFFSET(pincntl239), MODE(0x01) },
	{ OFFSET(pincntl240), MODE(0x01) },
	{ OFFSET(pincntl241), MODE(0x01) },
	{ OFFSET(pincntl242), MODE(0x01) },
	{ OFFSET(pincntl243), MODE(0x01) },
	{ OFFSET(pincntl244), MODE(0x01) },
	{ OFFSET(pincntl245), MODE(0x01) },
	{ OFFSET(pincntl246), MODE(0x01) },
	{ OFFSET(pincntl247), MODE(0x01) },
	{ OFFSET(pincntl248), MODE(0x01) },
	{ OFFSET(pincntl249), MODE(0x01) },
	{ OFFSET(pincntl250), MODE(0x01) },
	{ OFFSET(pincntl251), MODE(0x01) },
	{ OFFSET(pincntl252), MODE(0x01) },
	{ OFFSET(pincntl253), MODE(0x01) },
	{ OFFSET(pincntl254), MODE(0x01) },
	{ OFFSET(pincntl255), MODE(0x01) },
	{ OFFSET(pincntl256), MODE(0x01) },
	{ OFFSET(pincntl257), MODE(0x01) },
	{ OFFSET(pincntl258), MODE(0x01) },
	{ -1 },
};

const struct dmm_lisa_map_regs evm_lisa_map_regs = {
	.dmm_lisa_map_0 = 0x00000000,
	.dmm_lisa_map_1 = 0x00000000,
	.dmm_lisa_map_2 = 0x80640300,
	.dmm_lisa_map_3 = 0xC0640320,
};

/*
 * DDR2 related definitions
 */
#ifdef CONFIG_TI816X_EVM_DDR2
static struct ddr_data ddr2_data = {
	.datardsratio0		= ((0x40<<10) | (0x40<<0)),
	.datawdsratio0		= ((0x4A<<10) | (0x4A<<0)),
	.datawiratio0		= ((0x0<<10) | (0x0<<0)),
	.datagiratio0		= ((0x0<<10) | (0x0<<0)),
	.datafwsratio0		= ((0x13A<<10) | (0x13A<<0)),
	.datawrsratio0		= ((0x8A<<10) | (0x8A<<0)),
};

static struct cmd_control ddr2_ctrl = {
	.cmd0csratio	= 0x80,
	.cmd0iclkout	= 0x00,

	.cmd1csratio	= 0x80,
	.cmd1iclkout	= 0x00,

	.cmd2csratio	= 0x80,
	.cmd2iclkout	= 0x00,

};

static struct emif_regs ddr2_emif0_regs = {
	.sdram_config		= 0x43801A3A,
	.ref_ctrl		= 0x10000C30,
	.sdram_tim1		= 0x0AAB15E2,
	.sdram_tim2		= 0x423631D2,
	.sdram_tim3		= 0x0080032F,
	.emif_ddr_phy_ctlr_1	= 0x0, /* depend on cpu rev, set later */
};

static struct emif_regs ddr2_emif1_regs = {
	.sdram_config		= 0x43801A3A,
	.ref_ctrl		= 0x10000C30,
	.sdram_tim1		= 0x0AAB15E2,
	.sdram_tim2		= 0x423631D2,
	.sdram_tim3		= 0x0080032F,
	.emif_ddr_phy_ctlr_1	= 0x0, /* depend on cpu rev, set later */
};
#endif

/*
 * DDR3 related definitions
 */

#if defined(CONFIG_TI816X_DDR_PLL_400)
#define RD_DQS		0x03B
#define WR_DQS		0x0A6
#define RD_DQS_GATE	0x12A
#define EMIF_SDCFG	0x62A41032
#define EMIF_SDREF	0x10000C30
#define EMIF_TIM1	0x0CCCE524
#define EMIF_TIM2	0x30308023
#define EMIF_TIM3	0x009F82CF
#define EMIF_PHYCFG	0x0000010B
#elif defined(CONFIG_TI816X_DDR_PLL_531)
#define RD_DQS		0x039
#define WR_DQS		0x0B4
#define RD_DQS_GATE	0x13D
#define EMIF_SDCFG	0x62A51832
#define EMIF_SDREF	0x1000102E
#define EMIF_TIM1	0x0EF136AC
#define EMIF_TIM2	0x30408063
#define EMIF_TIM3	0x009F83AF
#define EMIF_PHYCFG	0x0000010C
#elif defined(CONFIG_TI816X_DDR_PLL_675)
#define RD_DQS		0x039
#define WR_DQS		0x091
#define RD_DQS_GATE	0x196
#define EMIF_SDCFG	0x62A63032
#define EMIF_SDREF	0x10001491
#define EMIF_TIM1	0x13358875
#define EMIF_TIM2	0x5051806C
#define EMIF_TIM3	0x009F84AF
#define EMIF_PHYCFG	0x0000010F
#elif defined(CONFIG_TI816X_DDR_PLL_796)
#define RD_DQS		0x035
#define WR_DQS		0x093
#define RD_DQS_GATE	0x1B3
#define EMIF_SDCFG	0x62A73832
#define EMIF_SDREF	0x10001841
#define EMIF_TIM1	0x1779C9FE
#define EMIF_TIM2	0x50608074
#define EMIF_TIM3	0x009F857F
#define EMIF_PHYCFG	0x00000110
#endif

static struct ddr_data ddr3_data = {
	.datardsratio0		= ((RD_DQS<<10) | (RD_DQS<<0)),
	.datawdsratio0		= ((WR_DQS<<10) | (WR_DQS<<0)),
	.datawiratio0		= ((0x20<<10) | 0x20<<0),
	.datagiratio0		= ((0x20<<10) | 0x20<<0),
	.datafwsratio0		= ((RD_DQS_GATE<<10) | (RD_DQS_GATE<<0)),
	.datawrsratio0		= (((WR_DQS+0x40)<<10) | ((WR_DQS+0x40)<<0)),
};

static const struct cmd_control ddr3_ctrl = {
	.cmd0csratio	= 0x100,
	.cmd0iclkout	= 0x001,

	.cmd1csratio	= 0x100,
	.cmd1iclkout	= 0x001,

	.cmd2csratio	= 0x100,
	.cmd2iclkout	= 0x001,
};

static const struct emif_regs ddr3_emif0_regs = {
	.sdram_config		= EMIF_SDCFG,
	.ref_ctrl		= EMIF_SDREF,
	.sdram_tim1		= EMIF_TIM1,
	.sdram_tim2		= EMIF_TIM2,
	.sdram_tim3		= EMIF_TIM3,
	.emif_ddr_phy_ctlr_1	= EMIF_PHYCFG,
};

static const struct emif_regs ddr3_emif1_regs = {
	.sdram_config		= EMIF_SDCFG,
	.ref_ctrl		= EMIF_SDREF,
	.sdram_tim1		= EMIF_TIM1,
	.sdram_tim2		= EMIF_TIM2,
	.sdram_tim3		= EMIF_TIM3,
	.emif_ddr_phy_ctlr_1	= EMIF_PHYCFG,
};

void set_uart_mux_conf(void) {}

void set_mux_conf_regs(void)
{
	configure_module_pin_mux(mmc_pin_mux);
#ifdef CONFIG_DRIVER_TI_CPSW
	configure_module_pin_mux(rgmii1_pin_mux);
#endif
}

void sdram_init(void)
{
	config_dmm(&evm_lisa_map_regs);

#ifdef CONFIG_TI816X_EVM_DDR2
	if (CONFIG_TI816X_USE_EMIF0) {
		ddr2_emif0_regs.emif_ddr_phy_ctlr_1 =
			(get_cpu_rev() == 0x1 ? 0x0000010B : 0x0000030B);
		config_ddr(0, NULL, &ddr2_data, &ddr2_ctrl, &ddr2_emif0_regs,
			   0);
	}

	if (CONFIG_TI816X_USE_EMIF1) {
		ddr2_emif1_regs.emif_ddr_phy_ctlr_1 =
			(get_cpu_rev() == 0x1 ? 0x0000010B : 0x0000030B);
		config_ddr(1, NULL, &ddr2_data, &ddr2_ctrl, &ddr2_emif1_regs,
			   1);
	}
#endif

#ifdef CONFIG_TI816X_EVM_DDR3
	if (CONFIG_TI816X_USE_EMIF0)
		config_ddr(0, NULL, &ddr3_data, &ddr3_ctrl, &ddr3_emif0_regs,
			   0);

	if (CONFIG_TI816X_USE_EMIF1)
		config_ddr(1, NULL, &ddr3_data, &ddr3_ctrl, &ddr3_emif1_regs,
			   1);
#endif
}
#endif /* CONFIG_SPL_BUILD */

#ifdef CONFIG_DRIVER_TI_CPSW

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 1,
		.phy_if		= PHY_INTERFACE_MODE_RGMII,
	},
};

static void cpsw_control(int enabled)
{
	/* nothing */
}

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

int board_eth_init(bd_t *bis)
{
	int ret;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	puts("Validating E-fuse MAC Address\n");
	if (is_valid_ether_addr(mac_addr)) {
		eth_setenv_enetaddr("ethaddr", mac_addr);
	} else {
		if (!getenv("ethaddr"))
			puts("<ethaddr> not set\n");
	}

	writel(RGMII_MODE_ENABLE, &cdev->miisel);
	cpsw_slaves[0].phy_if = PHY_INTERFACE_MODE_RGMII;

	ret = cpsw_register(&cpsw_data);
	if (ret < 0)
		printf("Error %d registering CPSW switch\n", ret);

	return ret;
}

#endif /* CONFIG_DRIVER_TI_CPSW */
