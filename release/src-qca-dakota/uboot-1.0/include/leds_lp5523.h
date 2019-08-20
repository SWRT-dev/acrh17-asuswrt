/*
 *  * Copyright 2013, ASUSTeK Inc.
 *   * All Rights Reserved.
 *    */
#include <common.h>
#include <command.h>
#include <asm/errno.h>
#include <malloc.h>
#include <compiler.h>
#include <linux/compat.h>
#include <i2c.h>

#ifndef _LEDS_LP5523_H
#define _LEDS_LP5523_H

#define LP5523_PROGRAM_LENGTH		32      /* bytes */
/* Memory is used like this:
 * 0x00 engine 1 program
 * 0x10 engine 2 program
 * 0x20 engine 3 program
 * 0x30 engine 1 muxing info
 * 0x40 engine 2 muxing info
 * 0x50 engine 3 muxing info
 *                   */
#define LP5523_MAX_LEDS                 9
#define LP5523_MAX_ENGINES              3

/* Registers */
#define LP5523_REG_ENABLE               0x00
#define LP5523_REG_OP_MODE              0x01
#define LP5523_REG_ENABLE_LEDS_MSB      0x04
#define LP5523_REG_ENABLE_LEDS_LSB      0x05
#define LP5523_REG_LED_PWM_BASE         0x16
#define LP5523_REG_LED_CURRENT_BASE     0x26
#define LP5523_REG_CONFIG               0x36
#define LP5523_REG_STATUS               0x3A
#define LP5523_REG_RESET                0x3D
#define LP5523_REG_LED_TEST_CTRL        0x41
#define LP5523_REG_LED_TEST_ADC         0x42
#define LP5523_REG_CH1_PROG_START       0x4C
#define LP5523_REG_CH2_PROG_START       0x4D
#define LP5523_REG_CH3_PROG_START       0x4E
#define LP5523_REG_PROG_PAGE_SEL        0x4F
#define LP5523_REG_PROG_MEM             0x50

/* Bit description in registers */
#define LP5523_ENABLE                   0x40
#define LP5523_AUTO_INC                 0x40
#define LP5523_PWR_SAVE                 0x20
#define LP5523_PWM_PWR_SAVE             0x04
#define LP5523_CP_AUTO                  0x18
#define LP5523_AUTO_CLK                 0x02

#define LP5523_EN_LEDTEST               0x80
#define LP5523_LEDTEST_DONE             0x80
#define LP5523_RESET                    0xFF
#define LP5523_ADC_SHORTCIRC_LIM        80
#define LP5523_EXT_CLK_USED             0x08
#define LP5523_ENG_STATUS_MASK          0x07

/* Memory Page Selection */
#define LP5523_PAGE_ENG1                0
#define LP5523_PAGE_ENG2                1
#define LP5523_PAGE_ENG3                2
#define LP5523_PAGE_MUX1                3
#define LP5523_PAGE_MUX2                4
#define LP5523_PAGE_MUX3                5

/* Program Memory Operations */
#define LP5523_MODE_ENG1_M              0x30    /* Operation Mode Register */
#define LP5523_MODE_ENG2_M              0x0C
#define LP5523_MODE_ENG3_M              0x03
#define LP5523_LOAD_ENG1                0x10
#define LP5523_LOAD_ENG2                0x04
#define LP5523_LOAD_ENG3                0x01

#define LP5523_EXEC_ENG1_M              0x30    /* Enable Register */
#define LP5523_EXEC_ENG2_M              0x0C
#define LP5523_EXEC_ENG3_M              0x03
#define LP5523_EXEC_M                   0x3F
#define LP5523_RUN_ENG1                 0x20
#define LP5523_RUN_ENG2                 0x08
#define LP5523_RUN_ENG3                 0x02

/* Proc Error Type */
#define LP5523_ERR_PROC_CHIP		1
#define LP5523_ERR_PROC_INIT		2
#define LP5523_ERR_PROC_DEF_LED		3
#define LP5523_ERR_SET_PATTERN		4
#define LP5523_ERR_SET_LED		5
#define LP5523_ERR_GET_PATTERN		6

#define LP5523_ENG1_IS_LOADING(mode)    \
	((mode & LP5523_MODE_ENG1_M) == LP5523_LOAD_ENG1)
#define LP5523_ENG2_IS_LOADING(mode)    \
	((mode & LP5523_MODE_ENG2_M) == LP5523_LOAD_ENG2)
#define LP5523_ENG3_IS_LOADING(mode)    \
	((mode & LP5523_MODE_ENG3_M) == LP5523_LOAD_ENG3)

#define LED_ACTIVE(mux, led)            (!!(mux & (0x0001 << led)))
#define sleep(a)        udelay(a * 1000)        //ms=1000

#define DEBUG				0

struct lp55xx_engine {
	int id;
	int mode;
	char *pattern;
	char *led;
};

struct lp55xx_blink_leds_pattern {
	int ptn_mode;
	char *blink_pattern1;
	char *blink_pattern2;
	char *blink_pattern3;
	char *leds_pattern1;
	char *leds_pattern2;
	char *leds_pattern3;
};

enum lp55xx_engine_index {
        LP55XX_ENGINE_INVALID = 0,
        LP55XX_ENGINE_1,
        LP55XX_ENGINE_2,
        LP55XX_ENGINE_3,
        LP55XX_ENGINE_MAX = LP55XX_ENGINE_3 +1,
};

enum lp55xx_engine_mode {
        LP55XX_ENGINE_DISABLED,
        LP55XX_ENGINE_LOAD,
        LP55XX_ENGINE_RUN,
};

enum lp55xx_blink_leds_mode {
	LP55XX_INIT_LEDS = 0,
	LP55XX_NONE_LEDS,
	LP55XX_RESCUE_LEDS,
	LP55XX_RESCUE_RCV_DATA_LEDS,
	LP55XX_RESET_LEDS,
	LP55XX_END,
};

extern void lp5523_leds_proc(int ptn_mode);
extern int lp55xx_write(u8 addr, u8 val);
extern int lp55xx_read(u8 addr);
extern int lp55xx_update_bits(u8 reg, u8 mask, u8 val);
#endif /* _LEDS_LP5523_H */
