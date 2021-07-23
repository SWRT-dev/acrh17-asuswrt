/*
 * lp5523.c - LP5523 LED Driver
 *
 * Copyright (C) 2010 Nokia Corporation
 * Copyright (C) 2012 Texas Instruments
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *          Milo(Woogyom) Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#include <leds_lp5523.h>

static uchar chip_addr;

/*
 * Max Pattern size: 32bytes.
 * Led Patten: BBBGGGRRR
 *
 * slow blink:	"7e004200"	= 499.2ms
 * fast blink:	"5000"		= 124.8ms
*/
struct lp55xx_blink_leds_pattern lp55xx_blnk_leds_ptn[] = {
#if 1
	{ LP55XX_INIT_LEDS, "9d80400044101ef042001ff04510", "9d80440044101af042001bf04510", "9d80480044101cf042001df04510", "111000000", "000111000", "000000111" },
#else /* for Hydra running RT-AC58U firmware, special edition */
	{ LP55XX_INIT_LEDS, "9d804030", "", "", "111000000", "", "" }, /* low power blue */
#endif
	{ LP55XX_NONE_LEDS, "9d804000", "9d804000", "9d804000", "111111111", "", "" },
	{ LP55XX_RESCUE_LEDS, "9d8040ff", "9d804000", "9d80405a", "111000000", "000111000", "000000111" }, // purple
	{ LP55XX_RESCUE_RCV_DATA_LEDS, "9d8040ff7e00420040007e004200", "9d8040007e00420040ff7e004200", "9d80405a7e00420040007e004200", "111000000", "000111000", "000000111" }, // slow blink - purple+green
	//{ LP55XX_RESET_LEDS, "9d8040b47e00420040007e004200", "", "", "000000111", "", "" }, // slow  blink - red
	{ LP55XX_RESET_LEDS, "9d8040b4500040005000", "", "", "000000111", "", "" }, // fast blink - red
	{ LP55XX_END, NULL, NULL, NULL, NULL, NULL, NULL }
};

static int chip_probe(void)
{
	int i, addr=0;

	for (i = 0; i < 128; i++) {
		if (i2c_probe(i) == 0)
			addr = i;
	}

	return addr;
}

int lp55xx_write(u8 addr, u8 val)
{
	int alen=1, len=1;
	u8 buf = val;

	if(DEBUG) printf("i2c_write, chip[%2x], addr[%2x], val[%2x]\n", chip_addr, addr, buf);
	return i2c_write(chip_addr, addr, alen, &buf, len);
}

int lp55xx_read(u8 addr)
{
	int alen=1, len=1;
	u8 buf;

	i2c_read(chip_addr, addr, alen, &buf, len);
	if(DEBUG) printf("i2c_read, chip[%2x], addr[%2x], val[%2x]\n", chip_addr, addr, buf);

	return buf;
}

int lp55xx_update_bits(u8 reg, u8 mask, u8 val)
{
	u8 tmp;

	tmp = (u8)lp55xx_read(reg);

	tmp &= ~mask;
	tmp |= val & mask;

	return lp55xx_write(reg, tmp);
}

static int lp5523_update_program_memory(char *pattern, int size)
{
	char cmd[3];
	int i = 0;

	if ((size%4 != 0) || (size/2 > LP5523_PROGRAM_LENGTH))
		goto err;

	while (i < LP5523_PROGRAM_LENGTH) {
		memset(cmd, '\0', sizeof(cmd)); 

		if (i < size/2) {
			strncpy(cmd, pattern, 2);
			cmd[3] = '\0';

			pattern += 2;
		}

		if(lp55xx_write(LP5523_REG_PROG_MEM + i, (int)simple_strtol(cmd, NULL, 16)))
			goto err;

		i++;
	}

	return 0;

err:
	return LP5523_ERR_SET_PATTERN;
}

static void lp5523_stop_all_engines(void)
{
        lp55xx_write(LP5523_REG_OP_MODE, 0);
	sleep(2);
}

static void lp5523_stop_engine(int engine_idx)
{
	enum lp55xx_engine_index idx = engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	lp55xx_update_bits(LP5523_REG_OP_MODE, mask[idx], 0);
	sleep(2);
}

static void lp5523_load_engine(int engine_idx)
{
	enum lp55xx_engine_index idx = engine_idx;
	u8 mask[] = {
		[LP55XX_ENGINE_1] = LP5523_MODE_ENG1_M,
		[LP55XX_ENGINE_2] = LP5523_MODE_ENG2_M,
		[LP55XX_ENGINE_3] = LP5523_MODE_ENG3_M,
	};

	u8 val[] = {
		[LP55XX_ENGINE_1] = LP5523_LOAD_ENG1,
		[LP55XX_ENGINE_2] = LP5523_LOAD_ENG2,
		[LP55XX_ENGINE_3] = LP5523_LOAD_ENG3,
	};

	lp55xx_update_bits(LP5523_REG_OP_MODE, mask[idx], val[idx]);
	sleep(2);
}

static void lp5523_load_engine_and_select_page(int engine_idx)
{
	enum lp55xx_engine_index idx = engine_idx;
	u8 page_sel[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_ENG1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_ENG2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_ENG3,
	};

	lp5523_load_engine(engine_idx);

	lp55xx_write(LP5523_REG_PROG_PAGE_SEL, page_sel[idx]);
}

static int lp5523_set_led_current(u8 led_current)
{
	int i;

	for (i = 0; i < LP5523_MAX_LEDS; i++)
		if (lp55xx_write(LP5523_REG_LED_CURRENT_BASE + i, led_current))
			return LP5523_ERR_PROC_DEF_LED;
	return 0;
}

static int lp5523_set_led_PWM(u8 led_PWM)
{
	int i;

	for (i = 0; i < LP5523_MAX_LEDS; i++)
		if (lp55xx_write(LP5523_REG_LED_PWM_BASE + i, led_PWM))
			return 1;
	return 0;
}

static void lp5523_run_engine(int start)
{
	u8 mode, exec;
	/* stop engine */
	if (!start) {
		int i;
		for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) 
			lp5523_stop_engine(i);
		
		if (lp5523_set_led_PWM(0x00))
			printf("Set led PWM to default failed !!\n");

		printf("Stop all engine !!\n");

		return;
	}

	/*
	* To run the engine,
	* operation mode and enable register should updated at the same time
	*/

	mode = (unsigned int)lp55xx_read(LP5523_REG_OP_MODE);
	exec = (unsigned int)lp55xx_read(LP5523_REG_ENABLE);

	/* change operation mode to RUN only when each engine is loading */
	if (LP5523_ENG1_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG1_M) | LP5523_RUN_ENG1;
		exec = (exec & ~LP5523_EXEC_ENG1_M) | LP5523_RUN_ENG1;
	}

	if (LP5523_ENG2_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG2_M) | LP5523_RUN_ENG2;
		exec = (exec & ~LP5523_EXEC_ENG2_M) | LP5523_RUN_ENG2;
	}

	if (LP5523_ENG3_IS_LOADING(mode)) {
		mode = (mode & ~LP5523_MODE_ENG3_M) | LP5523_RUN_ENG3;
		exec = (exec & ~LP5523_EXEC_ENG3_M) | LP5523_RUN_ENG3;
	}


	lp55xx_write(LP5523_REG_OP_MODE, mode);
	sleep(2);

	lp55xx_update_bits(LP5523_REG_ENABLE, LP5523_EXEC_M, exec);
}

static int lp5523_load_mux(int engine_idx, u16 mux)
{
	int ret;
	u8 mux_page[] = {
		[LP55XX_ENGINE_1] = LP5523_PAGE_MUX1,
		[LP55XX_ENGINE_2] = LP5523_PAGE_MUX2,
		[LP55XX_ENGINE_3] = LP5523_PAGE_MUX3,
	};

	lp5523_load_engine(engine_idx);

	ret = lp55xx_write(LP5523_REG_PROG_PAGE_SEL, mux_page[engine_idx]);
	if (ret)
		return ret;

	ret = lp55xx_write(LP5523_REG_PROG_MEM , (u8)(mux >> 8));
	if (ret)
		return ret;

	ret = lp55xx_write(LP5523_REG_PROG_MEM + 1, (u8)(mux));
	if (ret)
		return ret;

	return 0;
}

static int lp5523_mux_parse(const char *buf, u16 *mux, int size)
{
	u16 tmp_mux = 0;
	int i;

	size = min_t(int, size, LP5523_MAX_LEDS);

	for (i = 0; i < size; i++) {
		switch (buf[i]) {
		case '1':
			tmp_mux |= (1 << i);
			break;
		case '0':
			break;
		case '\n':
			i = size;
			break;
		default:
			return 1;
		}
	}
	*mux = tmp_mux;

	return 0;
}

static int store_engine_leds(struct lp55xx_engine *engine)
{
	u16 mux = 0;

	if (lp5523_mux_parse(engine->led, &mux, strlen(engine->led)))
		goto err;

	if (engine->mode != LP55XX_ENGINE_LOAD)
		goto err;

	if (lp5523_load_mux(engine->id, mux))
		goto err;

	return 0;
err:
	return LP5523_ERR_SET_LED;
}

static int store_engine_load(struct lp55xx_engine *engine)
{
	lp5523_load_engine_and_select_page(engine->id);
	return lp5523_update_program_memory(engine->pattern, strlen(engine->pattern));
}

static void store_engine_mode(struct lp55xx_engine *engine, const char *buf)
{
	if (!strncmp(buf, "run", 3)) {
		lp5523_run_engine(1);
		engine->mode = LP55XX_ENGINE_RUN;
	} 
	else if (!strncmp(buf, "load", 4)) {
		lp5523_stop_engine(engine->id);
		lp5523_load_engine(engine->id); 
		engine->mode = LP55XX_ENGINE_LOAD;
	}
	else if (!strncmp(buf, "disabled", 8)) 	{
		lp5523_stop_engine(engine->id);
		engine->mode = LP55XX_ENGINE_DISABLED;
	}
}

static int lp5523_init_program_engine(void)
{
	int i, j;
	int ret = LP5523_ERR_PROC_INIT;
	u8 status;
	/* one pattern per engine setting LED MUX start and stop addresses */
	static const u8 pattern[][LP5523_PROGRAM_LENGTH] =  {
		{ 0x9c, 0x30, 0x9c, 0xb0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x40, 0x9c, 0xc0, 0x9d, 0x80, 0xd8, 0x00, 0},
		{ 0x9c, 0x50, 0x9c, 0xd0, 0x9d, 0x80, 0xd8, 0x00, 0},
	};

	/* hardcode 32 bytes of memory for each engine from program memory */
	if (lp55xx_write(LP5523_REG_CH1_PROG_START, 0x00))
		goto err;
if (lp55xx_write(LP5523_REG_CH2_PROG_START, 0x10)) goto err; 
	if (lp55xx_write(LP5523_REG_CH3_PROG_START, 0x20))
		goto err;

	/* write LED MUX address space for each engine */
	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		lp5523_load_engine_and_select_page(i);

		for (j = 0; j < LP5523_PROGRAM_LENGTH; j++) {
			if (lp55xx_write(LP5523_REG_PROG_MEM + j, pattern[i - 1][j]))
				goto err;
		}
	}

	lp5523_run_engine(1);
	sleep(6);
	status = (unsigned int)lp55xx_read(LP5523_REG_STATUS);
	status &= LP5523_ENG_STATUS_MASK;
	if (status != LP5523_ENG_STATUS_MASK) {
		printf("cound not configure LED engine, status = 0x%.2x\n", status);
		goto err;
	}

	ret = 0;
err:
	lp5523_stop_all_engines();
	return ret;
}

static int lp5523_init_device(void)
{
	if (lp55xx_write(LP5523_REG_RESET, LP5523_RESET))
		goto err;

	if (lp55xx_write(LP5523_REG_ENABLE, LP5523_ENABLE))
		goto err;

	sleep(2);

	if (lp55xx_write(LP5523_REG_CONFIG, LP5523_AUTO_INC | LP5523_PWR_SAVE | LP5523_CP_AUTO | LP5523_AUTO_CLK | LP5523_PWM_PWR_SAVE))
		goto err;

	/* turn on all leds */
	if (lp55xx_write(LP5523_REG_ENABLE_LEDS_MSB, 0x01))
		goto err;

	if (lp55xx_write(LP5523_REG_ENABLE_LEDS_LSB, 0xff))
		goto err;

	return lp5523_init_program_engine();

err:
	return LP5523_ERR_PROC_INIT;
}

int lp5523_set_engine(struct lp55xx_engine *engine, int ptn_mode)
{
	struct lp55xx_blink_leds_pattern *blnk_leds_ptn = lp55xx_blnk_leds_ptn;
	int i;

	for (; blnk_leds_ptn->ptn_mode!=LP55XX_END; blnk_leds_ptn++) {	
		if (ptn_mode == blnk_leds_ptn->ptn_mode) {
			for (i = LP55XX_ENGINE_INVALID; i <= LP55XX_ENGINE_3; i++) { 
				engine[i].id = i;
				engine[i].mode = LP55XX_ENGINE_DISABLED;
				if (i == LP55XX_ENGINE_INVALID) {
					engine[i].pattern = "";
					engine[i].led = "";
				}
				else if (i == LP55XX_ENGINE_1) {
					engine[i].pattern = blnk_leds_ptn->blink_pattern1;
					engine[i].led = blnk_leds_ptn->leds_pattern1;
				}
				else if (i == LP55XX_ENGINE_2) {
					engine[i].pattern = blnk_leds_ptn->blink_pattern2;
					engine[i].led = blnk_leds_ptn->leds_pattern2;
				}
				else if (i == LP55XX_ENGINE_3) {
					engine[i].pattern = blnk_leds_ptn->blink_pattern3;
					engine[i].led = blnk_leds_ptn->leds_pattern3;
				}
			}
			return 0;
		}
	}

	return LP5523_ERR_GET_PATTERN;
}

int lp5523_blink_leds(struct lp55xx_engine *engine)
{
	int err_proc=0, i;

	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++)
		store_engine_mode(&engine[i], "disabled");

	for (i = LP55XX_ENGINE_1; i <= LP55XX_ENGINE_3; i++) {
		store_engine_mode(&engine[i], "load");

		err_proc = store_engine_load(&engine[i]);
		if (err_proc)
			return err_proc;

		err_proc = store_engine_leds(&engine[i]);
		if (err_proc)
			return err_proc;
	}

	store_engine_mode(&engine[LP55XX_ENGINE_1], "run");

	return err_proc;
}

void lp5523_leds_main(int ptn_mode)
{
	struct lp55xx_engine engine[LP5523_MAX_ENGINES +1];
	int err_proc=0;

	if (ptn_mode == LP55XX_INIT_LEDS) {
		chip_addr = chip_probe();
		if (chip_addr==0){
			err_proc = LP5523_ERR_PROC_CHIP;
			goto err_msg;
		}
		else
			printf("LP5523 Chip addresses: %02X \n", chip_addr);

		err_proc = lp5523_init_device();
		if (err_proc)
			goto err_msg;

		err_proc = lp5523_set_led_current(0xfa);
		if (err_proc)
			goto err_msg;
	}

	err_proc = lp5523_set_engine(engine, ptn_mode);
	if (err_proc)
		goto err_msg;

	err_proc = lp5523_blink_leds(engine);
	if (err_proc)
		goto err_msg;

err_msg:
	switch (err_proc){

	case LP5523_ERR_PROC_CHIP:
		printf("Get chip addr failed !!\n");
		break;
	case LP5523_ERR_PROC_INIT:
		printf("Init device failed !!\n");
		break;	
	case LP5523_ERR_PROC_DEF_LED:
		printf("Set default Led PWM failed !!\n");
		break;	
	case LP5523_ERR_SET_PATTERN:
		printf("Set ENGX pattern failed !!\n");
		break;	
	case LP5523_ERR_SET_LED:
		printf("Set ENGX led failed !!\n");
		break;	
	case LP5523_ERR_GET_PATTERN:
		printf("Get ENGX pattern failed !!\n");
		break;	
		
	default:
		break;
	}
}

void lp5523_leds_proc(int ptn_mode)
{
	lp5523_leds_main(LP55XX_NONE_LEDS);
	if (ptn_mode != LP55XX_NONE_LEDS)
		lp5523_leds_main(ptn_mode);
}
