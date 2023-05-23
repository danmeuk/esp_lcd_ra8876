/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2023 Daniel Austin <me@dan.me.uk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_ra8876.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_RA8876_TIMEOUT_US		(30 * 1000)					/*!< How long to wait for panel operations (in uS) */

static const char					*TAG = "ra8876";

static esp_err_t panel_ra8876_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ra8876_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ra8876_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ra8876_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ra8876_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ra8876_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ra8876_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ra8876_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ra8876_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
	esp_lcd_panel_t					base;
	esp_lcd_panel_io_handle_t		io;
	int								wait_gpio_num;				/*!< WAIT gpio pin (or GPIO_NUM_NC if unused) */
	int								reset_gpio_num;				/*!< RST gpio pin (or GPIO_NUM_NC if unused) */
	bool							reset_level;				/*!< RST level required, in case of an inverted RST logic panel */
	int								x_gap;
	int								y_gap;
	unsigned int					bits_per_pixel;				/*!< bpp used (should always be 16) */
	uint16_t						lcd_width;					/*!< LCD width in pixels */
	uint16_t						lcd_height;					/*!< LCD height in pixels */
	uint8_t							chip_config_register;		/*!< CCR value */
	uint8_t							display_config_register;	/*!< PDCR value */
	bool							swap_axes;					/*!< whether to swap X/Y axis */
	uint16_t						backlight_level;			/*! backlight level if using PWM (16-bit integer) */
} ra8876_panel_t;

esp_err_t esp_lcd_new_panel_ra8876(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
	esp_err_t						ret = ESP_OK;
	ra8876_panel_t					*ra8876 = NULL;
	esp_lcd_panel_ra8876_config_t	*vendor_cfg = (esp_lcd_panel_ra8876_config_t *) panel_dev_config->vendor_config;

	ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
	ESP_GOTO_ON_FALSE(vendor_cfg, ESP_ERR_INVALID_ARG, err, TAG, "vendor config cannot be null");

	ra8876 = calloc(1, sizeof(ra8876_panel_t));
	ESP_GOTO_ON_FALSE(ra8876, ESP_ERR_NO_MEM, err, TAG, "no mem for ra8876 panel");

	/* RST GPIO config (if applicable) */
	if (panel_dev_config->reset_gpio_num >= GPIO_NUM_0)
	{
		gpio_config_t io_conf =
		{
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
		};
		ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
	}

	/* WAIT GPIO config (if applicable) */
	if (vendor_cfg->wait_gpio_num >= GPIO_NUM_0)
	{
		gpio_config_t io_conf =
		{
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = 1ULL << vendor_cfg->wait_gpio_num,
			.pull_up_en = GPIO_PULLUP_ENABLE,
		};
		ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for WAIT line failed");
	}

	/* Setup default CCR and PDCR values (both 0x00 by default) */
	ra8876->chip_config_register = 0x00;
	ra8876->display_config_register = 0x00;
	
	// Note: Color space used for compatibility with IDF v4.4
	switch (panel_dev_config->color_space)
	{
	case ESP_LCD_COLOR_SPACE_RGB:
		break;
	default:
		ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space, only RGB supported.");
		break;
	}

	switch (panel_dev_config->bits_per_pixel)
	{
	case 16:
		break;
	default:
		ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width (supports 16-bit 5:6:5 format only)");
		break;
	}

	switch (vendor_cfg->mcu_bit_interface)
	{
	case 8:
		/* default value of 0x00 is for 8-bit */
		break;
	case 16:
		ra8876->chip_config_register |= 0x01;
		break;
	default:
		ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported MCU interface width (supports only 8-bit and 16-bit");
		break;
	}

	ra8876->io = io;
	ra8876->lcd_width = vendor_cfg->lcd_width;
	ra8876->lcd_height = vendor_cfg->lcd_height;
	ra8876->wait_gpio_num = vendor_cfg->wait_gpio_num;
	ra8876->bits_per_pixel = panel_dev_config->bits_per_pixel;
	ra8876->reset_gpio_num = panel_dev_config->reset_gpio_num;
	ra8876->reset_level = panel_dev_config->flags.reset_active_high;
	ra8876->base.del = panel_ra8876_del;
	ra8876->base.reset = panel_ra8876_reset;
	ra8876->base.init = panel_ra8876_init;
	ra8876->base.draw_bitmap = panel_ra8876_draw_bitmap;
	ra8876->base.invert_color = panel_ra8876_invert_color;
	ra8876->base.set_gap = panel_ra8876_set_gap;
	ra8876->base.mirror = panel_ra8876_mirror;
	ra8876->base.swap_xy = panel_ra8876_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
	ra8876->base.disp_off = panel_ra8876_disp_on_off;
#else
	ra8876->base.disp_on_off = panel_ra8876_disp_on_off;
#endif
	*ret_panel = &(ra8876->base);
	ESP_LOGD(TAG, "new ra8876 panel @%p", ra8876);

	return ESP_OK;

err:
	if (ra8876)
	{
		/* tidy up RST and/or WAIT gpio pins */
		if (ra8876->reset_gpio_num >= GPIO_NUM_0)
			gpio_reset_pin(ra8876->reset_gpio_num);
		if (ra8876->wait_gpio_num >= GPIO_NUM_0)
			gpio_reset_pin(ra8876->wait_gpio_num);
		free(ra8876);
	}
	return ret;
}

static esp_err_t panel_ra8876_del(esp_lcd_panel_t *panel)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

	if (ra8876->reset_gpio_num >= GPIO_NUM_0)
		gpio_reset_pin(ra8876->reset_gpio_num);
	if (ra8876->wait_gpio_num >= GPIO_NUM_0)
		gpio_reset_pin(ra8876->wait_gpio_num);

	ESP_LOGD(TAG, "del ra8876 panel @%p", ra8876);
	free(ra8876);
	return ESP_OK;
}

static esp_err_t panel_ra8876_reset(esp_lcd_panel_t *panel)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	esp_lcd_panel_io_handle_t		io = ra8876->io;

	if (ra8876->reset_gpio_num >= 0)
	{
		/* perform hardware reset */
		gpio_set_level(ra8876->reset_gpio_num, ra8876->reset_level);
		vTaskDelay(pdMS_TO_TICKS(50));
		gpio_set_level(ra8876->reset_gpio_num, !ra8876->reset_level);
		vTaskDelay(pdMS_TO_TICKS(50));
	} else {
		/* perform software reset */
		esp_lcd_panel_io_tx_param(io, RA8876_REG_SRR, (uint8_t[]){ 0x01 }, 1);
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	return ESP_OK;
}

static void panel_ra8876_wait(esp_lcd_panel_t *panel)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	uint64_t						start = esp_timer_get_time();
	uint64_t						now = start;

	if (ra8876->wait_gpio_num >= GPIO_NUM_0)
	{
		while (gpio_get_level(ra8876->wait_gpio_num) == 0 && ((now = esp_timer_get_time()) - start) < ESP_RA8876_TIMEOUT_US);
		if ((now - start) > ESP_RA8876_TIMEOUT_US)
		{
			ESP_LOGE(TAG, "RA8876 Timeout!");
			//            ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
		}
	}
}

static esp_err_t panel_ra8876_tx_param(esp_lcd_panel_t *panel, int lcd_cmd, uint8_t param)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	esp_lcd_panel_io_handle_t		io = ra8876->io;

	panel_ra8876_wait(panel);
	return esp_lcd_panel_io_tx_param(io, lcd_cmd, (uint8_t[]) {	param }, 1);
}

static esp_err_t panel_ra8876_init(esp_lcd_panel_t *panel)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	uint8_t							val;

	/* perform hardware reset */
	vTaskDelay(pdMS_TO_TICKS(100));
	panel_ra8876_reset(panel);
	vTaskDelay(pdMS_TO_TICKS(500));
	/* perform software reset */
	panel_ra8876_tx_param(panel, RA8876_REG_SRR, 0x01);
	panel_ra8876_wait(panel);

	/* set pixel clock */
#if (RA8876_SCAN_FREQ >= 63)
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC1, RA8876_PLL_DIV_4);
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC2, (RA8876_SCAN_FREQ * 4 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_SCAN_FREQ >= 32) && (RA8876_SCAN_FREQ <= 62)
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC1, RA8876_PLL_DIV_8);
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC2, (RA8876_SCAN_FREQ * 8 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_SCAN_FREQ >= 16) && (RA8876_SCAN_FREQ <= 31)
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC1, RA8876_PLL_DIV_16);
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC2, (RA8876_SCAN_FREQ * 16 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_SCAN_FREQ >= 8) && (RA8876_SCAN_FREQ <= 15)
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC1, RA8876_PLL_DIV_32);
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC2, (RA8876_SCAN_FREQ * 32 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_SCAN_FREQ >= 0) && (RA8876_SCAN_FREQ <= 7)
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC1, RA8876_PLL_DIV_64);
	panel_ra8876_tx_param(panel, RA8876_REG_PPLLC2, (RA8876_SCAN_FREQ * 64 / RA8876_OSC_FREQ) - 1);
#endif

	/* set sdram clock */
#if (RA8876_DRAM_FREQ >= 125)
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC1, RA8876_PLL_DIV_2);
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC2, (RA8876_DRAM_FREQ * 2 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_DRAM_FREQ >= 63) && (RA8876_DRAM_FREQ <= 124)
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC1, RA8876_PLL_DIV_4);
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC2, (RA8876_DRAM_FREQ * 4 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_DRAM_FREQ >= 31) && (RA8876_DRAM_FREQ <= 62)
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC1, RA8876_PLL_DIV_8);
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC2, (RA8876_DRAM_FREQ * 8 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_DRAM_FREQ <= 30)
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC1, RA8876_PLL_DIV_8);
	panel_ra8876_tx_param(panel, RA8876_REG_MPLLC2, (30 * 8 / RA8876_OSC_FREQ) - 1);
#endif

	/* set core clock */
#if (RA8876_CORE_FREQ >= 125)
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC1, RA8876_PLL_DIV_2);
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC2, (RA8876_CORE_FREQ * 2 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_CORE_FREQ >= 63) && (RA8876_CORE_FREQ <= 124)
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC1, RA8876_PLL_DIV_4);
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC2, (RA8876_CORE_FREQ * 4 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_CORE_FREQ >= 31) && (RA8876_CORE_FREQ <= 62)
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC1, RA8876_PLL_DIV_8);
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC2, (RA8876_CORE_FREQ * 8 / RA8876_OSC_FREQ) - 1);
#endif
#if (RA8876_CORE_FREQ <= 30)
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC1, RA8876_PLL_DIV_8);
	panel_ra8876_tx_param(panel, RA8876_REG_SPLLC2, (30 * 8 / RA8876_OSC_FREQ) - 1);
#endif

	/* reconfigure PLLs */
	panel_ra8876_tx_param(panel, RA8876_REG_CCR, 0x00);
	vTaskDelay(pdMS_TO_TICKS(10));
	panel_ra8876_tx_param(panel, RA8876_REG_CCR, 0x80);		// 0x80 (bit 7) = Reconfigure PLL frequency
	vTaskDelay(pdMS_TO_TICKS(10));

	/* SDRAM (W9812G6JH) */
	panel_ra8876_tx_param(panel, RA8876_REG_SDRAR, 0x29);	// 64Mb/8MB/4Mx16: 4 banks, 4096 rows, 512 cols
	panel_ra8876_tx_param(panel, RA8876_REG_SDRMD, 0x03);	// CAS:2=0x02; ACAS:3=0x03
	uint16_t sdram_itv = ((64000000 / 8192) / (1000 / RA8876_DRAM_FREQ)) - 2;
	panel_ra8876_tx_param(panel, RA8876_REG_SDR_REF_ITVL0, sdram_itv & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_SDR_REF_ITVL1, (sdram_itv >> 8) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_SDRCR, 0x01);	// 0x01 (bit 0) = initialise SDRAM
	vTaskDelay(pdMS_TO_TICKS(10));

	/* set chip configuration register */
	panel_ra8876_tx_param(panel, RA8876_REG_CCR, ra8876->chip_config_register);	// set tft panel 24bit, host bus 8/16bit

	/* set memory access control register */
	val = 0;
	val |= 0b01000000;	// bits 7-6 = image data format (16bit to 16bit)
	/* bits 2-1 = host write memory direction (00 = L-R,T-B, 01 = R-L,T-B, 10 = T-B,L-R, 11 = B-T,L-R */
	panel_ra8876_tx_param(panel, RA8876_REG_MACR, val);

	/* set input control register */
	val = 0;
	// bit 7 = interrupt pin level, 0 = active low, 1 = active high
	// bit 2 = text mode enable, 0 = graphic mode, 1 = text mode
	// bits 1-0 = memory select, 00 = sdram, 01 = gamma table, 10 = graphic cursor ram, 11 = color palette ram
	panel_ra8876_tx_param(panel, RA8876_REG_ICR, val);

	/* set display configuration register */
	ra8876->display_config_register |= 0b10000000;
	// bit 7 = pclk inversion, 0 = rising edge, 1 = falling edge
	panel_ra8876_tx_param(panel, RA8876_REG_DPCR, ra8876->display_config_register);

	/* set panel scan clock and data setting register */
	val = 0b00000011;
	// bit 7 = hsync polarity, 0 = low active, 1 = high active
	val |= 0b10000000;
	// bit 6 = vsync polarity, 0 = low active, 1 = high active
	val |= 0b01000000;
	panel_ra8876_tx_param(panel, RA8876_REG_PCSR, val);
	
	/* set horizontal display width register
	 * set horizontal non-display period register
	 */
	val = (ra8876->lcd_width / 8) - 1;
	panel_ra8876_tx_param(panel, RA8876_REG_HDWR, val);
	panel_ra8876_tx_param(panel, RA8876_REG_HNDR, val);
	val = (ra8876->lcd_width % 8);
	panel_ra8876_tx_param(panel, RA8876_REG_HDWFTR, val);
	panel_ra8876_tx_param(panel, RA8876_REG_HNDFTR, val);

	/* set vertical display height register */
	panel_ra8876_tx_param(panel, RA8876_REG_VDHR0, ((ra8876->lcd_height) - 1) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_VDHR1, (((ra8876->lcd_height) - 1) >> 8) & 0xff);

	/* set hsync start position register */
	panel_ra8876_tx_param(panel, RA8876_REG_HSTR, (ra8876->lcd_width / 8) - 1);

	/* set hsync pulse width register */
	panel_ra8876_tx_param(panel, RA8876_REG_HPWR, (ra8876->lcd_width / 8) - 1);

	/* set vertical non-display period register */
	panel_ra8876_tx_param(panel, RA8876_REG_VNDR0, 22);
	panel_ra8876_tx_param(panel, RA8876_REG_VNDR1, 0x00);

	/* set vsync start position register */
	panel_ra8876_tx_param(panel, RA8876_REG_VSTR, 11);

	/* set vsync pulse width register */
	panel_ra8876_tx_param(panel, RA8876_REG_VPWR, 9);

	/* set main/pip window control register */
	val = 0;
	// bits 3-2 = main image colour depth, 00 = 8bpp, 01 = 16bpp, 1x = 24bpp
	val |= 0b00000100;
	panel_ra8876_tx_param(panel, RA8876_REG_MPWCTR, val);

	/* set main image start address */
	panel_ra8876_tx_param(panel, RA8876_REG_MISA0, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_MISA1, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_MISA2, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_MISA3, 0x00);

	/* set main image width */
	panel_ra8876_tx_param(panel, RA8876_REG_MIW0, ra8876->lcd_width & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_MIW1, (ra8876->lcd_width >> 8) & 0xff);

	/* set main window start X/Y */
	panel_ra8876_tx_param(panel, RA8876_REG_MWULX0, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_MWULX1, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_MWULY0, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_MWULY1, 0x00);

	/* set canvas start address */
	panel_ra8876_tx_param(panel, RA8876_REG_CVSSA0, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_CVSSA1, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_CVSSA2, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_CVSSA3, 0x00);

	/* set canvas image width */
	panel_ra8876_tx_param(panel, RA8876_REG_CVS_IMWTH0, ra8876->lcd_width & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_CVS_IMWTH1, (ra8876->lcd_width >> 8) & 0xff);

	/* set active window start X/Y */
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_X0, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_X1, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_Y0, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_Y1, 0x00);

	/* set active window width and height */
	panel_ra8876_tx_param(panel, RA8876_REG_AW_WTH0, ra8876->lcd_width & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AW_WTH1, (ra8876->lcd_width >> 8) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AW_HT0, ra8876->lcd_height & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AW_HT1, (ra8876->lcd_height >> 8) & 0xff);

	/* set color depth of canvas and active window */
	val = 0;
	// bit 2 = canvas addressing mode, 0 = block mode, 1 = linear mode
	// bits 1-0 = color depth, 00 = 8bpp, 01 = 16bpp, 1x = 24bpp
	val |= 0b00000001;
	panel_ra8876_tx_param(panel, RA8876_REG_AW_COLOR, val);

	/* set main/pip window control register */
	val = 0;
	// bits 3-2 = main image colour depth, 00 = 8bpp, 01 = 16bpp, 1x = 24bpp
	val |= 0b00000100;
	panel_ra8876_tx_param(panel, RA8876_REG_MPWCTR, val);

	/* turn display on */
	ra8876->display_config_register |= 0b01000000;
	panel_ra8876_tx_param(panel, RA8876_REG_DPCR, ra8876->display_config_register);
	vTaskDelay(pdMS_TO_TICKS(20));

	/* Enable_PWM0_Interrupt() */
	val = 0x01;		// PWM Timer-0 Interrupt Enable
	panel_ra8876_tx_param(panel, RA8876_REG_INTEN, val);
	/* Clear_PWM0_Interrupt_Flag() */
	val = 0x01;		// Clear PWM0 Interrupt Flag
	panel_ra8876_tx_param(panel, RA8876_REG_INTF, val);
	/* Mask_PWM0_Interrupt_Flag() */
	val = 0x01;		// Mask PWM0 Timer Interrupt Flag
	panel_ra8876_tx_param(panel, RA8876_REG_MINTFR, val);
	/* Select_PWM0_Clock_Divided_By_2()
	 * Select_PWM0()
	 */
	val = 0b00010010;	// Setup PWM Timer-0 Divisor (div 2), PWM0 output PWM Timer 0
	panel_ra8876_tx_param(panel, RA8876_REG_PMUXR, val);
	/* Enable_PWM0_Dead_Zone()
	 * Auto_Reload_PWM0()
	 * Start_PWM0()
	 */
	val = 0b00101011;	// PWM Timer-0 Dead Zone Enable (bit 3),
						// PWM Timer-0 Auto Reload on (bit 1)
						// PWM Timer-0 Start (bit 0)
	panel_ra8876_tx_param(panel, RA8876_REG_PCFGR, val);
	/* Set_Timer0_Compare_Buffer(0x0000) */
	/* set BL level to off */
	panel_ra8876_tx_param(panel, RA8876_REG_TCMPB0L, 0x00);
	panel_ra8876_tx_param(panel, RA8876_REG_TCMPB0H, 0x00);
	ra8876->backlight_level = 0x0000;

	return ESP_OK;
}

static void panel_ra8876_set_window(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

	if (ra8876->swap_axes)
	{
		int xs = x_start;
		int xe = x_end;

		x_start = y_start;
		y_start = xs;

		x_end = y_end;
		y_end = xe;
	}

	/* set active window start X/Y */
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_X0, x_start & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_X1, (x_start >> 8) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_Y0, y_start & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AWUL_Y1, (y_start >> 8) & 0xff);

	/* set active window width and height */
	panel_ra8876_tx_param(panel, RA8876_REG_AW_WTH0, (x_end - x_start) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AW_WTH1, ((x_end - x_start) >> 8) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AW_HT0, (y_end - y_start) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_AW_HT1, ((y_end - y_start) >> 8) & 0xff);
}

static void panel_ra8876_set_cursor(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

	if (ra8876->swap_axes)
	{
		int xs = x_start;

		x_start = y_start;
		y_start = xs;
	}

	panel_ra8876_tx_param(panel, RA8876_REG_CURH0, x_start & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_CURH1, (x_start >> 8) & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_CURV0, y_start & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_CURV1, (y_start >> 8) & 0xff);
}

static esp_err_t panel_ra8876_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	esp_lcd_panel_io_handle_t		io = ra8876->io;
	assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");

	x_start += ra8876->x_gap;
	x_end += ra8876->x_gap;
	y_start += ra8876->y_gap;
	y_end += ra8876->y_gap;

	/* define an area of frame memory where MCU can access */
	panel_ra8876_set_window(panel, x_start, y_start, x_end, y_end);

	/* set cursor */
	panel_ra8876_set_cursor(panel, x_start, y_start, x_end, y_end);

	/* Write to graphic RAM */
	size_t len = (x_end - x_start) * (y_end - y_start) * ra8876->bits_per_pixel / 8;
	esp_lcd_panel_io_tx_color(io, RA8876_REG_MRWDP, color_data, len);

	return ESP_OK;
}

static esp_err_t panel_ra8876_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
	ESP_LOGE(TAG, "invert color is unsupported");
	return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_ra8876_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
	ESP_LOGE(TAG, "mirror is unsupported");
	return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_ra8876_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

	ra8876->swap_axes = swap_axes;

	return ESP_OK;
}

static esp_err_t panel_ra8876_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

	ra8876->x_gap = x_gap;
	ra8876->y_gap = y_gap;
	
	return ESP_OK;
}

static esp_err_t panel_ra8876_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
	on_off = !on_off;
#endif

	if (on_off)
		ra8876->display_config_register |= 0b01000000;
	else
		ra8876->display_config_register &= ~0b01000000;

	panel_ra8876_tx_param(panel, RA8876_REG_DPCR, ra8876->display_config_register);
	vTaskDelay(pdMS_TO_TICKS(20));

	return ESP_OK;
}

esp_err_t esp_lcd_panel_set_backlight(esp_lcd_panel_t *panel, uint8_t level)
{
	esp_err_t						ret = ESP_OK;
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	uint16_t						raw_level;

	raw_level = (0xff * level);
	
	if (raw_level == ra8876->backlight_level)
		return ret;
	
	/* set new backlight level */
	ra8876->backlight_level = raw_level;
	panel_ra8876_tx_param(panel, RA8876_REG_TCMPB0L, raw_level & 0xff);
	panel_ra8876_tx_param(panel, RA8876_REG_TCMPB0H, (raw_level >> 8) & 0xff);

	return ret;
}

#ifdef __cplusplus
}
#endif