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
	uint8_t							macr;						/*!< Memory Access Control Register value */
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
	if (panel_dev_config->reset_gpio_num != GPIO_NUM_NC)
	{
		gpio_config_t io_conf =
		{
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
		};
		ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
	}

	/* WAIT GPIO config (if applicable) */
	if (vendor_cfg->wait_gpio_num != GPIO_NUM_NC)
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
	/* Setup default memory address control register to 16bpp, no rotation */
	ra8876->macr = 0x40;
	
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
	ra8876->reset_gpio_num = panel_dev_config->reset_gpio_num;
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
		if (ra8876->reset_gpio_num != GPIO_NUM_NC)
			gpio_reset_pin(ra8876->reset_gpio_num);
		if (ra8876->wait_gpio_num != GPIO_NUM_NC)
			gpio_reset_pin(ra8876->wait_gpio_num);
		free(ra8876);
	}
	return ret;
}

static esp_err_t panel_ra8876_del(esp_lcd_panel_t *panel)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);

	if (ra8876->reset_gpio_num != GPIO_NUM_NC)
		gpio_reset_pin(ra8876->reset_gpio_num);
	if (ra8876->wait_gpio_num != GPIO_NUM_NC)
		gpio_reset_pin(ra8876->wait_gpio_num);

	ESP_LOGD(TAG, "del ra8876 panel @%p", ra8876);
	free(ra8876);
	return ESP_OK;
}

static esp_err_t panel_ra8876_reset(esp_lcd_panel_t *panel)
{
	ra8876_panel_t					*ra8876 = __containerof(panel, ra8876_panel_t, base);
	esp_lcd_panel_io_handle_t		io = ra8876->io;

	if (ra8876->reset_gpio_num != GPIO_NUM_NC)
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

	vTaskDelay(pdMS_TO_TICKS(100));
	panel_ra8876_reset(panel);
	vTaskDelay(pdMS_TO_TICKS(100));
	/* wait for reset to complete */
	panel_ra8876_wait(panel);

	/* perform soft reset */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SRR, (uint8_t[]) { 0x01 }, 1);
	panel_ra8876_wait(panel);

	/* set pixel clock */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_PPLLC1, (uint8_t[]) { 0x06 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_PPLLC2, (uint8_t[]) { (RA8876_SCAN_FREQ * 8 / RA8876_OSC_FREQ) - 1 }, 1);
	/* set SDRAM clock */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MPLLC1, (uint8_t[]) { 0x04 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MPLLC2, (uint8_t[]) { (RA8876_DRAM_FREQ * 4 / RA8876_OSC_FREQ) - 1 }, 1);
	/* set core clock */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SPLLC1, (uint8_t[]) { 0x04 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SPLLC2, (uint8_t[]) { (RA8876_CORE_FREQ * 4 / RA8876_OSC_FREQ) - 1 }, 1);
	/* reconfigure PLL generator */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CCR, (uint8_t[]) { 0x00 }, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CCR, (uint8_t[]) { 0x80 }, 1);
	vTaskDelay(pdMS_TO_TICKS(1));

	/* configure SDRAM */
	uint16_t sdram_itv = ((64000000 / 8192) / (1000 / RA8876_DRAM_FREQ)) - 2;
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SDRAR, (uint8_t[]) { 0x31 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SDRMD, (uint8_t[]) { 0x03 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SDR_REF_ITVL0, (uint8_t[]) { sdram_itv & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SDR_REF_ITVL1, (uint8_t[]) { sdram_itv >> 8 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_SDRCR, (uint8_t[]) { 0x01 }, 1);
	/* wait for SDRAM to become ready */
	panel_ra8876_wait(panel);
	vTaskDelay(pdMS_TO_TICKS(1));

	/* set chip config register */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CCR, (uint8_t[]) { ra8876->chip_config_register }, 1);

	/* configure memory address control register */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MACR, (uint8_t[]) { ra8876->macr }, 1);

	/* set graphic mode */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_ICR, (uint8_t[]) { 0x00 }, 1);

	/* set display configuration register */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_DPCR, (uint8_t[]) { ra8876->display_config_register }, 1);

	/* set HSYNC+VSYNC+DE high active */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_PCSR, (uint8_t[]) { 0xc0 }, 1);

	/* set LCD width */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_HDWR, (uint8_t[]) { (ra8876->lcd_width / 8) - 1 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_HDWFTR, (uint8_t[]) { ra8876->lcd_width % 8 }, 1);

	/* set LCD height */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_VDHR0, (uint8_t[]) { (ra8876->lcd_height - 1) & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_VDHR1, (uint8_t[]) { (ra8876->lcd_height - 1) >> 8 }, 1);

	/* set horizontal non-display period / back porch */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_HNDR, (uint8_t[]) { (RA8876_PANEL_HNDR / 8) - 1 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_HNDFTR, (uint8_t[]) { RA8876_PANEL_HNDR % 8 }, 1);

	/* set horizontal start position / front porch */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_HSTR, (uint8_t[]) { (RA8876_PANEL_HSTR / 8) - 1 }, 1);

	/* set HSYNC pulse width */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_HPWR, (uint8_t[]) { (RA8876_PANEL_HPWR / 8) - 1 }, 1);

	/* set vertical non-display period */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_VNDR0, (uint8_t[]) { (RA8876_PANEL_VNDR - 1) & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_VNDR1, (uint8_t[]) { (RA8876_PANEL_VNDR - 1) >> 8 }, 1);

	/* set vertical start position */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_VSTR, (uint8_t[]) { (RA8876_PANEL_VSTR - 1) }, 1);

	/* set VSYNC pulse width */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_VPWR, (uint8_t[]) { (RA8876_PANEL_VPWR - 1) }, 1);

	/* set panel to PIP disabled, 16bpp TFT (65k colours) */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MPWCTR, (uint8_t[]) { 0x04 }, 1);

	/* set main image start address to start of sdram */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MISA0, (uint8_t[]) { 0x00 & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MISA1, (uint8_t[]) { 0x00 >> 8 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MISA2, (uint8_t[]) { 0x00 >> 16 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MISA3, (uint8_t[]) { 0x00 >> 24 }, 1);

	/* set main image width to panel width */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MIW0, (uint8_t[]) { ra8876->lcd_width & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MIW1, (uint8_t[]) { ra8876->lcd_width >> 8 }, 1);

	/* set main window start coordinates to 0x0 */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MWULX0, (uint8_t[]) { 0x00 & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MWULX1, (uint8_t[]) { 0x00 >> 8 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MWULY0, (uint8_t[]) { 0x00 & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MWULY1, (uint8_t[]) { 0x00 >> 8 }, 1);

	/* set canvas image start address to start of sdram */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CVSSA0, (uint8_t[]) { 0x00 & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CVSSA1, (uint8_t[]) { 0x00 >> 8 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CVSSA2, (uint8_t[]) { 0x00 >> 16 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CVSSA3, (uint8_t[]) { 0x00 >> 24 }, 1);

	/* set canvas image width to panel width */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CVS_IMWTH0, (uint8_t[]) { ra8876->lcd_width & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_CVS_IMWTH1, (uint8_t[]) { ra8876->lcd_width >> 8 }, 1);

	/* set top left corner of active window to 0,0 */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AWUL_X0, (uint8_t[]) { 0x00 & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AWUL_X1, (uint8_t[]) { 0x00 >> 8 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AWUL_Y0, (uint8_t[]) { 0x00 & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AWUL_Y1, (uint8_t[]) { 0x00 >> 8 }, 1);

	/* width active window to full panel width and height */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AW_WTH0, (uint8_t[]) { ra8876->lcd_width & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AW_WTH1, (uint8_t[]) { ra8876->lcd_width >> 8 }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AW_HT0, (uint8_t[]) { ra8876->lcd_height & 0xff }, 1);
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AW_HT1, (uint8_t[]) { ra8876->lcd_height >> 8 }, 1);

	/* configure block mode, and 16bpp memory mode */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_AW_COLOR, (uint8_t[]) { 0x01 }, 1);

	/* set panel to PIP disabled, 16bpp TFT (65k colours) */
	esp_lcd_panel_io_tx_param(ra8876->io, RA8876_REG_MPWCTR, (uint8_t[]) { 0x04 }, 1);

	return ESP_OK;
}

static void panel_ra8876_set_window(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
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
	size_t len = (x_end - x_start) * (y_end - y_start) * 2;
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
	ESP_LOGE(TAG, "swap_xy is unsupported");
	return ESP_ERR_NOT_SUPPORTED;
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