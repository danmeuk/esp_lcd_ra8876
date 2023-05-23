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
#pragma once

#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "ra8876_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	RA8876_OSC_FREQ				10		/*!< crystal clock (MHz) */
#define	RA8876_DRAM_FREQ			100		/*!< SDRAM clock frequency (MHz) */
#define RA8876_CORE_FREQ			100		/*!< core (system) clock frequency (MHz) */
#define RA8876_SCAN_FREQ			50		/*!< pixel scan clock frequency (MHz) */

#define RA8876_PLL_DIV_2			0x02	/*!< PLL divided by 2 */
#define RA8876_PLL_DIV_4			0x04	/*!< PLL divided by 4 */
#define RA8876_PLL_DIV_8			0x06	/*!< PLL divided by 8 */
#define RA8876_PLL_DIV_16			0x16	/*!< PLL divided by 16 */
#define RA8876_PLL_DIV_32			0x26	/*!< PLL divided by 32 */
#define RA8876_PLL_DIV_64			0x36	/*!< PLL divided by 64 */

	/**
	 * @brief Vendor specific configuration structure for panel device
	 */
	typedef struct {
		int			wait_gpio_num;			/*!< GPIO used to indicate busy state of the LCD panel, set to GPIO_NUM_NC if it's not used */
		uint16_t	lcd_width;				/*!< Width of the LCD panel in pixels */
		uint16_t	lcd_height;				/*!< Height of the LCD panel in pixels */
		int			mcu_bit_interface;		/*!< Selection between 8-bit and 16-bit MCU interface */
	} esp_lcd_panel_ra8876_config_t;

	/**
	 * @brief Create LCD panel for model RA8876/LT768x
	 *
	 * @param[in] io LCD panel IO handle
	 * @param[in] panel_dev_config general panel device configuration
	 * @param[out] ret_panel Returned LCD panel handle
	 * @return
	 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
	 *          - ESP_ERR_NO_MEM        if out of memory
	 *          - ESP_OK                on success
	 */
	esp_err_t esp_lcd_new_panel_ra8876(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

	/**
	 * @brief Update LCD panel backlight via RA8876/LT768x PWM
	 * 
	 * @param[in] LCD panel handle
	 * @param[in] backlight level (%)
	 * @return
	 *			- ESP_OK				on success
	 */
	esp_err_t esp_lcd_panel_set_backlight(esp_lcd_panel_t *panel, uint8_t level);
	
#ifdef __cplusplus
}
#endif