idf_component_register(
	SRCS "esp_lcd_ra8876.c"
	INCLUDE_DIRS "."
	REQUIRES "esp_lcd"
	PRIV_REQUIRES "driver esp_timer"
)