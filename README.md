# esp_lcd driver for RA8876/LT768x based displays

This component provides an implementation of the RA8876/LT768x LCD controller using the esp_lcd component APIs.
Both of these chips are functionally identical and can be used interchangeably.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| RA8876/LT768x  | Intel 8080              | esp_lcd_ra8876 | [Specification](https://www.buydisplay.com/download/ic/RA8876.pdf) |

## Note on supported communication interfaces

When using the Intel 8080 (Parallel) interface the 16-bit color depth mode should be used.
This uses RGB565 format data.

## Using this component in your project

This package can be added to your project as follows:

```
dependencies:
  esp_lcd_ra8876:
    git: https://github.com/danmeuk/esp_lcd_ra8876.git
```

For more information on the usage of the `idf_component.yml` file please refer to [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Supported platforms

Due to the i80 nature of the interface, this requires ESP32-S3 or higher.
Older ESP32 chips do not have hardware support for i80 interfaces.
