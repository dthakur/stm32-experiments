### stm32 experiments

- All projects created with `STM32CubeMX`
- All projects compiled with `gcc`
- All projects tested on [nucleo-f767zi](http://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
- [Board user manual](http://www.st.com/content/ccc/resource/technical/document/user_manual/group0/26/49/90/2e/33/0d/4a/da/DM00244518/files/DM00244518.pdf/jcr:content/translations/en.DM00244518.pdf)
- [Board schematic](nucleo-f767zi-schematic.pdf)
- All binaries loaded using [stlink](https://github.com/texane/stlink)
- Flash using `st-flash --format ihex write <program>.hex`

### catalog

| Project  | Description |
| ------------- | ------------- |
| [blinky](nucleo-f767zi/blinky)         | Blinks the two on-board LEDs at 10Hz  |
| [blinky-timers](nucleo-f767zi/blinky-timers)  | Blinks the two on-board LEDs at 10Hz using timers and interrupts |
| [blinky-rtos](nucleo-f767zi/blinky-rtos)  | Blinks the two on-board LEDs at 10Hz using a `FreeRTOS` thread |
| [blinky-usb](nucleo-f767zi/blinky-usb)  | Toggles blinking on and off based on usb virtual com port input |
| [blinky-button](nucleo-f767zi/blinky-button)  | Toggles blinking on and off based on button press (not debounced) |
| [speedy](nucleo-f767zi/speedy)         | Toggles `PC8` and `PC9` at different speeds  |
| [blinky-timers-dma](nucleo-f767zi/blinky-timers-dma)         | Toggle LEDs using the hardware timers and DMA  |
| [synthesizer-adf4351](nucleo-f767zi/synthesizer-adf4351) | Interface with `ADF4351` over SPI |
