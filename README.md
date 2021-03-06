### stm32 experiments

- All projects created with `STM32CubeMX`
- All projects compiled with `gcc`
- All projects tested on [NUCLEO-F767ZI](http://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
- [NUCLEO-F767ZI user manual](http://www.st.com/content/ccc/resource/technical/document/user_manual/group0/26/49/90/2e/33/0d/4a/da/DM00244518/files/DM00244518.pdf/jcr:content/translations/en.DM00244518.pdf)
- [NUCLEO-F767ZI schematic](nucleo-f767zi-schematic.pdf)
- [STM32F767ZIT6 reference manual](http://www.st.com/content/ccc/resource/technical/document/reference_manual/group0/96/8b/0d/ec/16/22/43/71/DM00224583/files/DM00224583.pdf/jcr:content/translations/en.DM00224583.pdf)
- [STM32F767ZIT6 programming manual](http://www.st.com/content/ccc/resource/technical/document/programming_manual/group0/78/47/33/dd/30/37/4c/66/DM00237416/files/DM00237416.pdf/jcr:content/translations/en.DM00237416.pdf)
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
| [hc-sr04](nucleo-f767zi/hc-sr04) | Making distance measurements using `HC-SR04` ultrasonic sensor |
