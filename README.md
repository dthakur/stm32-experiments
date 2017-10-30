### stm32 experiments

- All projects created with `STM32CubeMX`
- All projects compiled with `gcc`
- All projects tested on [nucleo-f767zi](http://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
- All binaries loaded using [stlink](https://github.com/texane/stlink)

### catalog

| Project  | Description |
| ------------- | ------------- |
| [blinky](nucleo-f767zi/blinky)         | Blinks the two on-board LEDs at 10Hz  |
| [blinky-timers](nucleo-f767zi/blinky-timers)  | Blinks the two on-board LEDs at 10Hz using timers and interrupts |
| [blinky-rtos](nucleo-f767zi/blinky-rtos)  | Blinks the two on-board LEDs at 10Hz using a `FreeRTOS` thread |
| [blinky-usb](nucleo-f767zi/blinky-usb)  | Toggles blinking on and off based on usb virtual com port input |
| [blinky-button](nucleo-f767zi/blinky-button)  | Toggles blinking on and off based on button press (not debounced) |
| [speedy](nucleo-f767zi/speedy)         | Toggles `PC8` and `PC9` at different speeds  |
