### stm32 experiments

- All projects created with `STM32CubeMX`
- All projects compiled with `gcc`
- All projects tested on [nucleo-f767zi](http://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
- All binaries loaded using [stlink](https://github.com/texane/stlink)

### catalog

| Project  | Description |
| ------------- | ------------- |
| blinky         | Blinks the two on-board LEDs at 10Hz  |
| blinky-timers  | Blinks the two on-board LEDs at 10Hz using timers and interrupts |
| speedy         | Toggles `PC8` configured with `GPIO_SPEED_FREQ_LOW` and `PC9` configured with `GPIO_SPEED_FREQ_VERY_HIGH`  |

#### blinky

It blinks, yes.

![img_4507 mov](https://user-images.githubusercontent.com/118714/31745294-ff07a19e-b416-11e7-883e-374cf1d737e8.gif)

#### speedy

Nothing obvious from far away. Yellow is `GPIO_SPEED_FREQ_LOW` and blue is `GPIO_SPEED_FREQ_VERY_HIGH`.

![image](https://user-images.githubusercontent.com/118714/31745231-be587812-b416-11e7-9500-c52c08ddece3.png)

30ns difference?

![image](https://user-images.githubusercontent.com/118714/31745199-90d0e08c-b416-11e7-8c05-57ac249e44a6.png)

TODO: change `AHB` frequency?
