[Sensor datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- Needs a `10uS` pulse
- Reading at `30Hz`

Tricks

- [Useful stm32 timer formulas](http://www.st.com/content/ccc/resource/training/technical/product_training/group0/2d/93/74/3f/33/83/47/95/STM32F7_WDG_TIMERS_GPTIM/files/STM32F7_WDG_TIMERS_GPTIM.pdf/_jcr_content/translations/en.STM32F7_WDG_TIMERS_GPTIM.pdf)
- Remember to start the timer
- Using bypass clock at 8MHz coming from ST-Link board
- Using `TIM4`, and `CH3` and `CH4` (for testing)
- `APBx` clocks are at 96MHz
- `Prescaler` at `50-1`
- `Period` at `64000-1`
- This gives a timer frequency of `30Hz`
- Pulse is set to `19200` giving a 10ms pulse on `PD14`
