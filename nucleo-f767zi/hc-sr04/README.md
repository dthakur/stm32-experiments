Tricks

- [Useful stm32 timer formulas](http://www.st.com/content/ccc/resource/training/technical/product_training/group0/2d/93/74/3f/33/83/47/95/STM32F7_WDG_TIMERS_GPTIM/files/STM32F7_WDG_TIMERS_GPTIM.pdf/_jcr_content/translations/en.STM32F7_WDG_TIMERS_GPTIM.pdf)
- Remember to start the timer
- Using bypass clock at 8MHz coming from ST-Link board
- Using `TIM4`, and `CH3`
- `APBx` clocks are at 96MHz
- `Prescaler` at `1500-1`
- `Period` at `64000-1`
- This gives a timer frequency of `1Hz`
- Pulse is set to `6400` giving a 10ms pulse on `PD14`
