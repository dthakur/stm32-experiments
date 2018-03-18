Tricks

- [Useful stm32 timer formulas](http://www.st.com/content/ccc/resource/training/technical/product_training/group0/2d/93/74/3f/33/83/47/95/STM32F7_WDG_TIMERS_GPTIM/files/STM32F7_WDG_TIMERS_GPTIM.pdf/_jcr_content/translations/en.STM32F7_WDG_TIMERS_GPTIM.pdf)
- Another (useful tutorial)[https://visualgdb.com/tutorials/arm/stm32/pwm/]
- Using bypass clock at 8MHz coming from ST-Link part of the board
- Using `TIM4-CH2`; `APBx` is set up with 8MHz clock
- `Prescaler` set 8-1
- `ARR` is set to 10000-1, giving a timer frequency of `100Hz`
- Remember to start the timer
