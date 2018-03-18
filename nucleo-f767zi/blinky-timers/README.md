Uses `TIM1`.

Tricks

- Using bypass clock at 8MHz coming from ST-Link board
- Configuring the correct `Prescaler` and `Period` values; They are off-by-1.
- TIM1 is on APB2, which is set to 216MHz
- PSC is 21600 and ARR is 10000, giving an interrupt every ~1s
- [Useful stm32 timer formulas](http://www.st.com/content/ccc/resource/training/technical/product_training/group0/2d/93/74/3f/33/83/47/95/STM32F7_WDG_TIMERS_GPTIM/files/STM32F7_WDG_TIMERS_GPTIM.pdf/_jcr_content/translations/en.STM32F7_WDG_TIMERS_GPTIM.pdf)
- Remember to start the timer
- `HAL_TIM_PeriodElapsedCallback` is called just by naming convention after `CubeMX` `NVIC` configuration
