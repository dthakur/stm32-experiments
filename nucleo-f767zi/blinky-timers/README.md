Uses `TIM1`.

Tricks

- Configuring the correct `Prescaler` and `Period` values; I might be off-by-1
- Remember to start the timer
- `HAL_TIM_PeriodElapsedCallback` is called just by naming convention after `CubeMX` `NVIC` configuration
