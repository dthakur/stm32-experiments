1. Uses timers and dma in circular mode to blink led
1. Timer buses are set to 1 MHz
1. Prescaler is set to 9999, giving 100 Hz tick
1. Counter period is set to 99, giving tick every 1s
1. So the LEDs are on for 1s and then off for 1s
1. Use DMA in circular mode to copy from buffer to LED port
1. Be sure to start the timer
1. Need to start dma manually as `HAL_TIM_Base_Start_DMA` makes assumptions on dma destination (`ARR`)
