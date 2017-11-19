Calls `HAL_GPIO_TogglePin` in a tight loop. I don't think the relevant `APB` is configured at the max clock speed.

Nothing obvious from far away. Yellow is `GPIO_SPEED_FREQ_LOW` and blue is `GPIO_SPEED_FREQ_VERY_HIGH`.

![image](https://user-images.githubusercontent.com/118714/31745231-be587812-b416-11e7-9500-c52c08ddece3.png)

30ns difference?

![image](https://user-images.githubusercontent.com/118714/31745199-90d0e08c-b416-11e7-8c05-57ac249e44a6.png)
