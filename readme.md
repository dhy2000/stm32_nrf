# Readme

照着csdn博客（原博客应该是抄的正点原子的教材）抄的STM32与nRF24L01无线模块的例程，实现板间通讯。

本人使用的开发板为STM32F103C8T6，开发环境为CubeIDE。

需要注意的点：

- 需要自己在CubeMX中配置SPI的CE CSN IRC引脚绑定，其中CSN和CE都是GPIO_Output, IRC是GPIO_External_Interrupt。
- 根据自己配置的引脚编号调整`urf24l01.h`中定义如上引脚的宏。
- 必须发射端与接收端同时启动才能工作，如果只有发射端没有接收端，则发送函数会一直返回错误标志"达到最大重发次数"。

