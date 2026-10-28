#ifndef __SOLENOID_H
#define __SOLENOID_H

#include "main.h"

typedef struct Solenoid_t
{
    GPIO_TypeDef *gpio_port;  /*这组 SDA/CLK 所在的 GPIO 端口*/
    uint16_t gpio_pin_sda;   /*SDA 使用哪个 GPIO Pin*/
    uint16_t gpio_pin_clk;   /*CLK 使用哪个 GPIO Pin*/
    uint8_t data_prve;   /*保存上一次已经发送到这组电磁阀板的 4 路状态。*/
} Solenoid_t;

void solenoid_init(uint8_t usart_channel);
void solenoid_on(uint8_t usart_channel, uint8_t cmd);
#endif /* __SOLENOID_H */
