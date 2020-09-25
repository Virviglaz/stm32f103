# STM32F103 open source highly efficient peripheral driver

This drivers is a simplified and highly enchansed alternative to the official HAL/SPL ST's drivers.
It is specially made to be used with realtime operational systems as well as with the bare metall platforms.

* Exluding waiting calls
* Efficently using DMA
* All in one single initialization
* Sanity checks
* Embedded interrupt vectors

## GPIO example
```c
gpio_output_init(PB01, PUSHPULL_OUTPUT, GPIO_FREQ_50MHz);

/* set pin HIGH */
gpio_set(PB01); gpio_set_state(PB01, true);

/* set pin LOW */
gpio_reset(PB01); gpio_set_state(PB01, false);
```

## UART example
```c
/* init USART1 with 9600b/s (pins PA9/PA10 will be used) */
uart_init(1, 9600);

/* send string to USART1 using DMA */
uart_send_string(1, "Hallo world!\r\n");
```

add a handler and enable receiving
```c
/* uart_rx will be executed when buffer is full or carriage return simbol is received  */
void uart_rx(uint8_t uart_num, char *data, uint16_t size, void *private_data)
{
  /* your code here */
}

/* enable interrupt based receiving */
uart_enable_rx_buffer(1, buf, sizeof(buf), uart_rx, 0);
```

## SPI example
```c
/* init SPI1 with 12MHz clock low polarity(set true to use high polarity) */
spi_init(1, 12000000, false);
```

send data to register:

```c
/* send buffer to register 0xEE using SPI1 */
spi_write_reg(1, CS_PIN, 0xEE, buf, sizeof(buf));
```

read data from register:

```c
/* read data to buffer from register 0xEE using SPI1 */
spi_read_reg(1, CS_PIN, 0xEE, buf, sizeof(buf));
```

## I2C example

```c
/* init I2C1 in normal (false) 100kHz or fast(true) 400kHz mode. */
i2c_init(1, false);

/* send buffer to register 0x05 of device with address 0xEE using I2C1 */
i2c_write_reg(1, 0xEE, 0x05, buf, sizeof(buf));

/* read 10 bytes to buffer from register 0x07 of device with address 0xD0 using I2C1 */
i2c_read_reg(1, 0xD0, 0x07, buf, 10);
```

## Delays example
simply use delay_us or delay_ms. No initialization is needed.
```c
delay_ms(1000);
delay_us(1000);
```
# Added FreeRTOS support:
## No waiting calls. All functions using async execution.
### UART
```c
/* block the task while string is sending with DMA */
uart_send_string_rtos(1, "Hallo world\r\n");
```
### EXTI
```c
/* block the task until pin change from HIGH to LOW */
wait_for_pinchange_rtos(PB11, false, true);
```
### I2C
```c
/* block the task while I2C is reading the data using DMA */
i2c_read_reg_rtos(1, 0x68, 0xEE, buf, sizeof(buf));
```
### SPI
```c
/* block the task while SPI is reading the data using DMA */
spi_read_reg_rtos(1, PA3, 0xEE, buf, sizeof(buf));
```
### DMA
```c
/* block the task while DMA is copying the data from memory to memory */
memcpy_dma8((void *)buf, (void *)s, sizeof(buf));
```
### ADC
```c
/* block the task while ADC is performing conversion */
adc_single_conversion_rtos(1, 2);
```
Development is in progress. Examples will be provided.
For remarks contact me to pavelnadein@gmail.com
