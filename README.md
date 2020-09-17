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
gpio_set(PB01) or gpio_set_state(PB01, true) // set pin HIGH
gpio_reset(PB01) or gpio_set_state(PB01, false) // set pin LOW
```

## UART example

```c
uart_init(1, 9600); // init USART1 with 9600b/s (pins PA9/PA10 will be used)
uart_send_string(1, "Hallo world!\r\n"); // send string to USART1 using DMA
```

add a handler and enable receiving

```c
/* uart_rx will be executed when data will be filled to buffer */
void uart_rx(uint8_t uart_num, char *data, uint16_t size, void *private_data)
{
  /* your code here */
}

/* enable interrupt based receiving */
uart_enable_rx_buffer(1, buf, sizeof(buf), uart_rx, 0);
```

## SPI example

```c
spi_init(1, 12000000, false); // init SPI1 with 12MHz clock low polarity(set true to use high polarity)
```

send data to register:

```c
spi_write_reg(1, CS_PIN, 0xEE, buf, sizeof(buf)); // send buffer to register 0xEE using SPI1
```

read data from register:

```c
spi_read_reg(1, CS_PIN, 0xEE, buf, sizeof(buf)); // read data to buffer from register 0xEE using SPI1
```

## Delays example
simply use delay_us or delay_ms. No initialization is needed.
```c
delay_ms(1000);
delay_us(1000);
```

... also available ADC, CRC, RTC and I2C drivers

Development is in progress. Examples will be provided.
For remarks contact me to pavelnadein@gmail.com
