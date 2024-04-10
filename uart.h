#define IO_UART_SUCCESS                     0
#define IO_UART_ERROR                      -1
#define IO_UART_ERROR_OPEN                 -2
#define IO_UART_ERROR_INIT                 -3
#define IO_UART_ERROR_WRITE                -4
#define IO_UART_ERROR_READ                 -5

int32_t UART_Init(void);
int32_t UART_Read(void *rx_buffer, uint8_t len);
int32_t UART_Write(void *tx_buffer, uint8_t len);
