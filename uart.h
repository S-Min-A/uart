int UART_Init(void);
void UART_Read(int fd, void *rx_buffer, uint8_t len);
void UART_Write(int fd, void *tx_buffer, uint8_t len);
