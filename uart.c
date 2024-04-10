#include <stdio.h> // printf
#include <stdint.h> // uint8_t
#include <fcntl.h> // open
#include <unistd.h> // read, write
#include <termios.h>

#include "uart.h"

#define UART_DEVICE "/dev/ttyS0" // replace
#define UART_BAUDRATE B115200 // replace

static int32_t uart_fd;

int main(void)
{
    uint8_t tx_buffer[1] = {0x01};
    uint8_t rx_buffer[1] = {0x00};

    UART_Init();

    while (1)
    {
        rx_buffer[0] = 0x00;
        
        UART_Write(tx_buffer, 1);
        printf("tx : %d\n", tx_buffer[0]);

        UART_Read(rx_buffer, 1);
        printf("rx : %d\n", rx_buffer[0]);

        sleep(1);
    }
}

int32_t UART_Init(void)
{
    if ((uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    {
        return IO_UART_ERROR_OPEN;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, UART_BAUDRATE);
    cfsetospeed(&options, UART_BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	options.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 1;
    tcsetattr(uart_fd, TCSANOW, &options);

    return IO_UART_SUCCESS;
}

int32_t UART_Read(void *rx_buffer, uint8_t len)
{
    if (uart_fd < 0)
    {
        return IO_UART_ERROR_INIT;
    }
    if (read(uart_fd, rx_buffer, len) < 0)
    {
        return IO_UART_ERROR_READ;
    }

    return IO_UART_SUCCESS;
}

int32_t UART_Write(void *tx_buffer, uint8_t len)
{
    if (uart_fd < 0)
    {
        return IO_UART_ERROR_INIT;
    }
    if (write(uart_fd, tx_buffer, len) < 0)
    {
        return IO_UART_ERROR_WRITE;
    }
    
    return IO_UART_SUCCESS;
}
