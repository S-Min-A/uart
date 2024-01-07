#include <stdio.h> // printf
#include <stdint.h> // uint8_t
#include <fcntl.h> // open
#include <unistd.h> // read, write
#include <termios.h>

#include "uart.h"

#define UART_DEVICE "/dev/ttyS0" // replace
#define UART_BAUDRATE B115200 // replace

int main(void)
{
    int fd;
    uint8_t tx_buffer[1] = {0x00};
    uint8_t rx_buffer[1] = {0x00};

    fd = UART_Init();

    while (1)
    {
        UART_Write(fd, tx_buffer, 1);
        printf("tx : %d\n", tx_buffer[0]);
        UART_Read(fd, rx_buffer, 1);
        printf("rx : %d\n", rx_buffer[0]);
        sleep(1);
    }
}

int UART_Init(void)
{
    int fd;

    if ((fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    {
        printf("Open error!\n");
    }

    struct termios options;
    tcgetattr(fd, &options);
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
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

void UART_Read(int fd, void *rx_buffer, uint8_t len)
{
    if (read(fd, rx_buffer, len) < 0)
    {
        printf("Read error!\n");
    }
}

void UART_Write(int fd, void *tx_buffer, uint8_t len)
{
    if (write(fd, tx_buffer, len) < 0)
    {
        printf("Write error!\n");
    }
}
