#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "dma.h"
#include "nvic_table.h"
#include "AS608.h"

uint8_t TxData[16];
uint8_t RxData[144*267+12];
uint8_t Finger[144][256];

mxc_uart_req_t read_req;
mxc_uart_req_t write_req;

volatile int READ_FLAG;


void UART1_Handler(void)
{
    MXC_UART_AsyncHandler(MXC_UART1);
}


void readCallback(mxc_uart_req_t *req, int error)
{
    READ_FLAG = error;
}


int as608_uart_init(void)
{   
    int error;
    printf("AS608 uart init processing ...\n");

    NVIC_ClearPendingIRQ(UART1_IRQn);
    NVIC_DisableIRQ(UART1_IRQn);
    MXC_NVIC_SetVector(UART1_IRQn, UART1_Handler);
    NVIC_EnableIRQ(UART1_IRQn);

    if ((error = MXC_UART_Init(MXC_UART1, AS608_UART_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) 
    {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    if ((error = MXC_UART_Init(MXC_UART2, AS608_UART_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) 
    {
        printf("-->Error initializing UART: %d\n", error);
        printf("-->Example Failed\n");
        return error;
    }

    read_req.uart = MXC_UART1;
    read_req.txData = NULL;
    read_req.txLen = 0;
    read_req.callback = readCallback;

    write_req.uart = MXC_UART2;
    write_req.rxData = NULL;
    write_req.rxLen = 0;
    write_req.callback = NULL;

    //as608 default command
    TxData[0] = 0xef;
    TxData[1] = 0x01;
    TxData[2] = 0xff;
    TxData[3] = 0xff;
    TxData[4] = 0xff;
    TxData[5] = 0xff;
    TxData[6] = 0x01;
    TxData[7] = 0x00;
    TxData[8] = 0x03;
    TxData[9] = 0x01;
    TxData[10] = 0x00;
    TxData[11] = 0x05;
    

    printf("AS608 uart init done!\n");

    return 0;
}

void uart_write(uint8_t *buf, uint32_t len)
{
    write_req.txData = buf;
    write_req.txLen = len;
    MXC_UART_Transaction(&write_req);
}

void uart_read(uint8_t *buf, uint32_t len)
{
    read_req.rxData = buf;
    read_req.rxLen = len;
    READ_FLAG = 1;
    MXC_UART_ClearRXFIFO(MXC_UART1); // Clear any previously pending data
    MXC_UART_TransactionAsync(&read_req);
}


void wait_receiving(void)
{
    while(READ_FLAG){}
}

uint8_t cmd_checksum(uint8_t cmd)
{
    uint8_t sum = 4 + cmd; // 4: commd ID(0x01) + commd Length(0x03)
    return sum;
}

/*give command*/
void write_to_as608(uint8_t cmd) 
{
    TxData[9] = cmd;
    TxData[11] = cmd_checksum(cmd);
    uart_write(TxData, CMD_LEN);
}

/*give length*/
void read_from_as608(uint32_t len)
{
    RxData[9] = 0xFF;
    uart_read(RxData, len);
}


