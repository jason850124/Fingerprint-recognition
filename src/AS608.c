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
uint8_t Finger[112*112];


mxc_uart_req_t read_req;
mxc_uart_req_t write_req;

volatile int READ_FLAG;

volatile uint8_t face_detected;


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


void finger_detection(void)
{
    read_from_as608(CMD_LEN);
    write_to_as608(AS608_SCAN_REQ);
    wait_receiving();
    if(RxData[9]==AS608_DETECTED_FINGER){
        face_detected = 1;
    }
}

void finger_recv_data(void){
    read_from_as608(sizeof(RxData));
    write_to_as608(AS608_SAVE_REQ);
    wait_receiving();
    finger_resize(RxData,Finger);
}


static inline uint8_t rx_get_gray8(const uint8_t *rxData, int sx, int sy)
{

    int rowPair   = sy >> 1;         // sy / 2
    int nibbleSel = sy & 1;          // sy % 2

    const uint8_t b = rxData[RX_HDR_BYTES + rowPair * RX_ROW_STRIDE + sx];

    uint8_t val4 = (nibbleSel == 0) ? (b >> 4) : (b & 0x0F); 
    return (uint8_t)(val4 * 17); // 4-bit (0..15) → 8-bit (0..255) 17
}


#if USE_BILINEAR
static inline uint8_t sample_bilinear(const uint8_t *rxData, float sxf, float syf)
{

    int sx0 = (int)floorf(sxf);
    int sy0 = (int)floorf(syf);
    int sx1 = (sx0 + 1 < SRC_W) ? sx0 + 1 : SRC_W - 1;
    int sy1 = (sy0 + 1 < SRC_H) ? sy0 + 1 : SRC_H - 1;

    float wx = sxf - sx0;
    float wy = syf - sy0;

    uint8_t p00 = rx_get_gray8(rxData, sx0, sy0);
    uint8_t p10 = rx_get_gray8(rxData, sx1, sy0);
    uint8_t p01 = rx_get_gray8(rxData, sx0, sy1);
    uint8_t p11 = rx_get_gray8(rxData, sx1, sy1);

    float top    = p00 + wx * (p10 - p00);
    float bottom = p01 + wx * (p11 - p01);
    float v      = top + wy * (bottom - top);


    int vi = (int)(v + 0.5f);
    if (vi < 0) vi = 0; else if (vi > 255) vi = 255;
    return (uint8_t)vi;
}
#else
static inline uint8_t sample_nearest(const uint8_t *rxData, float sxf, float syf)
{
    int sx = (int)floorf(sxf + 0.5f);
    int sy = (int)floorf(syf + 0.5f);
    if (sx < 0) sx = 0; else if (sx >= SRC_W) sx = SRC_W - 1;
    if (sy < 0) sy = 0; else if (sy >= SRC_H) sy = SRC_H - 1;
    return rx_get_gray8(rxData, sx, sy);
}
#endif


void finger_resize(const uint8_t *rxData, uint8_t *dst112x112)
{

    const float scaleX = (float)SRC_W / (float)DST_W;
    const float scaleY = (float)SRC_H / (float)DST_H;

    for (int y = 0; y < DST_H; ++y) {
        // center-aligned mapping
        float syf = ( (y + 0.5f) * scaleY ) - 0.5f;

        for (int x = 0; x < DST_W; ++x) {
            float sxf = ( (x + 0.5f) * scaleX ) - 0.5f;

            #if USE_BILINEAR
                dst112x112[y * DST_W + x] = sample_bilinear(rxData, sxf, syf);
            #else
                dst112x112[y * DST_W + x] = sample_nearest (rxData, sxf, syf);
            #endif
        }
    }
}


