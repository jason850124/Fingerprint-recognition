#ifndef AS608_H_
#define AS608_H_

#include <stdint.h>
#include "uart.h"

#define AS608_UART_BAUD 57600
#define BUFF_SIZE 1024
#define CMD_LEN   12

#define AS608_SCAN_REQ          0X01
#define AS608_SAVE_REQ          0x0A
#define AS608_DETECTED_FINGER   0X00
// #define AS608_TRANSMIT_READY  0XFF


// uint8_t TxData[]
extern uint8_t TxData[16];
extern uint8_t RxData[144*267+12];
extern uint8_t Finger[112*112];

extern mxc_uart_req_t read_req;
extern mxc_uart_req_t write_req;

extern volatile int READ_FLAG;

extern int as608_uart_init(void);
extern void uart_write(uint8_t *buf, uint32_t len);
extern void uart_read(uint8_t *buf, uint32_t len);
extern void wait_receiving(void);
extern void write_to_as608(uint8_t cmd);
extern void read_from_as608(uint32_t len);
extern void finger_detection(void);


#endif