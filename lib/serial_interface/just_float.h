#ifndef JUST_FLOAT
#define JUST_FLOAT
#include <Arduino.h>
#include "uwb_config.h"


const uint8_t HEADER_BYTES[] = {0xFF, 0xAA};
const uint8_t TAIL_BYTES[]   = {0x00, 0x00, 0x80, 0x7F};

// 发送信息的串口协议
typedef struct{
    uint8_t  header[2]  {0xFF, 0xAA};
    uint8_t  command[2] {0x01, 0x01};
    float    original_distances  [MAX_ANCHOR_NUM];
    float    calibrated_distances[MAX_ANCHOR_NUM];
    float    rx_power            [MAX_ANCHOR_NUM];
    Position position;
    uint8_t  is_activate[8];
    uint8_t  tail[8]{0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00, '\n'};
}JustFloatFrame;


typedef struct{
    uint8_t  header[2]{0xFF, 0xAA};
    uint8_t  command;           // 0x01：发送校准参数；0x02：发送anchor位置
    uint8_t  anchor_num;
    float    data[3];
    uint8_t  tail[8]{0x00, 0x00, 0x80, 0x7f, 0x00, 0x00, 0x00, '\n'};
}CommandFrame;


const uint8_t COMMAND_SET_KB       = 0x01;
const uint8_t COMMAND_SET_POSITION = 0x02;

const uint8_t COMMAND_FRAME_SIZE = sizeof(CommandFrame);

#endif
