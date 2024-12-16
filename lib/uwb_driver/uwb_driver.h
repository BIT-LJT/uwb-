#ifndef UWB_DRIVER_H
#define UWB_DRIVER_H
#include <Arduino.h>
#include <SPI.h>
#include "uwb_config.h"
#include "DW1000Ranging.h"






class UwbDriver{
private:
    // UWB工作模式
    uint16_t   short_addr;
    BoardType  mode;

    // 函数声明
    void    read_config(void);
    void    begin_dw1000(void);
    static void uwb_loop_task(void* pvParameters);

public:
    UwbDriver() {
        memset(distance,    0,   sizeof(distance));
        memset(rxpower,     0,   sizeof(rxpower));
        memset(&cal_parms,  0,   sizeof(cal_parms));
        memset(caldistance, 0,   sizeof(caldistance));
        memset(is_activate, 0,   sizeof(is_activate));
    }

    float   distance[MAX_ANCHOR_NUM];
    float   rxpower [MAX_ANCHOR_NUM];
    float   caldistance [MAX_ANCHOR_NUM];
    uint8_t is_activate[8];
    void  begin(void);
    void  update_distance(uint8_t index, float distance);
    void  update_rxpower (uint8_t index, float power);
    void  update_kb(uint8_t index, float k, float b);
    void  update_caldistance(uint8_t index, float distance);
    void  set_state(uint8_t index, uint8_t state);

    // UWB标定
    struct {
        float ks[MAX_ANCHOR_NUM];
        float bs[MAX_ANCHOR_NUM];
    }cal_parms;
};

extern UwbDriver uwb_driver;


#endif

