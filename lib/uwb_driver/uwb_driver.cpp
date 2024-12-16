#include <EEPROM.h>
#include "uwb_driver.h"
#include "log.h"
#include "SPI.h"


#define EEPROM_UWB_CAL_START_ADDREDD 0



void new_range(DW1000Device *device)
{
    LOG_DEBUG("000");
    uint16_t from_device = device->getShortAddress();
    float distance = device->getRange();
    float rxpower  = device->getRXPower();
    float cal_distance = uwb_driver.cal_parms.ks[from_device] * distance + uwb_driver.cal_parms.bs[from_device];
    uwb_driver.update_distance(from_device, distance);
    uwb_driver.update_caldistance(from_device, cal_distance);
    uwb_driver.update_rxpower(from_device, rxpower);
    LOG_DEBUG("from: %d, distance: %lf, caldistance: %lf, power: %lf\n", from_device, distance, cal_distance, rxpower);
}

void new_device(DW1000Device *device)
{
    if (device==NULL)
        return;
    uint8_t device_index = (device->getShortAddress())%16;
    uwb_driver.set_state(device_index, 1);
    LOG_DEBUG("DW1000: ranging init; 1 device added, index: ");
    LOGLN(device_index);
}

void inactive_device(DW1000Device *device)
{
    if (device==NULL)
        return;
    uint8_t device_index = (device->getShortAddress())%16;
    uwb_driver.set_state(device_index, 0);
    LOG_DEBUG("DW1000: delete inactive device: ");
    LOGLN_DEBUG(device_index, HEX);
}


void UwbDriver::set_state(uint8_t index, uint8_t state){
    this->is_activate[index] = 1;
}

void UwbDriver::read_config(void){
    // Step1：读取引脚确定功能
    // 设置配置功能引脚
    pinMode(mode_select_pin,      INPUT_PULLUP);
    pinMode(index_select_pins[0], INPUT_PULLUP);
    pinMode(index_select_pins[1], INPUT_PULLUP);
    pinMode(index_select_pins[2], INPUT_PULLUP);
    // 读取配置
    this->mode       = digitalRead(mode_select_pin) ? BoardType::TAG : BoardType::ANCHOR;
    this->short_addr = 4*digitalRead(index_select_pins[0]) + 2*digitalRead(index_select_pins[1]) + 1*digitalRead(index_select_pins[2]);

    // Step2：从EEPROM读取参数
    EEPROM.readBytes(EEPROM_UWB_CAL_START_ADDREDD, &this->cal_parms, sizeof(cal_parms));
    LOG_INFO("Calculation Parms: \n");
    for (int index=0; index<MAX_ANCHOR_NUM; index++){
        LOG("\t Anchor: %d: k = %.2lf, b = %.2lf\n", index, this->cal_parms.ks[index], this->cal_parms.bs[index]);
    }
    LOG("\n");
}

void UwbDriver::begin_dw1000(void){
    LOG_INFO("DW1000 Configuration: \n");
    static constexpr byte RADIO_CFG[] = {DW1000.TRX_RATE_6800KBPS, DW1000.TX_PULSE_FREQ_16MHZ, DW1000.TX_PREAMBLE_LEN_64};
    const char *macAddress            = "5B:D5:A9:9A:E2:9C";

    // 初始化通信
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.init(this->mode, this->short_addr, macAddress, false, RADIO_CFG, PIN_RST, PIN_SS, PIN_IRQ);

    // 绑定回调函数
    DW1000Ranging.attachNewRange(new_range);
    DW1000Ranging.attachNewDevice(new_device);
    DW1000Ranging.attachInactiveDevice(inactive_device);
    // DW1000Ranging.getCurrentShortAddress
    LOG("\n");
}


void UwbDriver::begin(void){
    this->read_config();
    this->begin_dw1000();

    // 启动循环
    xTaskCreate(UwbDriver::uwb_loop_task, "uwb_loop_task", 24*1024, this, 1, NULL);
}


void UwbDriver::update_distance(uint8_t index, float distance){
    this->distance[index] = distance;
}

void UwbDriver::update_caldistance(uint8_t index, float distance){
    this->caldistance[index] = distance;
}

void UwbDriver::update_rxpower(uint8_t index, float power){
    this->rxpower[index] = power;
}

void UwbDriver::update_kb(uint8_t index, float k, float b){
    this->cal_parms.ks[index] = k;
    this->cal_parms.bs[index] = b;
    EEPROM.writeBytes(EEPROM_UWB_CAL_START_ADDREDD, &this->cal_parms, sizeof(this->cal_parms));
    EEPROM.commit();
}

void UwbDriver::uwb_loop_task(void *pvParameters) {
    UwbDriver* this_pointer = static_cast<UwbDriver*>(pvParameters);
    while(1) {
        DW1000Ranging.loop();
        taskYIELD();
    }
}


UwbDriver uwb_driver;



