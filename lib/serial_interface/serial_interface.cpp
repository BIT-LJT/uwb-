#include <Arduino.h>
#include <EEPROM.h>
#include "just_float.h"
#include "serial_interface.h"
#include "uwb_driver.h"
#include "log.h"


void SerialInterface::begin(void){
    xTaskCreate(SerialInterface::data_send_task, "serial_data_send_task", 2048, this, 1, NULL);
    xTaskCreate(SerialInterface::cmd_parse_task, "serial_cmd_parse_task", 2048, this, 1, NULL);

}


void SerialInterface::data_send_task(void *pvParameters) {
    SerialInterface* this_pointer = static_cast<SerialInterface*>(pvParameters);
    JustFloatFrame frame;
    static portTickType xLastWakeTime;
    const portTickType  xFrequency = pdMS_TO_TICKS(100);
    xLastWakeTime = xTaskGetTickCount();
    while(1) {
        // uwb_solver_2d.solve(uwb_driver.caldistance);
        memcpy(frame.original_distances,   uwb_driver.distance,    sizeof(frame.original_distances));
        memcpy(frame.calibrated_distances, uwb_driver.caldistance, sizeof(frame.calibrated_distances));
        memcpy(frame.rx_power,             uwb_driver.rxpower,     sizeof(frame.rx_power));
        // memcpy(&frame.position, &uwb_solver_2d.filter_position, sizeof(uwb_solver_2d.filter_position));
        memset(&frame.position, 0, sizeof(frame.position));
        memcpy(frame.is_activate, uwb_driver.is_activate, sizeof(frame.is_activate));
        Serial.write((char*)&frame, sizeof(frame));
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void SerialInterface::cmd_parse_task(void *pvParameters) {
    SerialInterface* this_pointer = static_cast<SerialInterface*>(pvParameters);
    CommandFrame frame;
    uint8_t buffer[64];
    while(1) {
        memset(buffer, 0, sizeof(buffer));
        int count = Serial.readBytesUntil('\n',buffer,64);
        if (count>0) {
            memcpy(&frame, buffer, sizeof(frame));
            // 校准参数
            if (frame.command == COMMAND_SET_KB){
                uwb_driver.update_kb(frame.anchor_num, frame.data[0], frame.data[1]);
            }
            // Anchor位置参数
            if (frame.command == COMMAND_SET_POSITION){
                // uwb_solver_2d.update_anchor_position(frame.anchor_num, frame.data[0], frame.data[1], frame.data[2]);
            }

            LOGLN_INFO("Parameter updated!");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



SerialInterface serial_interface;
