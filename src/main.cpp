#include <Arduino.h>
#include <EEPROM.h>
#include "uwb_driver.h"
#include "serial_interface.h"
#include <SPI.h>


#define EEPROM_SIZE 256

void setup(){
    EEPROM.begin(EEPROM_SIZE);
    Serial.begin(2000000);
    Serial.println();
    uwb_driver.begin();
    // if (uwb_driver.mode == uwb_driver.UWB_TAG){
    //     serial_interface.begin();
    // }

}



void loop(){
    ;
}


