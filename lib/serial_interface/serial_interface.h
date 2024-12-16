#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H


class SerialInterface{
private:
    static void data_send_task(void* pvParameters);
    static void cmd_parse_task(void* pvParameters);

public:
    void begin(void);

};

extern SerialInterface serial_interface;


#endif

