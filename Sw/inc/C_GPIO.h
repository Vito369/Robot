//
// Created by Ondra on 22.02.2020.
//

#ifndef DOJDEM_C_GPIO_H
#define DOJDEM_C_GPIO_H

#include "C_I2C.h"


#define GPIO_I2C_ADDRESS 0x20
#define ADDR_MCP23017_A(a)    (0x20 | ((a) & 7))

// bank 0 scheme - the power on reset value
// all registers little endian (A = lo, B= hi)
#define MCP23017_IODIR        0x00      // 16bit
#define MCP23017_IOPOL        0x02      // 16bit
#define MCP23017_GPINTEN      0x04      // 16bit
#define MCP23017_DEFVAL       0x06      // 16bit
#define MCP23017_INTCON       0x08      // 16bit
#define MCP23017_IOCON1       0x0A      // 8bit same as IOCON2
#define MCP23017_IOCON2       0x0B      // 8bit same as IOCON1
#define MCP23017_GPPU         0x0C      // 16bit
#define MCP23017_INTF         0x0E      // 16bit
#define MCP23017_INTCAP       0x10      // 16bit
#define MCP23017_GPIO         0x12      // 16bit
#define MCP23017_OLAT         0x14      // 16bit

#define MCP23017_IOCON_BANK       (1 << 7)
#define MCP23017_IOCON_MIRROR     (1 << 6)
#define MCP23017_IOCON_SEQOP      (1 << 5)
#define MCP23017_IOCON_DISSLW     (1 << 4)
#define MCP23017_IOCON_HAEN       (1 << 3)
#define MCP23017_IOCON_ODR        (1 << 2)
#define MCP23017_IOCON_INTPOL     (1 << 1)

#define PB7              (1 << 15)
#define PB6              (1 << 14)
#define PB5              (1 << 13)
#define PB4              (1 << 12)
#define PB3              (1 << 11)
#define PB2              (1 << 10)
#define PB1              (1 << 9)
#define PB0              (1 << 8)
#define PA7              (1 << 7)
#define PA6              (1 << 6)
#define PA5              (1 << 5)
#define PA4              (1 << 4)
#define PA3              (1 << 3)
#define PA2              (1 << 2)
#define PA1              (1 << 1)
#define PA0              (1 << 0)

#define LEDS_B PB6|PB5|PB4|PB3|PB2|PB1|PB0
#define LEDS_A PA5|PA4|PA3|PA2|PA1|PA0
#define BUZZER PB7
#define BUTTONS PA7|PA6



class C_GPIO:public C_I2C {
    uint16_t OLAT_Register;
public:
    C_GPIO(int busnumber, int descriptor, int address): C_I2C(busnumber, descriptor, address)
    {
    }
    void GPIO_Init(){
        Set_output(LEDS_A|LEDS_B|BUZZER);
        Set_input(BUTTONS);
        i2c_read_leuint16(get_fd(), get_i2c_address(), MCP23017_GPIO, &OLAT_Register);
        LED_OFF(LEDS_B|LEDS_A);
    }

    inline void Set_output(uint16_t pins)
    {
        i2c_rmw_leuint16(get_fd(), get_i2c_address(), MCP23017_IODIR, pins, 0, 0);
    }


    inline void Set_input(uint16_t pins)
    {
        i2c_rmw_leuint16(get_fd(), get_i2c_address(), MCP23017_IODIR, 0, pins, 0);
        i2c_rmw_leuint16(get_fd(), get_i2c_address(), MCP23017_GPPU, pins, 0, 0);
    }


    inline void Set_Low( uint16_t pins)
    {
        OLAT_Register&=(~pins);
        i2c_write_leuint16(get_fd(), get_i2c_address(),  MCP23017_OLAT, OLAT_Register);
        //i2c_rmw_leuint16(get_fd(), get_i2c_address(),  MCP23017_OLAT, pins, 0, 0);
    }

    inline void Set_High(uint16_t pins)
    {
        OLAT_Register|=pins;
        i2c_write_leuint16(get_fd(), get_i2c_address(),  MCP23017_OLAT, OLAT_Register);
        //i2c_rmw_leuint16(get_fd(), get_i2c_address(), MCP23017_OLAT, 0, pins, 0);
    }

    inline int Read(void){
        uint16_t read_value;
        i2c_read_leuint16(get_fd(), get_i2c_address(), MCP23017_GPIO, &read_value);
        return read_value;
    }

    inline bool Button_Read(uint16_t button){
        return ((~Read())&(button));
    }

    inline void LED_ON(uint16_t pins)
    {
        Set_Low(pins);
    }

    inline void LED_OFF(uint16_t pins)
    {
        Set_High(pins);
    }

    void Beep(uint32_t time_on, uint32_t time_off){
        Buzzer_ON();
        usleep(1000*time_on);
        Buzzer_OFF();
        usleep(1000*time_off);
    }

    inline void Buzzer_ON()
    {
        Set_High(BUZZER);
    }
    inline void Buzzer_OFF()
    {
        Set_Low(BUZZER);
    }
};


#endif //DOJDEM_C_GPIO_H
