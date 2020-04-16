//
// Created by Ondra on 22.02.2020.
//

#ifndef DOJDEM_C_I2C_H
#define DOJDEM_C_I2C_H

#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <alloca.h>

#include "i2c.h"

class C_I2C {
    int bus_number;
    int bus_descriptor;
    int i2c_address;
public:
    C_I2C() : bus_number(),bus_descriptor(),i2c_address()
    {}

    C_I2C(int busnumber) : bus_number(busnumber),bus_descriptor(),i2c_address()
    {}

    C_I2C(int busnumber, int descriptor) : bus_number(busnumber),bus_descriptor(descriptor),i2c_address()
    {}

    C_I2C(int busnumber, int descriptor, int adress) : bus_number(busnumber),bus_descriptor(descriptor),i2c_address(adress)
    {}

    ~C_I2C()
    {
        close(bus_descriptor);
    }
    void i2c_init(void)
    {
        char name[256];
        sprintf(name, "/dev/i2c-%d", bus_number);
        bus_descriptor = open(name, O_RDWR);
    }

    inline int get_i2c_address(void){
        return  i2c_address;
    }

    inline int get_fd(void){
        return  bus_descriptor;
    }
    inline int get_bus_number(void){
        return  bus_number;
    }

};


#endif //DOJDEM_C_I2C_H
