//
// Created by Ondra on 23.02.2020.
//

#ifndef DOJDEM_C_ADC_H
#define DOJDEM_C_ADC_H

#include "C_I2C.h"
#include "ad799x.h"






class C_ADC: public C_I2C {
    uint16_t sensors_value[8];
    uint8_t previous_closest_sensor;
    uint16_t sensor_min;
    uint16_t sensor_max;
    uint16_t sensors_both_have_same_values; //is changing during run
    uint16_t sensor_value_when_other_sensor_is_in_min; //is set anytime sensor reaches minimum

public:

    C_ADC(int busnumber, int descriptor, int address): C_I2C(busnumber, descriptor, address)
    {
        for(uint8_t i=0;i<8;i++) {
            sensors_value[i]=0;
        }


         //brown cardboard
         /*
        previous_closest_sensor=123;
        sensor_value_when_other_sensor_is_in_min=2700;
        sensors_both_have_same_values=1500;
        sensor_min=950;
        sensor_max=3300;
*/


        //paper

        previous_closest_sensor=123;
        sensor_value_when_other_sensor_is_in_min=3700;
        sensors_both_have_same_values=2000;
        sensor_min=1100;
        sensor_max=3900;

        //official track
        /*
        previous_closest_sensor=123;
        sensor_value_when_other_sensor_is_in_min=3700;
        sensors_both_have_same_values=2200;
        sensor_min=1200;
        sensor_max=3900;
         */
    }

/**
 * Set Configuration register
 *
 * @param Config can be combination of AD799X_CONFIG
 */
    void Set_Config(uint16_t Config){
        i2c_write_beuint16(get_fd(),get_i2c_address(),AD799X_CONFIG,Config);
    }


    uint16_t Read_Register(uint8_t Register){
        uint16_t receive;
        i2c_read_beuint16(get_fd(),get_i2c_address(),Register,&receive);
        return receive;
    }


    void Read_Sensors(void){
        uint16_t Read;
        uint8_t Channel;
        for(uint8_t i=1;i<3;i++){
            //read conversion result register
            Read=Read_Register(AD799X_RESULT_CH(i));

            //check which channel it was
            Channel=((Read&AD799X_RESULT_CHAN)>>12);

            //save value
            sensors_value[Channel]=(Read&0x0FFF);
        }
    }


    float Calculate_Distance(uint8_t auto_setting){
        float distance;
        uint8_t closest=Find_Min(auto_setting);
        float constant_outside=(sensor_max-sensor_min)/15;
        float constant_inside=(sensors_both_have_same_values-sensor_min)/5;

        //only happens on start
        if(previous_closest_sensor==123) previous_closest_sensor=closest;

        if(closest==1){
            if(previous_closest_sensor==2)
            {
                distance=0;
                previous_closest_sensor=1;
            }
            //tape is between sensors
            else if(sensors_value[2]<sensor_value_when_other_sensor_is_in_min){
                distance=-5+((sensors_value[1]-sensor_min)/constant_inside);
            }
            //tape is far from center of vehicle
            else {
                distance=-5-((sensors_value[1]-sensor_min)/constant_outside);
            }
        }

        else if(closest==2){

            if(previous_closest_sensor==1)
            {
                distance=0;
                previous_closest_sensor=2;
            }
            else if(sensors_value[1]<sensor_value_when_other_sensor_is_in_min){
                distance=5-((sensors_value[2]-sensor_min)/constant_inside);
            }
            else {
                distance=5+((sensors_value[2]-sensor_min)/constant_outside);
            }


        }
        if(isnanf(distance)){
            distance=0;
        }
        if(isinf(distance)){
            distance=0;
        }
        return distance;

    }


    uint8_t Find_Min(uint8_t auto_setting){
        if(sensors_value[1]<sensors_value[2])
        {
            if((sensors_value[1]<sensor_min)&&(auto_setting)){
                sensor_min=sensors_value[1];
                sensor_value_when_other_sensor_is_in_min=sensors_value[2];

            }
            if(sensors_value[2]>sensor_max){
                sensor_max=sensors_value[2];
            }
            return 1;
        }
        else{
            if((sensors_value[2]<sensor_min)&&(auto_setting)){
                sensor_min=sensors_value[2];
                sensor_value_when_other_sensor_is_in_min=sensors_value[1];
            }
            if(sensors_value[1]>sensor_max){
                sensor_max=sensors_value[1];
            }
            return 2;
        }
    }

    float Get_Sensor_Value(uint8_t Channel){
        return sensors_value[Channel];
    }

    bool Is_Outside_the_Tape(float distance){
        return ((distance>14.)||(distance<-14.));
    }


    bool Is_Crossing(){
        return ((sensors_value[1]<sensor_min)&&
            (sensors_value[2]<(sensor_min)));
    }


    void Print_Info(void){
        std::cout << "min: " << sensor_min << std::endl;
        std::cout << "max: " << sensor_max << std::endl;
        std::cout << "center: " << sensors_both_have_same_values << std::endl;
        std::cout << "val1 when 2 is in min: " << sensor_value_when_other_sensor_is_in_min << std::endl;
    }
};


#endif //DOJDEM_C_ADC_H
