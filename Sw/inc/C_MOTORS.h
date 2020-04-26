//
// Created by Ondra on 20.03.2020.
//

#ifndef DOJDEM_C_MOTORS_H
#define DOJDEM_C_MOTORS_H

#include "C_I2C.h"
#include "km2.h"
#include "cmath"


//
#define C_TW    0.135 //width of car in m
#define D_WHEEL 0.0660 //diameter of wheel in m

#define C_WHEEL (D_WHEEL*M_PI) //circumference of wheel in m
#define C_KM2 262. //write_to_reg=(tim*rot)/((f/M)*(step/(microstep*360)))=[rot/s]*262
#define C_LR C_KM2/C_WHEEL

extern uint16_t SAMPLE_TIME;
extern void Delay_ms(uint32_t time);


class C_MOTORS : public C_I2C {
    float linear_speed, angular_speed;
    int16_t right_wheel_speed, left_wheel_speed;
    float x_position, y_position, distance_traveled,distance_from_start;
    float angle;

public:

    C_MOTORS(int busnumber, int descriptor, int address): C_I2C(busnumber, descriptor, address)
    {
      linear_speed=0; angular_speed=0; right_wheel_speed=0; left_wheel_speed=0;
      x_position=0; y_position=0; distance_traveled=0, distance_from_start=0;
      angle=0;
    }

/**
 * Set linear and angular speed of robot
 *
 * @param linear linear speed of robot in m/s
 * @param angular angular speed of robot in rad/s
 */
    void Set_Speed(float linear, float angular){
        //calculate actual position
        Position_Calculate(linear_speed,angular_speed);
        int16_t pole[2];

        if(isnanf(angular)) {
            std::cerr << "ang isnan" << std::endl;
            angular=0;
        }
        linear_speed=linear;
        angular_speed=angular;

        //left_wheel_speed=(int16_t)(linear_speed+0.5*C_TW*angular_speed)*C_LR;
        //right_wheel_speed=-(int16_t)(linear_speed-0.5*C_TW*angular_speed)*C_LR;

        left_wheel_speed=(linear_speed+0.5*0.135*angular_speed)*(262./(0.066*M_PI));
        right_wheel_speed=-(linear_speed-0.5*0.135*angular_speed)*(262./(0.066*M_PI));

        pole[0]=left_wheel_speed;
        pole[1]=right_wheel_speed;
        if(left_wheel_speed>700) pole[0]=700;
        if(right_wheel_speed>700) pole[1]=700;
        if(left_wheel_speed<-700) pole[0]=-700;
        if(right_wheel_speed<-700) pole[1]=-700;

        std::cout << "left: " << left_wheel_speed  << "\tright: " << right_wheel_speed << std::endl;
        std::cout << "linear: " << linear_speed  << "\tangular: " << angular_speed << std::endl;
        i2c_write_leint16_array(get_fd(), get_i2c_address(), KM2_SPEED, &pole[0], 2);


    }

/**
 * Calculate all
 *
 * @param linear linear speed of robot in m/s
 * @param angular angular speed of robot in rad/s
 */
    void Position_Calculate(float linear, float angular){
        distance_traveled=distance_traveled+abs(linear*SAMPLE_TIME/1000.);
        angle=angle+angular*SAMPLE_TIME/1000.;
        x_position=x_position+sin(angle)*linear*SAMPLE_TIME/1000.;
        y_position=y_position+cos(angle)*linear*SAMPLE_TIME/1000.;
        distance_from_start=hypotf(x_position,y_position);
    }


    float get_speed_linear(void){
        return linear_speed;
    }

    float get_speed_angular(void){
        return angular_speed;
    }

    float get_distance_traveled(void){
        return distance_traveled;
    }

    float get_distance_from_start(void){
        return distance_from_start;
    }

    float get_angle(void){
        return angle;
    }

    void Position_Info(void){
        printf("x [cm] = %.2f \ty [cm] = %.2f \tangle [°] = %.2f\n",x_position*100,y_position*100,angle*180/M_PI);
    }
};


#endif //DOJDEM_C_MOTORS_H
