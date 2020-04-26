#include <iostream>
#include <chrono>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/mat.hpp>

#include <bpc_prp_opencv_lib/ImageProcessor.h>
#include <bpc_prp_opencv_lib/utils.h>
#include <zconf.h>


#include "../inc/C_GPIO.h"
#include "../inc/C_ADC.h"
#include "../inc/C_MOTORS.h"
#include <cmath>

#define I2C_BUS_NUMBER  1
#define CAMERA 1
#define STARTUP_SETTING 0

uint16_t SAMPLE_TIME=50; //time in ms



enum _State{
    NORMAL,
    CROSSING,
    OUT_OF_TAPE,
    WORKER,
    TESTING
}State=TESTING;


const char *graf(char* buffer, int nchars, float value);

float PSDController1 (float desiredValue, float measuredValue);
float PSDController2 (float desiredValue, float measuredValue);

int main(int argc, char* argv[])
{
    uint32_t millis=0;
    float ang=0, spd=0;
    float prev_ang[6]={0.};
    float distance=0;

    //GPIO
    C_GPIO GPIO(I2C_BUS_NUMBER, 0, GPIO_I2C_ADDRESS);
    GPIO.i2c_init();
    GPIO.GPIO_Init();
    //ADC
    C_ADC ADC(I2C_BUS_NUMBER,GPIO.get_fd(),ADDR_AD799X_0_L);
    ADC.Set_Config(AD799X_CONFIG_FLTR|AD799X_CONFIG_CH_ALL);
    (void)ADC.Read_Register(AD799X_CONFIG);
    //MOTORS
    C_MOTORS MOTORS(I2C_BUS_NUMBER,GPIO.get_fd(),ADDR_KM2_DEFAULT);
#if CAMERA
    //CAMERA
    /// example of bpc_prp_opencv_lib usage ///
    bpc_prp_opencv_lib::ImageProcessor imgProcessor(true, true, "/home/pi/images/");
    cv::VideoCapture cap;
    if(!cap.open(0)) {
        std::cerr << "Error, unable to open camera" << std::endl;
        return -1;
    }
    cv::Mat frame;
    cap >> frame;
    if( frame.empty() ) {
        std::cerr << "Error, frame is empty" << std::endl;
        return -2;
    }
#endif
    //START
    GPIO.Beep(10,1000);
#if STARTUP_SETTING
    while(1){
        ADC.Read_Sensors();
        (void)ADC.Calculate_Distance(1);
        ADC.Print_Info();

        if((GPIO.Button_Read(PA7)==true)&&(GPIO.Button_Read(PA6)==true)){
            break;
        }
        usleep(SAMPLE_TIME*1000);
    }
    GPIO.Beep(10,1000);
    GPIO.Beep(10,1000);
    GPIO.Beep(10,1000);
#endif
    GPIO.LED_ON(PB6);
    while(1) {
        //start of clock
        auto start = std::chrono::high_resolution_clock::now();
        std::cout << "=======================================" << std::endl;
        switch(State){
            case NORMAL:
            {
                ADC.Read_Sensors();
                distance=ADC.Calculate_Distance(0);
                std::cout << "dis: " << distance << std::endl;

                if(ADC.Is_Crossing()){
                    State=CROSSING;
                    break;
                }

                if(ADC.Is_Outside_the_Tape(distance)){
                    State=OUT_OF_TAPE;
                    break;
                }

                spd=PSDController1(C_WHEEL/1.5,MOTORS.get_speed_linear());
                ang=-PSDController2(0,distance);


                MOTORS.Set_Speed(spd+MOTORS.get_speed_linear(),ang);
                //MOTORS.Position_Info();

                prev_ang[5]=prev_ang[4];
                prev_ang[4]=prev_ang[3];
                prev_ang[3]=prev_ang[2];
                prev_ang[2]=prev_ang[1];
                prev_ang[1]=prev_ang[0];
                prev_ang[0]=ang;
#if CAMERA
                cap >> frame;
                auto detections = imgProcessor.analyzeImage(frame);

                if(imgProcessor.GetTapeWidth()>125) {
                    std::cerr << "Tape width: " << imgProcessor.GetTapeWidth() << std::endl;
                    State=CROSSING;
                }

#endif
                break;
            }
            case CROSSING:
            {
                for(uint8_t i=0;i<3;i++){
                    MOTORS.Set_Speed(spd+MOTORS.get_speed_linear(),prev_ang[5]);
                    Delay_ms(SAMPLE_TIME);
                }
                State=NORMAL;

                break;
            }
            case OUT_OF_TAPE:
            {
                ADC.Read_Sensors();
                distance=ADC.Calculate_Distance(0);
                MOTORS.Set_Speed(spd+MOTORS.get_speed_linear(),(prev_ang[4]+prev_ang[5])/2.);
                //MOTORS.Position_Info();
                if(!ADC.Is_Outside_the_Tape(distance)){
                    State=NORMAL;
                }
                break;
            }
            case WORKER:
                {
                break;
            }
            case TESTING:
                {

                cap >> frame;
                auto detections = imgProcessor.analyzeImage(frame);
/*
                if(imgProcessor.GetTapeWidth()>125) {
                    std::cerr << "Tape width: " << imgProcessor.GetTapeWidth() << std::endl;
                    State=CROSSING;
                }
*/
                while(1);
                break;
            }
            default:
                break;
        }



        if((GPIO.Button_Read(PA7)==true)&&(GPIO.Button_Read(PA6)==true)){
            printf("Ended in %.2f seconds.\n",millis/1000.);
            GPIO.LED_OFF(PB1|PB6);
            break;
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        //std::cout << "Duration [ms]: " << duration.count()/1000 << std::endl;
        millis+=(SAMPLE_TIME);

        if((duration.count()/1000)>50.) continue;
        else if((duration.count()/1000)<50.){
            Delay_ms(SAMPLE_TIME-(duration.count()/1000));
        }

        auto stop2 = std::chrono::high_resolution_clock::now();
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start);
        //std::cout << "Duration2 [ms]: " << duration2.count()/1000 << std::endl;
        std::cout << "==================================" << std::endl;
    }
    GPIO.Buzzer_OFF();
    MOTORS.Set_Speed(0,0);
    return 0;
}


void Delay_ms(uint32_t time){
    usleep(1000*time);
}


float PSDController1 (float desiredValue, float measuredValue) {
    float error=0, output=0;

    error = desiredValue - measuredValue;
    output = 0.05*error;

    return output;
}

float PSDController2 (float desiredValue, float measuredValue) {
    float error, integral, derivative, output;
    static float previous_error2;

    error = desiredValue - measuredValue;
    integral = integral + error*(SAMPLE_TIME/1000.);
    derivative = (error - previous_error2)/(SAMPLE_TIME/1000.);
    output = 0.5*error + 0.9*integral + 0.*derivative;
    //output = 0.2*error + 0.5*integral + 0.*derivative;
    previous_error2 = error;
    return output;
}

// funkce na vytvoreni jedne hodnoty grafu do konzole
// buffer ... predalokovany pracovni buffer do ktereho se bude zapisovat text
// nchars ... pocet znaku v bufferu
// val ... hodnota (0...1) k vytisteni
// navratovou hodnotu lze rovnou pouzit jako parametr do printf ("%s", s)
const char *graf(char* buffer, int nchars, float value)
{
    int end = (int)(value * nchars) - 3; // start pipe, end pipe a koncova nula

    buffer[0] = '|';
    int i=1;

    while (i < end)
        buffer[i++] = '=';

    buffer[i++] = '|';

    while (i < (nchars-1))
        buffer[i++] = ' ';

    buffer[nchars-2] = '|';
    buffer[nchars-1] = 0;
    return buffer;
}

