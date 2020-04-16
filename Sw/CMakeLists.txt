cmake_minimum_required(VERSION 3.7)
project(Sw)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )

add_subdirectory(libs/bpc_prp_opnecv_lib)

include_directories(libs/bpc_prp_opnecv_lib/include)

add_executable(Sw src/main.cpp  inc/i2c.h inc/C_I2C.h inc/C_GPIO.h inc/C_ADC.h inc/C_MOTORS.h)
target_link_libraries( Sw ${OpenCV_LIBS} bpc_prp_opencv_lib)