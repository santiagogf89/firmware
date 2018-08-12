#ifndef __LED_DRIVER_REGISTER_H__
#define __LED_DRIVER_REGISTER_H__

// Includes:

    #include "fsl_gpio.h"
    #include "fsl_port.h"
    #include "fsl_i2c.h"
    #include "i2c.h"
    #include "i2c_addresses.h"

// Macros:

    #define LED_DRIVER_SDB_PORT  PORTA
    #define LED_DRIVER_SDB_GPIO  GPIOA
    #define LED_DRIVER_SDB_CLOCK kCLOCK_PortA
    #define LED_DRIVER_SDB_PIN   2

    #define LED_DRIVER_REGISTER_SHUTDOWN            0x0A
    #define LED_DRIVER_REGISTER_FRAME               0xFD
    #define LED_DRIVER_REGISTER_CONFIGURATION       0x00
    #define LED_DRIVER_REGISTER_AUTO_PLAY_CONTROL_1 0x02
    #define LED_DRIVER_REGISTER_AUTO_PLAY_CONTROL_2 0x03
    #define LED_DRIVER_REGISTER_BREATH_CONTROL_1    0x08
    #define LED_DRIVER_REGISTER_BREATH_CONTROL_2    0x09

    #define LED_DRIVER_FRAME_1 0
    #define LED_DRIVER_FRAME_2 1
    #define LED_DRIVER_FRAME_3 2
    #define LED_DRIVER_FRAME_4 3
    #define LED_DRIVER_FRAME_5 4
    #define LED_DRIVER_FRAME_6 5
    #define LED_DRIVER_FRAME_7 6
    #define LED_DRIVER_FRAME_8 7
    #define LED_DRIVER_FRAME_FUNCTION 0x0B

    #define LED_DRIVER_LED_COUNT (2*8*9)
    #define LED_DRIVER_BUFFER_LENGTH (LED_DRIVER_LED_COUNT + 1)

    #define FRAME_REGISTER_LED_CONTROL_FIRST   0x00
    #define FRAME_REGISTER_LED_CONTROL_LAST    0x11
    #define FRAME_REGISTER_BLINK_CONTROL_FIRST 0x12
    #define FRAME_REGISTER_BLINK_CONTROL_LAST  0x23
    #define FRAME_REGISTER_PWM_FIRST           0x24
    #define FRAME_REGISTER_PWM_LAST            0xB3

    #define SHUTDOWN_MODE_SHUTDOWN 0
    #define SHUTDOWN_MODE_NORMAL   1

    #define DISPLAY_MODE_AUTO_FRAME_PLAY 0b01
    #define DISPLAY_MODE_SHIFT 3
    #define FRAME_START_1 0b000
    #define FRAME_START_8 0b111

    #define PLAY_LOOP_NUMBER_1 0b001
    #define PLAY_LOOP_NUMBER_SHIFT 4
    #define PLAY_FRAME_NUMBER_1 0b001

    #define FRAME_DELAY_TIME 1

    #define FADE_OUT_TIME 5
    #define FADE_OUT_TIME_SHIFT 4
    #define FADE_IN_TIME 5

    #define BREATH_ENABLE 1
    #define BREATH_ENABLE_SHIFT 4
    #define EXTINGUISH_TIME 0

// Functions:

    void InitLedDriver(void);

#endif
