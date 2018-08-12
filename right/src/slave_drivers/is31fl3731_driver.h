#ifndef __SLAVE_DRIVER_LED_DRIVER_H__
#define __SLAVE_DRIVER_LED_DRIVER_H__

// Includes:

    #include "fsl_common.h"
    #include "peripherals/led_driver.h"

// Macros:

    #define LED_DRIVER_MAX_COUNT 2
    #define LED_CONTROL_REGISTERS_COMMAND_LENGTH 19
    #define PMW_REGISTER_UPDATE_CHUNK_SIZE 8
    #define PWM_REGISTER_BUFFER_LENGTH (1 + PMW_REGISTER_UPDATE_CHUNK_SIZE)

    #define IS_ISO true
    #define ISO_KEY_LED_DRIVER_ID LedDriverId_Left
    #define ISO_KEY_CONTROL_REGISTER_POS 7
    #define ISO_KEY_CONTROL_REGISTER_BIT 1

// Typedefs:

    typedef enum {
        LedDriverId_Right,
        LedDriverId_Left,
        LedDriverId_Last = LedDriverId_Left,
    } led_driver_id_t;

    typedef enum {
        LedDriverPhase_Idle,
        LedDriverPhase_SetFunctionFrameInit,
        LedDriverPhase_SetFunctionFrameDisableLeds,
        LedDriverPhase_SetFunctionFrameEnableLeds,
        LedDriverPhase_SetShutdownModeNormalInit,
        LedDriverPhase_SetFrame1Init,
        LedDriverPhase_SetFrame1EnableLeds,
        LedDriverPhase_SetFrame1DisableLeds,
        LedDriverPhase_SetFrame2Init,
        LedDriverPhase_SetFrame8Init,
        LedDriverPhase_InitAutoPlayControlRegister1Init,
        LedDriverPhase_InitAutoPlayControlRegister2Init,
        LedDriverPhase_InitBreathControlRegister1Init,
        LedDriverPhase_InitBreathControlRegister2Init,
        LedDriverPhase_InitLedControlRegistersZero1Init,
        LedDriverPhase_InitLedControlRegistersZero2Init,
        LedDriverPhase_InitLedValuesInit,
        LedDriverPhase_UpdateChangedLedValuesUpdateLeds,
        LedDriverPhase_SetConfigurationRegisterDisableLeds,
        LedDriverPhase_SetConfigurationRegisterEnableLeds,
        LedDriverPhase_InitLedControlRegistersInit
    } led_driver_phase_t;

    typedef struct {
        volatile uint8_t phaseSequenceRequests;
        volatile led_driver_phase_t phase;
        volatile uint8_t targetLedValues[LED_DRIVER_LED_COUNT];
        volatile uint8_t ledIndex;
        uint8_t i2cAddress;
        uint8_t setupLedControlRegistersCommand[LED_CONTROL_REGISTERS_COMMAND_LENGTH];
    } led_driver_state_t;

// Variables:

    extern uint8_t KeyBacklightBrightness;
    extern uint8_t LedDriverValues[LED_DRIVER_MAX_COUNT][LED_DRIVER_LED_COUNT];

// Functions:

    void LedSlaveDriver_DisableLeds(uint8_t ledDriverId);
    void LedSlaveDriver_EnableLeds(uint8_t ledDriverId);
    void LedSlaveDriver_UpdateLeds(uint8_t ledDriverId);
    void LedSlaveDriver_Init(uint8_t ledDriverId);
    status_t LedSlaveDriver_Update(uint8_t ledDriverId);

#endif
