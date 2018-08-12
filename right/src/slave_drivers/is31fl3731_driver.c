#include "config.h"
#include "slave_drivers/is31fl3731_driver.h"
#include "slave_scheduler.h"
#include "led_display.h"

enum {
    PhaseSequenceRequest_Init = 1 << 0,
    PhaseSequenceRequest_UpdateLeds = 1 << 1,
    PhaseSequenceRequest_DisableLeds = 1 << 2,
    PhaseSequenceRequest_EnableLeds = 1 << 3
};

uint8_t KeyBacklightBrightness = 0xff;
uint8_t LedDriverValues[LED_DRIVER_MAX_COUNT][LED_DRIVER_LED_COUNT];

static led_driver_state_t ledDriverStates[LED_DRIVER_MAX_COUNT] = {
    {
        .i2cAddress = I2C_ADDRESS_IS31FL3731_RIGHT,
        .setupLedControlRegistersCommand = {
            FRAME_REGISTER_LED_CONTROL_FIRST,
            0b01111111, // key row 1
            0b00000000, // no display
            0b01111111, // keys row 2
            0b00000000, // no display
            0b01111111, // keys row 3
            0b00000000, // no display
            0b01111111, // keys row 4
            0b00000000, // no display
            0b01111010, // keys row 5
            0b00000000, // no display
            0b00000000, // keys row 6
            0b00000000, // no display
            0b00000000, // keys row 7
            0b00000000, // no display
            0b00000000, // keys row 8
            0b00000000, // no display
            0b00000000, // keys row 9
            0b00000000, // no display
        }
    },
    {
        .i2cAddress = I2C_ADDRESS_IS31FL3731_LEFT,
        .setupLedControlRegistersCommand = {
            FRAME_REGISTER_LED_CONTROL_FIRST,
            0b01111111, // key row 1
            0b00111111, // display row 1
            0b01011111, // keys row 2
            0b00111111, // display row 2
            0b01011111, // keys row 3
            0b00111111, // display row 3
            0b01111101, // keys row 4
            0b00011111, // display row 4
            0b00101111, // keys row 5
            0b00011111, // display row 5
            0b00000000, // keys row 6
            0b00011111, // display row 6
            0b00000000, // keys row 7
            0b00011111, // display row 7
            0b00000000, // keys row 8
            0b00011111, // display row 8
            0b00000000, // keys row 9
            0b00011111, // display row 9
        }
    },
};

static uint8_t setFunctionFrameBuffer[] = {LED_DRIVER_REGISTER_FRAME, LED_DRIVER_FRAME_FUNCTION};
static uint8_t setShutdownModeNormalBuffer[] = {LED_DRIVER_REGISTER_SHUTDOWN, SHUTDOWN_MODE_NORMAL};
static uint8_t setFrame1Buffer[] = {LED_DRIVER_REGISTER_FRAME, LED_DRIVER_FRAME_1};
static uint8_t setFrame2Buffer[] = {LED_DRIVER_REGISTER_FRAME, LED_DRIVER_FRAME_2};
static uint8_t setFrame8Buffer[] = {LED_DRIVER_REGISTER_FRAME, LED_DRIVER_FRAME_8};
static uint8_t initLedControlRegistersZeroBuffer[19] = { FRAME_REGISTER_LED_CONTROL_FIRST };
static uint8_t setConfigurationRegisterFadeInBuffer[] = {
    LED_DRIVER_REGISTER_CONFIGURATION,
    DISPLAY_MODE_AUTO_FRAME_PLAY << DISPLAY_MODE_SHIFT | FRAME_START_8
};
static uint8_t setConfigurationRegisterFadeOutBuffer[] = {
    LED_DRIVER_REGISTER_CONFIGURATION,
    DISPLAY_MODE_AUTO_FRAME_PLAY << DISPLAY_MODE_SHIFT | FRAME_START_1
};
static uint8_t initAutoPlayControlRegister1Buffer[] = {
    LED_DRIVER_REGISTER_AUTO_PLAY_CONTROL_1,
    PLAY_LOOP_NUMBER_1 << PLAY_LOOP_NUMBER_SHIFT | PLAY_FRAME_NUMBER_1
};
static uint8_t initAutoPlayControlRegister2Buffer[] = {
    LED_DRIVER_REGISTER_AUTO_PLAY_CONTROL_2,
    FRAME_DELAY_TIME
};
static uint8_t initBreathControlRegister1Buffer[] = {
    LED_DRIVER_REGISTER_BREATH_CONTROL_1,
    FADE_OUT_TIME << FADE_OUT_TIME_SHIFT | FADE_IN_TIME
};
static uint8_t initBreathControlRegister2Buffer[] = {
    LED_DRIVER_REGISTER_BREATH_CONTROL_2,
    BREATH_ENABLE << BREATH_ENABLE_SHIFT | EXTINGUISH_TIME
};
static uint8_t updatePwmRegistersBuffer[PWM_REGISTER_BUFFER_LENGTH];

void LedSlaveDriver_DisableLeds(uint8_t ledDriverId)
{
    ledDriverStates[ledDriverId].phaseSequenceRequests |= PhaseSequenceRequest_DisableLeds;
}

void LedSlaveDriver_EnableLeds(uint8_t ledDriverId)
{
    // ledDriverStates[ledDriverId].phaseSequenceRequests |= PhaseSequenceRequest_EnableLeds;
}

void LedSlaveDriver_UpdateLeds(uint8_t ledDriverId)
{
    ledDriverStates[ledDriverId].phaseSequenceRequests |= PhaseSequenceRequest_UpdateLeds;
}

void LedSlaveDriver_Init(uint8_t ledDriverId)
{
    if (ledDriverId == ISO_KEY_LED_DRIVER_ID && IS_ISO) {
        ledDriverStates[LedDriverId_Left].setupLedControlRegistersCommand[ISO_KEY_CONTROL_REGISTER_POS] |= 1 << ISO_KEY_CONTROL_REGISTER_BIT;
    }

    led_driver_state_t *currentLedDriverState = ledDriverStates + ledDriverId;
    currentLedDriverState->ledIndex = 0;
    memset(LedDriverValues[ledDriverId], KeyBacklightBrightness, LED_DRIVER_LED_COUNT);
    ledDriverStates[ledDriverId].phaseSequenceRequests |= PhaseSequenceRequest_Init;
}

status_t LedSlaveDriver_Update(uint8_t ledDriverId)
{
    status_t status = kStatus_Uhk_IdleSlave;
    uint8_t *ledValues = LedDriverValues[ledDriverId];
    led_driver_state_t *currentLedDriverState = ledDriverStates + ledDriverId;
    uint8_t ledDriverAddress = currentLedDriverState->i2cAddress;
    uint8_t *ledIndex = &currentLedDriverState->ledIndex;

    switch (currentLedDriverState->phase) {
        case LedDriverPhase_SetFunctionFrameInit:
            // if (ledDriverId == LedDriverId_Left && !Slaves[SlaveId_LeftKeyboardHalf].isConnected) {
            //     break;
            // }
            status = I2cAsyncWrite(ledDriverAddress, setFunctionFrameBuffer, sizeof(setFunctionFrameBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetShutdownModeNormalInit;
            break;
        case LedDriverPhase_SetShutdownModeNormalInit:
            status = I2cAsyncWrite(ledDriverAddress, setShutdownModeNormalBuffer, sizeof(setShutdownModeNormalBuffer));
            currentLedDriverState->phase = LedDriverPhase_InitAutoPlayControlRegister1Init;
            break;
        case LedDriverPhase_InitAutoPlayControlRegister1Init:
            status = I2cAsyncWrite(ledDriverAddress, initAutoPlayControlRegister1Buffer, sizeof(initAutoPlayControlRegister1Buffer));
            currentLedDriverState->phase = LedDriverPhase_InitAutoPlayControlRegister2Init;
            break;
        case LedDriverPhase_InitAutoPlayControlRegister2Init:
            status = I2cAsyncWrite(ledDriverAddress, initAutoPlayControlRegister2Buffer, sizeof(initAutoPlayControlRegister2Buffer));
            currentLedDriverState->phase = LedDriverPhase_InitBreathControlRegister1Init;
            break;
        case LedDriverPhase_InitBreathControlRegister1Init:
            status = I2cAsyncWrite(ledDriverAddress, initBreathControlRegister1Buffer, sizeof(initBreathControlRegister1Buffer));
            currentLedDriverState->phase = LedDriverPhase_InitBreathControlRegister2Init;
            break;
        case LedDriverPhase_InitBreathControlRegister2Init:
            status = I2cAsyncWrite(ledDriverAddress, initBreathControlRegister2Buffer, sizeof(initBreathControlRegister2Buffer));
            currentLedDriverState->phase = LedDriverPhase_SetFrame2Init;
            break;
        case LedDriverPhase_SetFrame2Init:
            status = I2cAsyncWrite(ledDriverAddress, setFrame2Buffer, sizeof(setFrame2Buffer));
            currentLedDriverState->phase = LedDriverPhase_InitLedControlRegistersZero1Init;
            break;
        case LedDriverPhase_InitLedControlRegistersZero1Init:
            status = I2cAsyncWrite(ledDriverAddress, initLedControlRegistersZeroBuffer, sizeof(initLedControlRegistersZeroBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetFrame8Init;
            break;
        case LedDriverPhase_SetFrame8Init:
            status = I2cAsyncWrite(ledDriverAddress, setFrame8Buffer, sizeof(setFrame8Buffer));
            currentLedDriverState->phase = LedDriverPhase_InitLedControlRegistersZero2Init;
            break;
        case LedDriverPhase_InitLedControlRegistersZero2Init:
            status = I2cAsyncWrite(ledDriverAddress, initLedControlRegistersZeroBuffer, sizeof(initLedControlRegistersZeroBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetFrame1Init;
            break;
        case LedDriverPhase_SetFrame1Init:
            status = I2cAsyncWrite(ledDriverAddress, setFrame1Buffer, sizeof(setFrame1Buffer));
            currentLedDriverState->phase = LedDriverPhase_InitLedControlRegistersInit;
            break;
        case LedDriverPhase_InitLedControlRegistersInit:
            status = I2cAsyncWrite(ledDriverAddress, currentLedDriverState->setupLedControlRegistersCommand, LED_CONTROL_REGISTERS_COMMAND_LENGTH);
            currentLedDriverState->phase = LedDriverPhase_InitLedValuesInit;
            break;
        case LedDriverPhase_InitLedValuesInit:
            updatePwmRegistersBuffer[0] = FRAME_REGISTER_PWM_FIRST + *ledIndex;
            uint8_t chunkSize = MIN(LED_DRIVER_LED_COUNT - *ledIndex, PMW_REGISTER_UPDATE_CHUNK_SIZE);
            memcpy(updatePwmRegistersBuffer+1, ledValues + *ledIndex, chunkSize);
            status = I2cAsyncWrite(ledDriverAddress, updatePwmRegistersBuffer, chunkSize + 1);
            *ledIndex += chunkSize;
            if (*ledIndex >= LED_DRIVER_LED_COUNT) {
                *ledIndex = 0;
                memcpy(currentLedDriverState->targetLedValues, ledValues, LED_DRIVER_LED_COUNT);
                currentLedDriverState->phase = LedDriverPhase_Idle;
            }
            break;
        case LedDriverPhase_Idle:
            if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_Init) {
                currentLedDriverState->phase = LedDriverPhase_SetFunctionFrameInit;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_Init;
            } else if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_EnableLeds) {
                currentLedDriverState->phase = LedDriverPhase_SetFunctionFrameEnableLeds;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_EnableLeds;
            } else if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_DisableLeds) {
                currentLedDriverState->phase = LedDriverPhase_SetFunctionFrameDisableLeds;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_DisableLeds;
            } else if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_UpdateLeds) {
                currentLedDriverState->phase = LedDriverPhase_UpdateChangedLedValuesUpdateLeds;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_UpdateLeds;
            }
            break;
        case LedDriverPhase_UpdateChangedLedValuesUpdateLeds: {
            uint8_t *targetLedValues = currentLedDriverState->targetLedValues;

            uint8_t lastLedChunkStartIndex = LED_DRIVER_LED_COUNT - PMW_REGISTER_UPDATE_CHUNK_SIZE;
            uint8_t startLedIndex = *ledIndex > lastLedChunkStartIndex ? lastLedChunkStartIndex : *ledIndex;

            uint8_t count;
            for (count=0; count<LED_DRIVER_LED_COUNT; count++) {
                if (ledValues[startLedIndex] != targetLedValues[startLedIndex]) {
                    break;
                }

                if (++startLedIndex >= LED_DRIVER_LED_COUNT) {
                    startLedIndex = 0;
                }
            }

            bool foundStartIndex = count < LED_DRIVER_LED_COUNT;
            if (!foundStartIndex) {
                *ledIndex = 0;
                currentLedDriverState->phase = LedDriverPhase_Idle;
                break;
            }

            uint8_t maxChunkSize = MIN(LED_DRIVER_LED_COUNT - startLedIndex, PMW_REGISTER_UPDATE_CHUNK_SIZE);
            uint8_t maxEndLedIndex = startLedIndex + maxChunkSize - 1;
            uint8_t endLedIndex = startLedIndex;
            for (uint8_t index=startLedIndex; index<=maxEndLedIndex; index++) {
                if (ledValues[index] != targetLedValues[index]) {
                    endLedIndex = index;
                }
            }

            updatePwmRegistersBuffer[0] = FRAME_REGISTER_PWM_FIRST + startLedIndex;
            uint8_t length = endLedIndex - startLedIndex + 1;
            memcpy(updatePwmRegistersBuffer+1, ledValues + startLedIndex, length);
            memcpy(currentLedDriverState->targetLedValues + startLedIndex, ledValues + startLedIndex, length);
            status = I2cAsyncWrite(ledDriverAddress, updatePwmRegistersBuffer, length+1);
            *ledIndex += length;
            if (*ledIndex >= LED_DRIVER_LED_COUNT) {
                *ledIndex = 0;
                currentLedDriverState->phase = LedDriverPhase_Idle;
            }
            break;
        }
        case LedDriverPhase_SetFunctionFrameDisableLeds:
            status = I2cAsyncWrite(ledDriverAddress, setFunctionFrameBuffer, sizeof(setFunctionFrameBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetConfigurationRegisterDisableLeds;
            break;
        case LedDriverPhase_SetConfigurationRegisterDisableLeds:
            status = I2cAsyncWrite(ledDriverAddress, setConfigurationRegisterFadeOutBuffer, sizeof(setConfigurationRegisterFadeOutBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetFrame1DisableLeds;
            break;
        case LedDriverPhase_SetFrame1DisableLeds:
            status = I2cAsyncWrite(ledDriverAddress, setFrame1Buffer, sizeof(setFrame1Buffer));
            currentLedDriverState->phase = LedDriverPhase_Idle;
            break;
        case LedDriverPhase_SetFunctionFrameEnableLeds:
            status = I2cAsyncWrite(ledDriverAddress, setFunctionFrameBuffer, sizeof(setFunctionFrameBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetConfigurationRegisterEnableLeds;
            break;
        case LedDriverPhase_SetConfigurationRegisterEnableLeds:
            status = I2cAsyncWrite(ledDriverAddress, setConfigurationRegisterFadeInBuffer, sizeof(setConfigurationRegisterFadeInBuffer));
            currentLedDriverState->phase = LedDriverPhase_SetFrame1EnableLeds;
            break;
        case LedDriverPhase_SetFrame1EnableLeds:
            status = I2cAsyncWrite(ledDriverAddress, setFrame1Buffer, sizeof(setFrame1Buffer));
            currentLedDriverState->phase = LedDriverPhase_Idle;
            break;
    }

    return status;
}
