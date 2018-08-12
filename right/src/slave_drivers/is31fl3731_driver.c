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

const static led_driver_phase_sequence_t ledDriverPhaseSequenceInit = {
    (led_driver_phase_t []){
        LedDriverPhase_SetFunctionFrame,
        LedDriverPhase_SetShutdownModeNormal,
        LedDriverPhase_InitAutoPlayControlRegister1,
        LedDriverPhase_InitAutoPlayControlRegister2,
        LedDriverPhase_InitBreathControlRegister1,
        LedDriverPhase_InitBreathControlRegister2,
        LedDriverPhase_SetFrame2,
        LedDriverPhase_InitLedControlRegistersZero,
        LedDriverPhase_SetFrame8,
        LedDriverPhase_InitLedControlRegistersZero,
        LedDriverPhase_SetFrame1,
        LedDriverPhase_InitLedControlRegisters,
        LedDriverPhase_InitLedValues
    },
    13
};

const static led_driver_phase_sequence_t ledDriverPhaseSequenceUpdateLeds = {
    (led_driver_phase_t []){
        LedDriverPhase_SetFrame1,
        LedDriverPhase_UpdateChangedLedValues
    },
    2
};

const static led_driver_phase_sequence_t ledDriverPhaseSequenceDisableLeds = {
    (led_driver_phase_t []){
        LedDriverPhase_SetFunctionFrame,
        LedDriverPhase_SetConfigurationRegisterFadeOut,
    },
    2
};

const static led_driver_phase_sequence_t ledDriverPhaseSequenceEnableLeds = {
    (led_driver_phase_t []){
        LedDriverPhase_SetFunctionFrame,
        LedDriverPhase_SetConfigurationRegisterFadeIn,
    },
    2
};

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

    if (!currentLedDriverState->phaseSequence) {
        if (currentLedDriverState->phaseSequenceRequests) {
            if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_Init) {
                currentLedDriverState->phaseSequence = &ledDriverPhaseSequenceInit;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_Init;
            } else if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_EnableLeds) {
                currentLedDriverState->phaseSequence = &ledDriverPhaseSequenceEnableLeds;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_EnableLeds;
            } else if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_DisableLeds) {
                currentLedDriverState->phaseSequence = &ledDriverPhaseSequenceDisableLeds;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_DisableLeds;
            } else if (currentLedDriverState->phaseSequenceRequests & PhaseSequenceRequest_UpdateLeds) {
                currentLedDriverState->phaseSequence = &ledDriverPhaseSequenceUpdateLeds;
                currentLedDriverState->phaseSequenceRequests &= ~PhaseSequenceRequest_UpdateLeds;
            }
            currentLedDriverState->phaseSequenceIndex = 0;
        } else {
            return kStatus_Uhk_IdleSlave;
        }
    }

    switch (currentLedDriverState->phaseSequence->phases[currentLedDriverState->phaseSequenceIndex]) {
        case LedDriverPhase_SetFunctionFrame:
            // if (ledDriverId == LedDriverId_Left && !Slaves[SlaveId_LeftKeyboardHalf].isConnected) {
            //     break;
            // }
            status = I2cAsyncWrite(ledDriverAddress, setFunctionFrameBuffer, sizeof(setFunctionFrameBuffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_SetShutdownModeNormal:
            status = I2cAsyncWrite(ledDriverAddress, setShutdownModeNormalBuffer, sizeof(setShutdownModeNormalBuffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_SetFrame1:
            status = I2cAsyncWrite(ledDriverAddress, setFrame1Buffer, sizeof(setFrame1Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_SetFrame2:
            status = I2cAsyncWrite(ledDriverAddress, setFrame2Buffer, sizeof(setFrame2Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_SetFrame8:
            status = I2cAsyncWrite(ledDriverAddress, setFrame8Buffer, sizeof(setFrame8Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_InitLedControlRegisters:
            status = I2cAsyncWrite(ledDriverAddress, currentLedDriverState->setupLedControlRegistersCommand, LED_CONTROL_REGISTERS_COMMAND_LENGTH);
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_InitLedValues:
            updatePwmRegistersBuffer[0] = FRAME_REGISTER_PWM_FIRST + *ledIndex;
            uint8_t chunkSize = MIN(LED_DRIVER_LED_COUNT - *ledIndex, PMW_REGISTER_UPDATE_CHUNK_SIZE);
            memcpy(updatePwmRegistersBuffer+1, ledValues + *ledIndex, chunkSize);
            status = I2cAsyncWrite(ledDriverAddress, updatePwmRegistersBuffer, chunkSize + 1);
            *ledIndex += chunkSize;
            if (*ledIndex >= LED_DRIVER_LED_COUNT) {
                *ledIndex = 0;
                memcpy(currentLedDriverState->targetLedValues, ledValues, LED_DRIVER_LED_COUNT);
                ++currentLedDriverState->phaseSequenceIndex;
            }
            break;
        case LedDriverPhase_UpdateChangedLedValues: {
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
                ++currentLedDriverState->phaseSequenceIndex;
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
                ++currentLedDriverState->phaseSequenceIndex;
            }
            break;
        }
        case LedDriverPhase_InitAutoPlayControlRegister1:
            status = I2cAsyncWrite(ledDriverAddress, initAutoPlayControlRegister1Buffer, sizeof(initAutoPlayControlRegister1Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_InitAutoPlayControlRegister2:
            status = I2cAsyncWrite(ledDriverAddress, initAutoPlayControlRegister2Buffer, sizeof(initAutoPlayControlRegister2Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_InitBreathControlRegister1:
            status = I2cAsyncWrite(ledDriverAddress, initBreathControlRegister1Buffer, sizeof(initBreathControlRegister1Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_InitBreathControlRegister2:
            status = I2cAsyncWrite(ledDriverAddress, initBreathControlRegister2Buffer, sizeof(initBreathControlRegister2Buffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_SetConfigurationRegisterFadeOut:
            status = I2cAsyncWrite(ledDriverAddress, setConfigurationRegisterFadeOutBuffer, sizeof(setConfigurationRegisterFadeOutBuffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_SetConfigurationRegisterFadeIn:
            status = I2cAsyncWrite(ledDriverAddress, setConfigurationRegisterFadeInBuffer, sizeof(setConfigurationRegisterFadeInBuffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
        case LedDriverPhase_InitLedControlRegistersZero:
            status = I2cAsyncWrite(ledDriverAddress, initLedControlRegistersZeroBuffer, sizeof(initLedControlRegistersZeroBuffer));
            ++currentLedDriverState->phaseSequenceIndex;
            break;
    }

    if (currentLedDriverState->phaseSequenceIndex == currentLedDriverState->phaseSequence->phaseCount) {
        currentLedDriverState->phaseSequence = NULL;
    }

    return status;
}
