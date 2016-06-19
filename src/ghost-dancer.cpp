// Controller for a dancing dress sculpture.
//
// Author: Otto Urpelainen
// Email: oturpe@iki.fi
// Date: 2016-06-18

#include "Atmega8Utils.h"

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

/// \brief
///    Toggles the indicator led state.
void toggleIndicator() {
    static bool lit;

    if (lit) {
        INDICATOR_DATA &= ~BV(INDICATOR_DATA_PIN);
    }
    else {
        INDICATOR_DATA |= BV(INDICATOR_DATA_PIN);
    }

    lit = !lit;
}

/// \brief
///    Reads both switches and returns a boolean array telling if they are
///    pressed or not.
///
/// \param switchState
///    Output parameter. Array of two booleans whose value is set to true if
///    the corresponding switch is being pressed.
void readSwitches(bool * switchState) {
    switchState[0] = SWITCH_1_INPUT & BV(SWITCH_1_INPUT_PIN);
    switchState[1] = SWITCH_2_INPUT & BV(SWITCH_2_INPUT_PIN);
}

/// \brief
///    Sets the motors to the state they should have on startup.
void initializeMotors() {
    // Set running forwards
    MOTOR_1_FORWARD_OUTPUT_COMPARE = MOTOR_1_SPEED;
    MOTOR_1_REVERSE_OUTPUT_COMPARE = 0x00;

    MOTOR_2_FORWARD_OUTPUT_COMPARE = MOTOR_1_SPEED;
    MOTOR_2_REVERSE_OUTPUT_COMPARE = 0x00;

    // Enable all outputs
    MOTOR_1_FORWARD_DATA |= BV(MOTOR_1_FORWARD_DATA_PIN);
    MOTOR_2_FORWARD_DATA |= BV(MOTOR_2_FORWARD_DATA_PIN);
    MOTOR_1_REVERSE_DATA |= BV(MOTOR_1_REVERSE_DATA_PIN);
    MOTOR_2_REVERSE_DATA |= BV(MOTOR_2_REVERSE_DATA_PIN);
    MOTOR_3_REVERSE_DATA |= BV(MOTOR_3_REVERSE_DATA_PIN);
}

/// \brief
///    Calculates the correct acceleration to use when current and target speed
///    are as given.
int16_t acceleration(uint8_t current, uint8_t target, uint8_t maxAcceleration) {
    int16_t acceleration;

    acceleration = target - current;

    if (acceleration > maxAcceleration) {
        acceleration = maxAcceleration;
    }

    return acceleration;
}

/// \brief
///    Changes motor 1 speed according to given speed target and maximum
///    acceleration.
///
/// \param forwardTarget
///    Forward target speed
///
/// \param reverseTarget
///    Reverse target speed
///
/// \param acceleration
///    Desired acceleration
void updateMotor1Speed(
    uint8_t forwardTarget,
    uint8_t reverseTarget,
    uint8_t acceleration
) {
    if (!MOTOR_1_REVERSE_OUTPUT_COMPARE) {
        MOTOR_1_FORWARD_OUTPUT_COMPARE += ::acceleration(
            MOTOR_1_FORWARD_OUTPUT_COMPARE,
            forwardTarget,
            acceleration
        );
    }
    if (!MOTOR_1_FORWARD_OUTPUT_COMPARE) {
        MOTOR_1_REVERSE_OUTPUT_COMPARE += ::acceleration(
            MOTOR_1_REVERSE_OUTPUT_COMPARE,
            reverseTarget,
            acceleration
        );
    }
}

/// \brief
///    Changes motor 2 speed according to given speed target and maximum
///    acceleration.
///
/// See updateMotor1Speed() function documentation for details. This function
/// behaves identically to that one.
///
/// \return
///    If requested speed was achieved
bool updateMotor2Speed(
    uint8_t forwardTarget,
    uint8_t reverseTarget,
    uint8_t acceleration
) {
    if (!MOTOR_2_REVERSE_OUTPUT_COMPARE) {
        MOTOR_2_FORWARD_OUTPUT_COMPARE += ::acceleration(
            MOTOR_2_FORWARD_OUTPUT_COMPARE,
            forwardTarget,
            acceleration
        );
    }
    if (!MOTOR_2_FORWARD_OUTPUT_COMPARE) {
        MOTOR_2_REVERSE_OUTPUT_COMPARE += ::acceleration(
            MOTOR_2_REVERSE_OUTPUT_COMPARE,
            reverseTarget,
            acceleration
        );
    }

    return
        MOTOR_2_FORWARD_OUTPUT_COMPARE == forwardTarget
        && MOTOR_2_REVERSE_OUTPUT_COMPARE == reverseTarget;
}

/// \brief
///    Sets motor 1 rotation according to given switch press information.
///
/// This function stores desired motor state as static variables. Every time
/// this function is called, desired state is updated if switch press
/// information requires this. Otherwise, motor speed is increased by motor
/// acceleration value as stored in config header.
///
/// \param clockwiseEnd
///    If the switch in end of clockwise motion is pressed
///
/// \param counterClockwiseEnd
///    If the switch in end of counter-clockwise motion is pressed
void controlMotor1(bool clockwiseEnd, bool counterClockwiseEnd) {
    static uint8_t forwardTarget = MOTOR_1_SPEED;
    static uint8_t reverseTarget = 0x00;

    // In clockwise end: change direction
    if(clockwiseEnd && !counterClockwiseEnd) {
        forwardTarget = MOTOR_1_SPEED;
        reverseTarget = 0x00;
    }
    // In counter-clockwise end: change direction
    else if(!clockwiseEnd && counterClockwiseEnd) {
        forwardTarget = 0x00;
        reverseTarget = MOTOR_1_SPEED;
    }
    // Should not happen: stop motor
    else if (clockwiseEnd && counterClockwiseEnd) {
        forwardTarget = 0x00;
        reverseTarget = 0x00;
    }

    updateMotor1Speed(forwardTarget, reverseTarget, MOTOR_1_ACCELERATION);
}

#define DANCE_SEQUENCE_LENGTH 50
// { forwardTarget, reverseTarget, acceleration, retain}
uint8_t danceSequence[DANCE_SEQUENCE_LENGTH][4] = {
    { 0x00, 0x00, 0x10, 40 },
    { 0x80, 0x00, 0x10, 10 },
    { 0x00, 0xd0, 0x02, 80 },
    { 0x00, 0x20, 0x40, 20 }
};

/// \brief
///    Sets motor 2 rotation following a predefined pattern.
void controlMotor2() {
    static uint8_t dancePosition = 0;
    static bool reached = true;
    static uint8_t counter = 0;

    uint8_t * current = danceSequence[dancePosition % DANCE_SEQUENCE_LENGTH];
    reached = updateMotor2Speed(current[0], current[1], current[2]);

    if (reached) {
        if (counter < current[3]) {
            counter++;
            return;
        }

        // Wait time completed
        dancePosition++;
        reached = false;
        counter = 0;
    }
}

int main() {
    INDICATOR_DATA_DIR |= BV(INDICATOR_DATA_DIR_PIN);

    SWITCH_1_DATA |= BV(SWITCH_1_DATA_PIN);
    SWITCH_2_DATA |= BV(SWITCH_2_DATA_PIN);

    MOTOR_1_FORWARD_DATA_DIR |= BV(MOTOR_1_FORWARD_DATA_DIR_PIN);
    MOTOR_1_REVERSE_DATA_DIR |= BV(MOTOR_1_REVERSE_DATA_DIR_PIN);
    MOTOR_2_FORWARD_DATA_DIR |= BV(MOTOR_2_FORWARD_DATA_DIR_PIN);
    MOTOR_2_REVERSE_DATA_DIR |= BV(MOTOR_2_REVERSE_DATA_DIR_PIN);

    Atmega8::initializeTimer0(
        PRESCALER_VALUE,
        Atmega8::PWM_PHASE_CORRECT,
        Atmega8::TOP_00FF
    );
    Atmega8::initializeTimer1(
        PRESCALER_VALUE,
        Atmega8::PWM_PHASE_CORRECT,
        Atmega8::TOP_00FF
    );
    Atmega8::initializeTimer2(
        PRESCALER_VALUE,
        Atmega8::PWM_PHASE_CORRECT,
        Atmega8::TOP_00FF
    );

    OCR1AH = 0x00;
    OCR1BH = 0x00;

    // Set non-inverting pwm
    TCCR0A |= BV(COM0A1) | BV(COM0B1);
    TCCR1A |= BV(COM1A1) | BV(COM1B1);
    TCCR2A |= BV(COM2A1) | BV(COM2B1);

    initializeMotors();

    uint32_t indicatorCounter = 0;
    uint32_t danceCounter = 0;
    uint8_t gradient = 0;

    while (true) {
        if (indicatorCounter % INDICATOR_HALF_PERIOD == 0) {
            toggleIndicator();
        }
        indicatorCounter++;

        bool switchState[6];
        readSwitches(switchState);

        controlMotor1(switchState[0], switchState[1]);
        controlMotor2();

        _delay_ms(LOOP_DELAY);
    }

}
