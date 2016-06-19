// Helpers for working with Atmega8 series.
//
// Works with the following models:
//    atmega48a
//    atmega48pa
//    atmega88a
//    atmega88pa
//    atmega168a
//    atmega168pa
//    atmega328
//    atmega328p
//
// Perhaps also with models with no 'a' designator, or with 'v' designator?

// Cleaner setting of bits
#define BV(x) (1<<x)

namespace Atmega8 {

/// Possible prescaler values for timers.
enum TimerPrescalerValue {
    PSV_1,
    PSV_8,
    PSV_32,
    PSV_64,
    PSV_128,
    PSV_256,
    PSV_1024,
};

/// \enum WaveformGenerationMode
///    Possible waveform generation modes
enum WaveformGenerationMode {
    NORMAL,
    PWM_PHASE_CORRECT,
    PWM_PHASE_AND_FREQUENCY_CORRECT, // timer 1 only
    PWM_FAST,
    CTC
};

/// \enum CounterTop
///    Top value for counter
enum CounterTop {
    TOP_00FF,
    TOP_01FF, // timer 1 only
    TOP_02FF, // timer 1 only
    TOP_ICR, // timer 1 only
    TOP_OCRA
};

/// Initializes timer 0 by setting waveform generation mode and prescaler.
///
/// This function assumes that neither Clock Select nor Waveform Generation bits
/// have not been touched yet. Note that timer 0 does not support all
/// Timer0PrescalerValue values. If illegal value is entered, prescaler is not
/// set to any value.
///
/// Parameter top toggles using OCRA register values as the TOP for pulse
/// width modulation. It it only used if mode is either PWM_PHASE_CORRECT or
/// PWM_FAST.
///
/// \param prescalerValue
///    Requested prescaler value
///
/// \param mode
///    Waveform generation mode
///
/// \param top
///    Selection of counter TOP
void initializeTimer0(
    TimerPrescalerValue prescalerValue,
    WaveformGenerationMode mode,
    CounterTop top
);

/// Initializes timer 1 by setting waveform generation mode and prescaler.
///
/// This function assumes that neither Clock Select nor Waveform Generation bits
/// have not been touched yet. Note that timer 1 does not support all
/// TimerPrescalerValue values. If illegal value is entered, prescaler is not
/// set to any value.
///
/// Parameter top selects the TOP value for timer. It it only used if mode is
/// either PWM_PHASE_CORRECT, PWM_PHASE_AND_FREQUENCY_CORRECT or PWM_FAST.
///
/// \param prescalerValue
///    Requested prescaler value
///
/// \param mode
///    Waveform generation mode
///
/// \param top
///    Selection of counter TOP
void initializeTimer1(
    TimerPrescalerValue prescalerValue,
    WaveformGenerationMode mode,
    CounterTop top
);

/// Initializes timer 2 by setting waveform generation mode and prescaler.
///
/// This function assumes that neither Clock Select nor Waveform Generation bits
/// have not been touched yet.
///
/// Parameter top toggles using OCRA register values as the TOP for pulse
/// width modulation. It it only used if mode is either PWM_PHASE_CORRECT or
/// PWM_FAST.
///
/// \param prescalerValue
///    Requested prescaler value
///
/// \param mode
///    Waveform generation mode
///
/// \param top
///    Selection of counter TOP
void initializeTimer2(
    TimerPrescalerValue prescalerValue,
    WaveformGenerationMode mode,
    CounterTop top
);

// Possible voltage references
enum VoltageReference {
    // Aref pin
    VREF_AREF,
    // Vcc with external capacitor
    VREF_VCC,
    // Internal 1.1 V with external capacitor
    VREF_INTERNAL_1_1V
};

// Sets analog voltage reference to requested type.
//
// This function assumes that Reference Selection bits have not been touched
// yet.
//
// \param reference
//    Requested voltage reference
void setVoltageReference(VoltageReference reference);

// Possible analog-to-digital converter prescaler values
enum AdcPrescalerValue {
    ADC_PSV_2,
    ADC_PSV_4,
    ADC_PSV_8,
    ADC_PSV_16,
    ADC_PSV_32,
    ADC_PSV_64,
    ADC_PSV_128
};

/// \brief
///    Sets adjustment of adc results to right (default) or left.
///
/// \param leftAdjust
///    If results are left adjusted.
void leftAdjustAdcResults(bool leftAdjust);

// Sets analog-to-digital converter prescaler value.
//
// This function assumes that ADV Prescaler Selection bits have not been touched
// yet.
//
// \param value
//    Requested prescaler value
void setAdcPrescalerValue(AdcPrescalerValue value);

} // namespace