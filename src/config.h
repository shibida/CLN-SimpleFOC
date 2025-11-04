#ifndef CONFIG_H
#define CONFIG_H

// FOC

#define FOC_LOOP_FREQUENCY 8000 // [Hz]

// Power supply

#define POWER_SUPPLY_VOLTAGE 45.0f // [V]

// Motor 42SHD0404-22

#define MOTOR_POLE_PAIR_NUMBER      50
#define MOTOR_PHASE_RESISTANCE      2.3f    // [Ohms]
#define MOTOR_KV_RATING             NOT_SET // [RPM/V] (RMS value)
#define MOTOR_PHASE_INDUCTANCE      0.0028f // [H]
#define MOTOR_SENSOR_ALIGN_VOLTAGE  5.0f    // [V]

// Encoder

#define ENCODER_CALIBRATION_LUT_SIZE 200

#endif // CONFIG_H