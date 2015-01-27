// Motor controller and AHRS for Urpobotti

// IMU https://www.pololu.com/product/1265 (links to the gyro and compass libs)
// AHRS library: https://github.com/rambo/MinIMU-9-Arduino-AHRS
// Motor shield https://www.pololu.com/product/2502
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;

// Needs to be included here or linker won't find it
#include <Wire.h>
// Gyro and compass libraries
#include <L3G.h>
#include <LSM303.h>
#include <MinIMU9AHRS.h>


// Input pins for the optical encoders in the motor shafts
const uint8_t motor_opto_pins[] = { 5, 13 };
const uint8_t pulses_per_revolution = 10;
// tire size in case we need to calculate distance the tires have travelled...
const uint8_t tire_size_mm = 67;

void setup()
{
    Serial.begin(115200);
    md.init();
    MinIMU9AHRS_setup();
    Serial.println(F("Urpobotti booted"));
}

void loop()
{
    MinIMU9AHRS_loop();
    MinIMU9AHRS_printdata();
    delay(20);
}
