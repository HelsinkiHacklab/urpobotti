// Motor controller and AHRS for Urpobotti

// Get this library from http://bleaklow.com/files/2010/Task.tar.gz (and fix WProgram.h -> Arduino.h)
// and read http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html for background and instructions
#include <Task.h>
#include <TaskScheduler.h>



// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472


// Pin change interrupt lib https://github.com/rambo/PinChangeInt_userData
#include "PinChangeInt_userData.h"


// Motor shield https://www.pololu.com/product/2502
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;

// Needs to be included here or linker won't find it
#include <Wire.h>

// TODO: We probably want to actually read the IMU with the panda directly
// IMU https://www.pololu.com/product/1265 (links to the gyro and compass libs)
// Gyro and compass libraries
#include <L3G.h>
#include <LSM303.h>
// AHRS library: https://github.com/rambo/MinIMU-9-Arduino-AHRS
#include <MinIMU9AHRS.h>



// Input pins for the optical encoders in the motor shafts
const uint8_t motor_opto_pins[] = { 5, 13 };

// To hold data for reach motor encoder
typedef struct {
    const uint8_t pin;
    volatile boolean new_data;
    volatile uint8_t pulses;
} PulseInput;

// Initialize the inputs to an array
PulseInput pulse_inputs[] = {
    { 5 }, // right == M1
    { 13 }, // left == M2
};
const uint8_t pulse_inputs_len = sizeof(pulse_inputs) / sizeof(PulseInput);


const uint8_t pulses_per_revolution = 10;
// tire size in case we need to calculate distance the tires have travelled...
const uint8_t tire_size_mm = 67;

void pulse_input_handler(void* inptr)
{
    PulseInput* input = (PulseInput*)inptr;
    input->pulses += 1;
    input->new_data = true;
}

#include "pidtask.h"
MotorPID motorctrl;

#include "ahrstask.h"
AHRSTask ahrs_task;

#include "serialtask.h"
SerialReader serialreader;

void setup()
{
    Serial.begin(115200);
    // Attach pin change interrupts for the pulse inputs
    for (uint8_t i=0; i < pulse_inputs_len; i++)
    {
        PCintPort::attachInterrupt(pulse_inputs[i].pin, &pulse_input_handler, RISING, &pulse_inputs[i]);
    }

    md.init();
    MinIMU9AHRS_setup();

    Serial.println(F("Urpobotti booted"));
}

void loop()
{
    // Initialise the task list and scheduler. (uitask must be the last one, otherwise it robs priority from everything else)
    Task *tasks[] = { &serialreader, &ahrs_task, &motorctrl };
    TaskScheduler sched(tasks, NUM_TASKS(tasks));
    
    // Run the scheduler - never returns.
    sched.run();
}
