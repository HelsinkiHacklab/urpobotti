// Motor controller and AHRS for Urpobotti

// Get this library from http://bleaklow.com/files/2010/Task.tar.gz (and fix WProgram.h -> Arduino.h)
// and read http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html for background and instructions
#include <Task.h>
#include <TaskScheduler.h>


// Pin change interrupt lib https://github.com/rambo/PinChangeInt_userData
#include "PinChangeInt_userData.h"


// Motor shield https://www.pololu.com/product/2502
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
// Reminder: speeds go from -400 to +400


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

#include "serialtask.h"
SerialReader serialreader;

void setup()
{
    Serial.begin(115200);
    // Attach pin change interrupts for the pulse inputs
    for (uint8_t i=0; i < pulse_inputs_len; i++)
    {
        pinMode(pulse_inputs[i].pin, INPUT_PULLUP);
        PCintPort::attachInterrupt(pulse_inputs[i].pin, &pulse_input_handler, RISING, &pulse_inputs[i]);
    }

    md.init();

    Serial.println(F("motorctrl booted"));
}

void loop()
{
    // Initialise the task list and scheduler. (uitask must be the last one, otherwise it robs priority from everything else)
    Task *tasks[] = { &serialreader, &motorctrl };
    TaskScheduler sched(tasks, NUM_TASKS(tasks));

    // Run the scheduler - never returns.
    sched.run();
}
