// Motor controller for Urpobotti

// Teensy 3.2 restart macro
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);


// Get this library from http://bleaklow.com/files/2010/Task.tar.gz (and fix WProgram.h -> Arduino.h)
// and read http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html for background and instructions
#include <Task.h>
#include <TaskScheduler.h>

#include "nopidtask.h"
MotorNOPID motorctrl;

#include "serialtask.h"
SerialReader serialreader;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    while (!Serial.dtr())
    {
        ; // Wait for connection
    }
    Serial.println();
    Serial.println(F("Board: motorctrl initializing"));

    motorctrl.setSpeeds(0,0);
    Serial.println(F("Board: motorctrl booted"));
}

void loop()
{
    // Initialise the task list and scheduler. (uitask must be the last one, otherwise it robs priority from everything else)
    Task *tasks[] = { &serialreader, &motorctrl };
    TaskScheduler sched(tasks, NUM_TASKS(tasks));

    // Run the scheduler - never returns.
    sched.run();
}

