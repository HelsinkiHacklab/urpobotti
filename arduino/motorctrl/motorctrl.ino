// Motor controller for Urpobotti

// Get this library from http://bleaklow.com/files/2010/Task.tar.gz (and fix WProgram.h -> Arduino.h)
// and read http://bleaklow.com/2010/07/20/a_very_simple_arduino_task_manager.html for background and instructions
#include <Task.h>
#include <TaskScheduler.h>

#include "pidtask.h"
MotorPID motorctrl;

#include "serialtask.h"
SerialReader serialreader;

void setup()
{
    Serial.begin(115200);

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
