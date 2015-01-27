#ifndef AHRSTASK_H
#define AHRSTASK_H

#include <Arduino.h>
#include <Task.h>

// We might actually want use the canrun 
class AHRSTask : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        AHRSTask();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);

};

AHRSTask::AHRSTask()
: Task()
{
    // Do we need to contruct something ?
}

bool AHRSTask::canRun(uint32_t now)
{
    // This will return true/false depending on if we have data
    return MinIMU9AHRS_loop();
}

void AHRSTask::run(uint32_t now)
{
    // Inform the motor task about new data ??
    MinIMU9AHRS_printdata();
}

#endif
