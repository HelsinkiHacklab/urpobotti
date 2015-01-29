#ifndef PIDTASK_H
#define PIDTASK_H

#include <Arduino.h>
#include <Task.h>

// We might actually want use the canrun 
class MotorPID : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        MotorPID();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);
        // TODO: control methods for setting desired speeds etc and receiving new-data notifications from the AHRS task

};

MotorPID::MotorPID()
: Task()
{
    // Do we need to contruct something ?
}

bool MotorPID::canRun(uint32_t now)
{
    // TODO: determine if we want to do something (or just run this as lowest priority but that might be a bad idea
    return true;
}

void MotorPID::run(uint32_t now)
{
    // Do something...
}



#endif
