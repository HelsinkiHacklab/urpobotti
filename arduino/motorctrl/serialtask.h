#ifndef SERIALTASK_H
#define SERIALTASK_H

#include <Arduino.h>
#include <Task.h>

// We might actually want use the canrun 
class SerialReader : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        SerialReader();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);
        // TODO: control methods for setting desired speeds etc and receiving new-data notifications from the AHRS task

};

SerialReader::SerialReader()
: Task()
{
    // Do we need to contruct something ?
}

bool SerialReader::canRun(uint32_t now)
{
    return Serial.available();
}

void SerialReader::run(uint32_t now)
{
    // TODO: Read the data and parse possible commands
}


#endif
