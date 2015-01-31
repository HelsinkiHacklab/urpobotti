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
        virtual void setSpeeds(int16_t m1value, int16_t m2value);
        // TODO: control methods for setting desired speeds etc and receiving new-data notifications from the AHRS task

    private:
        uint32_t last_run;
        int16_t m1target;
        int16_t m2target;
        boolean faulted;
};

MotorPID::MotorPID()
: Task()
{
    // Do we need to contruct something ?
    faulted = false;
    m1target = 0;
    m2target = 0;
}

bool MotorPID::canRun(uint32_t now)
{
    // Run if we have new pulse data
    /*
    for (uint8_t i=0; i < pulse_inputs_len; i++)
    {
        if (pulse_inputs[i].new_data)
        {
            return true;
        }
    }
    */
    // or 100ms has elapsed
    if ((now - last_run) > 1000)
    {
        return true;
    }
    return false;
}

void MotorPID::run(uint32_t now)
{
    last_run = now;

    if (   md.getM1Fault()
        || md.getM2Fault())
    {
        md.setBrakes(0, 0);
        md.setSpeeds(0, 0);
        faulted = true;
        Serial.println(F("PANIC: Motor fault!"));
        m1target = 0;
        m2target = 0;
        return;
    }

    for (uint8_t i=0; i < pulse_inputs_len; i++)
    {
        Serial.print(F("pulse_inputs["));
        Serial.print(i, DEC);
        Serial.print(F("].pulses="));
        Serial.print(pulse_inputs[i].pulses, DEC);
        if (pulse_inputs[i].new_data)
        {
            Serial.print(F(" <- NEW!"));
        }
        Serial.println("");

        pulse_inputs[i].new_data = false;

    }

    // Do something...
}

void MotorPID::setSpeeds(int16_t m1value, int16_t m2value)
{
    if (faulted)
    {
        Serial.println(0x15); // NACK
        return;
    }
    // Reverse M1 direction so we go forward on positive numbers
    md.setSpeeds(-m1value, m2value);
    Serial.println(0x6); // ACK
    return;
}



#endif
