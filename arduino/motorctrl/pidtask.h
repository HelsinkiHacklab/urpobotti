#ifndef PIDTASK_H
#define PIDTASK_H

#include <Arduino.h>
#include <Task.h>

#define PPS_MD_FACTOR 5
#define PPS_SAMPLE_TIME 100

// We might actually want use the canrun 
class MotorPID : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        MotorPID();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);
        virtual void setSpeeds(int16_t m1value, int16_t m2value);
        virtual void setBrakes(int16_t m1value, int16_t m2value);
        virtual void set1Brake(uint8_t mnum, int16_t bvalue);

    private:
        uint32_t last_run;
        int16_t m1_speed;
        int16_t m2_speed;
        boolean faulted;
};

MotorPID::MotorPID()
: Task()
{
    // Do we need to contruct something ?
    faulted = false;
    m1_speed = 0;
    m2_speed = 0;
}

bool MotorPID::canRun(uint32_t now)
{
    // Run if we have faulted motor controller
    if (   md.getM1Fault()
        || md.getM2Fault())
    {
        return true;
    }

    // Run if 100ms has elapsed
    if ((now - last_run) >= PPS_SAMPLE_TIME)
    {
        return true;
    }
    return false;
}

void MotorPID::run(uint32_t now)
{
    last_run = now;

    /*
    for (uint8_t i=0; i < pulse_inputs_len; i++)
    {
        Serial.print(F("DEBUG: pulse_inputs["));
        Serial.print(i, DEC);
        Serial.print(F("].pulses="));
        Serial.print(pulse_inputs[i].pulses, DEC);
        if (pulse_inputs[i].new_data)
        {
            Serial.print(F(" <- NEW"));
        }
        Serial.println("");
    }
    */
    /*
    Serial.print(F("DEBUG: M1 set speed="));
    Serial.println(m1_speed, DEC);
    Serial.print(F("DEBUG: M2 set speed="));
    Serial.println(m2_speed, DEC);
    */

    Serial.print(F("!PPS:"));
    Serial.print(pulse_inputs[0].pulses*(1000/PPS_SAMPLE_TIME), DEC);
    Serial.print(F(","));
    Serial.println(pulse_inputs[1].pulses*(1000/PPS_SAMPLE_TIME), DEC);

    if (   md.getM1Fault()
        || md.getM2Fault())
    {
        md.setBrakes(0, 0);
        md.setSpeeds(0, 0);
        faulted = true;
        Serial.println(F("PANIC: Motor fault!"));
        m1_speed = 0;
        m2_speed = 0;
        return;
    }

    pulse_inputs[0].pulses = 0;
    pulse_inputs[1].pulses = 0;
    pulse_inputs[0].new_data = false;
    pulse_inputs[1].new_data = false;
    // PONDER: measuere motor current ? md.getM2CurrentMilliamps()
    // Do something...
}

void MotorPID::setSpeeds(int16_t m1value, int16_t m2value)
{
    if (faulted)
    {
        Serial.println(0x15); // NACK
        return;
    }

    // M1 is reversed
    m1_speed = -m1value;
    m2_speed = m2value;

    md.setSpeeds(m1_speed, m2_speed);

    Serial.println(0x6); // ACK
}

void MotorPID::setBrakes(int16_t m1value, int16_t m2value)
{
    if (faulted)
    {
        Serial.println(0x15); // NACK
        return;
    }
    // The the speed targets to 0
    m1_speed = 0;
    m2_speed = 0;
    md.setBrakes(m1value, m2value);
    Serial.println(0x6); // ACK
}

void MotorPID::set1Brake(uint8_t mnum, int16_t bvalue)
{
    if (mnum == 1)
    {
        // The the speed target to 0
        m1_speed = 0;
        md.setM1Brake(bvalue);
        Serial.println(0x6); // ACK
        return;
    }
    if (mnum == 2)
    {
        // The the speed target to 0
        m2_speed = 0;
        md.setM2Brake(bvalue);
        Serial.println(0x6); // ACK
        return;
    }
    Serial.println(0x15); // NACK
}


#endif
