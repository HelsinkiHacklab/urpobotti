#ifndef PIDTASK_H
#define PIDTASK_H

#include <Arduino.h>
#include <Task.h>

#define PID_SAMPLE_TIME 100

double m1Setpoint, m1Input, m1Output;
PID m1pid(&m1Input, &m1Output, &m1Setpoint, 2, 5, 1, DIRECT);
double m2Setpoint, m2Input, m2Output;
PID m2pid(&m2Input, &m2Output, &m2Setpoint, 2, 5, 1, DIRECT);

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
        int16_t m1_ppstarget;
        int16_t m2_ppstarget;
        boolean faulted;
        boolean m1pid_processed;
        boolean m2pid_processed;
};

MotorPID::MotorPID()
: Task()
{
    // Do we need to contruct something ?
    faulted = false;
    m1_ppstarget = 0;
    m2_ppstarget = 0;
    // Set sampletimes
    m1pid.SetSampleTime(PID_SAMPLE_TIME);
    m2pid.SetSampleTime(PID_SAMPLE_TIME);
    // The motorcontroller has -400 to 400 range but I'm fairly sure we want to control direction separately
    m1pid.SetOutputLimits(0, 400);
    m2pid.SetOutputLimits(0, 400);
    // turn the PID on
    m1pid.SetMode(AUTOMATIC);
    m2pid.SetMode(AUTOMATIC);
}

bool MotorPID::canRun(uint32_t now)
{
    // Run if we have faulted motor controller
    if (   md.getM1Fault()
        || md.getM2Fault())
    {
        return true;
    }

    if (!m1pid_processed)
    {
        m1Input = pulse_inputs[0].pulses;
        m1pid_processed = m1pid.Compute();
    }
    if (!m2pid_processed)
    {
        m2Input = pulse_inputs[1].pulses;
        m2pid_processed = m2pid.Compute();
    }
    // Wait for both PIDs to process fully before resetting counts
    if (   m1pid_processed
        && m2pid_processed)
    {
        // Reset the pulse counts
        pulse_inputs[0].pulses = 0;
        pulse_inputs[1].pulses = 0;
        return true;
    }

    // Run if 100ms has elapsed
    if ((now - last_run) >= PID_SAMPLE_TIME)
    {
        return true;
    }
    return false;
}

void MotorPID::run(uint32_t now)
{
    last_run = now;
    // Clear the flags
    m1pid_processed = false;
    m2pid_processed = false;

    if (   md.getM1Fault()
        || md.getM2Fault())
    {
        md.setBrakes(0, 0);
        md.setSpeeds(0, 0);
        faulted = true;
        Serial.println(F("PANIC: Motor fault!"));
        m1_ppstarget = 0;
        m2_ppstarget = 0;
        return;
    }

    // Reverse M1 direction so we go forward on positive numbers
    if (m1_ppstarget < 0)
    {
        md.setM1Speed(m1Output);
    }
    else
    {
        md.setM1Speed(-m1Output);
    }
    if (m2_ppstarget < 0)
    {
        md.setM2Speed(-m2Output);
    }
    else
    {
        md.setM2Speed(m2Output);
    }
    
    Serial.print(F("M1 SetPoint="));
    Serial.print(m1Setpoint, DEC);
    Serial.print(F(" input="));
    Serial.print(m1Input, DEC);
    Serial.print(F(" output="));
    Serial.println(m1Output, DEC);

    Serial.print(F("M2 SetPoint="));
    Serial.print(m2Setpoint, DEC);
    Serial.print(F(" input="));
    Serial.print(m2Input, DEC);
    Serial.print(F(" output="));
    Serial.println(m2Output, DEC);

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

    m1_ppstarget = m1value;
    m1Setpoint = abs(m1_ppstarget);
    m2_ppstarget = m2value;
    m2Setpoint = abs(m2_ppstarget);
    
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
    m1_ppstarget = 0;
    m2_ppstarget = 0;
    md.setBrakes(m1value, m2value);
    Serial.println(0x6); // ACK
}

void MotorPID::set1Brake(uint8_t mnum, int16_t bvalue)
{
    if (mnum == 1)
    {
        // The the speed target to 0
        m1_ppstarget = 0;
        md.setM1Brake(bvalue);
        Serial.println(0x6); // ACK
        return;
    }
    if (mnum == 2)
    {
        // The the speed target to 0
        m2_ppstarget = 0;
        md.setM2Brake(bvalue);
        Serial.println(0x6); // ACK
        return;
    }
    Serial.println(0x15); // NACK
}


#endif
