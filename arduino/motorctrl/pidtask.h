#ifndef PIDTASK_H
#define PIDTASK_H

#include <Arduino.h>
#include <Task.h>

#define EMF_SAMPLE_INTERVAL 20 // 20ms -> 50Hz
#define M1_ENABLE_PIN 4
#define M2_ENABLE_PIN 7
#define M1_PWM_PIN 5
#define M2_PWM_PIN 6

#define M1_FWD_MEAS A2
#define M1_BWD_MEAS A3
#define M2_FWD_MEAS A0
#define M2_BWD_MEAS A1


// We might actually want use the canrun 
class MotorPID : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        MotorPID();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);
        virtual void setSpeeds(int16_t m1value, int16_t m2value);

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
    pinMode(M1_ENABLE_PIN, OUTPUT);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_ENABLE_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
    setSpeeds(0,0);
}

bool MotorPID::canRun(uint32_t now)
{
    // Run if 100ms has elapsed
    if ((now - last_run) >= EMF_SAMPLE_INTERVAL)
    {
        return true;
    }
    return false;
}

void MotorPID::run(uint32_t now)
{
    last_run = now;
}

void MotorPID::setSpeeds(int16_t m1value, int16_t m2value)
{
    m1_speed = constrain(m1value, -127, 127);
    m2_speed = constrain(m2value, -127, 127);

    // TODO: rewrite as register addressing
    if (m1_speed == 0)
    {
        digitalWrite(M1_ENABLE_PIN, 0);
        analogWrite(M1_PWM_PIN, 127);
    }
    else
    {
        digitalWrite(M1_ENABLE_PIN, 1);
        analogWrite(M1_PWM_PIN, 127 + m1_speed);
    }
    if (m2_speed == 0)
    {
        digitalWrite(M2_ENABLE_PIN, 0);
        analogWrite(M2_PWM_PIN, 127);
    }
    else
    {
        digitalWrite(M2_ENABLE_PIN, 1);
        analogWrite(M2_PWM_PIN, 127 + m2_speed);
    }

    Serial.println(0x6); // ACK
}


#endif
