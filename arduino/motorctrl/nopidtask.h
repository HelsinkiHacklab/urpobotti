#ifndef NOPIDTASK_H
#define NOPIDTASK_H

#include <Arduino.h>
#include <Task.h>

#define M1_ENABLE_PIN 5
#define M2_ENABLE_PIN 6
#define M1_PWM_PIN 3
#define M2_PWM_PIN 4

#define PWM_MIDDLE (2047)


int16_t m1_speed;
int16_t m2_speed;

void setPWMspeeds(int16_t m1value, int16_t m2value)
{
    m1_speed = constrain(m1value, -1*PWM_MIDDLE, PWM_MIDDLE);
    m2_speed = constrain(m2value, -1*PWM_MIDDLE, PWM_MIDDLE);

    // TODO: rewrite as register addressing
    if (m1_speed == 0)
    {
        digitalWrite(M1_ENABLE_PIN, 0);
        analogWrite(M1_PWM_PIN, PWM_MIDDLE);
    }
    else
    {
        digitalWrite(M1_ENABLE_PIN, 1);
        analogWrite(M1_PWM_PIN, PWM_MIDDLE + m1_speed);
    }
    if (m2_speed == 0)
    {
        digitalWrite(M2_ENABLE_PIN, 0);
        analogWrite(M2_PWM_PIN, PWM_MIDDLE);
    }
    else
    {
        digitalWrite(M2_ENABLE_PIN, 1);
        analogWrite(M2_PWM_PIN, PWM_MIDDLE + m2_speed);
    }
}

class MotorNOPID : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        MotorNOPID();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);
        virtual void setSpeeds(int16_t m1value, int16_t m2value);

    private:
        uint32_t last_run;
        boolean faulted;
};

MotorNOPID::MotorNOPID()
: Task()
{
    // Do we need to contruct something ?
    faulted = false;
    analogWriteResolution(12);
    pinMode(M1_ENABLE_PIN, OUTPUT);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_ENABLE_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
}

bool MotorNOPID::canRun(uint32_t now)
{
    // Right now we have nothing to do on iterations
    return false;
}

void MotorNOPID::run(uint32_t now)
{
    last_run = now;

}

void MotorNOPID::setSpeeds(int16_t m1value, int16_t m2value)
{
    setPWMspeeds(m1value, m2value);

    Serial.println(0x6); // ACK
}


#endif
