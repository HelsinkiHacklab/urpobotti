#ifndef NOPIDTASK_H
#define NOPIDTASK_H

#include <Arduino.h>
#include <Task.h>

#define M1_ENABLE_PIN 5
#define M2_ENABLE_PIN 6
#define M1_PWM_PIN 3
#define M2_PWM_PIN 4

#define PWM_MIDDLE (2048)


int16_t m1_speed;
int16_t m2_speed;
int16_t m1_pwm;
int16_t m2_pwm;


void setPWMspeeds(int16_t m1value, int16_t m2value)
{
    m1_speed = constrain(m1value, -1*PWM_MIDDLE, PWM_MIDDLE);
    m2_speed = constrain(m2value, -1*PWM_MIDDLE, PWM_MIDDLE);

    // TODO: rewrite as register addressing
    if (m1_speed == 0)
    {
        m1_pwm = PWM_MIDDLE;
        digitalWrite(M1_ENABLE_PIN, 0);
    }
    else
    {
        m1_pwm = PWM_MIDDLE - m1_speed;
        digitalWrite(M1_ENABLE_PIN, 1);
    }
    analogWrite(M1_PWM_PIN, m1_pwm);
    if (m2_speed == 0)
    {
        m2_pwm = PWM_MIDDLE;
        digitalWrite(M2_ENABLE_PIN, 0);
    }
    else
    {
        m2_pwm = PWM_MIDDLE + m2_speed;
        digitalWrite(M2_ENABLE_PIN, 1);
    }
    analogWrite(M2_PWM_PIN, m2_pwm);
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
    analogWriteFrequency(3, 11718.75);
    setPWMspeeds(0,0);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
    pinMode(M1_ENABLE_PIN, OUTPUT);
    pinMode(M2_ENABLE_PIN, OUTPUT);
    setPWMspeeds(0,0);
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
