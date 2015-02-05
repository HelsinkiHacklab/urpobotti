#ifndef PIDTASK_H
#define PIDTASK_H

#include <Arduino.h>
#include <Task.h>

#define PPS_SAMPLE_TIME 100
#define M1_DIR_PIN 7
#define M2_DIR_PIN 8
#define M1_PWM_PIN 10
#define M2_PWM_PIN 11

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
    pinMode(M1_DIR_PIN, OUTPUT);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
}

bool MotorPID::canRun(uint32_t now)
{
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

    pulse_inputs[0].pulses = 0;
    pulse_inputs[1].pulses = 0;
    pulse_inputs[0].new_data = false;
    pulse_inputs[1].new_data = false;
}

void MotorPID::setSpeeds(int16_t m1value, int16_t m2value)
{
    m1_speed = -m1value;
    m2_speed = m2value;

    // TODO: rewrite as register addressing
    if (m1_speed == 0)
    {
        digitalWrite(M1_DIR_PIN, 0);
        analogWrite(M1_PWM_PIN, 0);
    }
    else
    {
        if (m1_speed < 0)
        {
            digitalWrite(M1_DIR_PIN, 1);
            analogWrite(M1_PWM_PIN, 255 + m1_speed);
        }
        else
        {
            digitalWrite(M1_DIR_PIN, 0);
            analogWrite(M1_PWM_PIN, m1_speed);
        }
    }
    if (m2_speed == 0)
    {
        digitalWrite(M2_DIR_PIN, 0);
        analogWrite(M2_PWM_PIN, 0);
    }
    else
    {
        if (m2_speed < 0)
        {
            digitalWrite(M2_DIR_PIN, 1);
            analogWrite(M2_PWM_PIN, 255 + m2_speed);
        }
        else
        {
            digitalWrite(M2_DIR_PIN, 0);
            analogWrite(M2_PWM_PIN, m2_speed);
        }
    }

    Serial.println(0x6); // ACK
}


#endif
