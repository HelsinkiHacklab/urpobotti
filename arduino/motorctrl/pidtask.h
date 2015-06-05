#ifndef PIDTASK_H
#define PIDTASK_H

#include <Arduino.h>
#include <Task.h>
#include <PID_v1.h>

#define M_MIN -125
#define M_MAX 125

#define EMF_SAMPLE_INTERVAL 20 // 20ms -> 50Hz
#define M1_ENABLE_PIN 4
#define M2_ENABLE_PIN 7
#define M1_PWM_PIN 5
#define M2_PWM_PIN 6

#define M1_FWD_MEAS A2
#define M1_BWD_MEAS A3
#define M2_FWD_MEAS A0
#define M2_BWD_MEAS A1

#define M1_KP 1.30
#define M1_KI 0.75
#define M1_KD 0.00 // Tunign this is hard, hope the compiler optimizes the no-op of calculation with this away
#define M2_KP 1.30
#define M2_KI 0.75
#define M2_KD 0.00 // Tunign this is hard, hope the compiler optimizes the no-op of calculation with this away

int16_t m1_speed;
int16_t m2_speed;
double m1_measured;
double m2_measured;
double m1_setpoint;
double m2_setpoint;
double m1pid_output;
double m2pid_output;

PID M1_PID(&m1_measured, &m1pid_output, &m1_setpoint,M1_KP,M1_KI,M1_KD, DIRECT);
PID M2_PID(&m2_measured, &m2pid_output, &m2_setpoint,M2_KP,M2_KI,M2_KD, DIRECT);



void setPWMspeeds(int16_t m1value, int16_t m2value)
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
}



// TODO: look at https://github.com/MajenkoLibraries/Average
#define MPOINTS 5
int16_t avgtmp;
int measured[MPOINTS];

void print_measured()
{
    for(uint8_t i=0; i < MPOINTS; i++)
    {
        Serial.print(F("measured["));
        Serial.print(i, DEC);
        Serial.print(F("]="));
        Serial.println(measured[i], DEC);
    }
}

void measure_motor1()
{
    int readpin = 0;
    uint8_t hi_idx;
    uint8_t lo_idx;
    int8_t corr;

    if (m1_speed > -1)
    {
        corr = 1;
        readpin = M1_FWD_MEAS;
    }
    else
    {
        corr = -1;
        readpin = M1_BWD_MEAS;
    }
    
    hi_idx = 0;
    lo_idx = 0;

    digitalWrite(M1_ENABLE_PIN, 0);
    // give it some time
    delayMicroseconds(100);

    // Do measurements
    for(uint8_t i=0; i < MPOINTS; i++)
    {
        measured[i] = analogRead(readpin);
        if (measured[i] >  measured[hi_idx])
        {
            hi_idx = i;
        }
        if (measured[i] <  measured[hi_idx])
        {
            lo_idx = i;
        }
    }
    if (m1_speed != 0)
    {
        digitalWrite(M1_ENABLE_PIN, 1);
    }

    // Average them (but throw away highest and lowest values)
    avgtmp = 0;
    uint8_t counts = 0;
    for(uint8_t i=0; i < MPOINTS; i++)
    {
        // If these are the same then we have 4 counts instead of 3...
        if (   i == hi_idx
            || i == lo_idx)
        {
            continue;
        }
        avgtmp += measured[i];
        counts++;
    }
    m1_measured = ((avgtmp / counts) >> 2) * corr;
    /*
    Serial.println(F("M1 measurements"));
    print_measured();
    Serial.print(F("Averaged="));
    Serial.println(m1_measured);
    */
}


void measure_motor2()
{
    int readpin = 0;
    uint8_t hi_idx;
    uint8_t lo_idx;
    int8_t corr;

    if (m2_speed > -1)
    {
        corr = 1;
        readpin = M2_FWD_MEAS;
    }
    else
    {
        corr = -1;
        readpin = M2_BWD_MEAS;
    }
    
    hi_idx = 0;
    lo_idx = 0;

    digitalWrite(M2_ENABLE_PIN, 0);
    // give it some time
    delayMicroseconds(100);

    // Do measurements
    for(uint8_t i=0; i < MPOINTS; i++)
    {
        measured[i] = analogRead(readpin);
        if (measured[i] >  measured[hi_idx])
        {
            hi_idx = i;
        }
        if (measured[i] <  measured[hi_idx])
        {
            lo_idx = i;
        }
    }
    if (m2_speed != 0)
    {
        digitalWrite(M2_ENABLE_PIN, 1);
    }

    // Average them (but throw away highest and lowest values)
    avgtmp = 0;
    uint8_t counts = 0;
    for(uint8_t i=0; i < MPOINTS; i++)
    {
        // If these are the same then we have 4 counts instead of 3...
        if (   i == hi_idx
            || i == lo_idx)
        {
            continue;
        }
        avgtmp += measured[i];
        counts++;
    }
    m2_measured = ((avgtmp / counts) >> 2) * corr;
    
    /*
    Serial.println(F("M2 measurements"));
    print_measured();
    Serial.print(F("Averaged="));
    Serial.println(m2_measured);
    */
}



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
        boolean faulted;
};

MotorPID::MotorPID()
: Task()
{
    // Do we need to contruct something ?
    faulted = false;
    pinMode(M1_ENABLE_PIN, OUTPUT);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_ENABLE_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);

    M1_PID.SetOutputLimits(-127, 127);
    M2_PID.SetOutputLimits(-127, 127);

    setSpeeds(0,0);
}

bool MotorPID::canRun(uint32_t now)
{
    // Run if EMF_SAMPLE_INTERVAL has elapsed
    if ((now - last_run) >= EMF_SAMPLE_INTERVAL)
    {
        return true;
    }
    return false;
}

void MotorPID::run(uint32_t now)
{
    last_run = now;
    measure_motor1();
    M1_PID.Compute();
    measure_motor2();
    M2_PID.Compute();
    setPWMspeeds(m1pid_output, m2pid_output);

    /*
    Serial.print("M1 ");
    Serial.print("setpoint: ");Serial.print(m1_setpoint); Serial.print(" ");
    Serial.print("input: ");Serial.print(m1_measured); Serial.print(" ");
    Serial.print("output: ");Serial.print(m1pid_output); Serial.print(" ");
    Serial.print("kp: ");Serial.print(M1_PID.GetKp(), 4);Serial.print(" ");
    Serial.print("ki: ");Serial.print(M1_PID.GetKi(), 4);Serial.print(" ");
    Serial.print("kd: ");Serial.print(M1_PID.GetKd(), 4);Serial.println();

    Serial.print("M2 ");
    Serial.print("setpoint: ");Serial.print(m2_setpoint); Serial.print(" ");
    Serial.print("input: ");Serial.print(m2_measured); Serial.print(" ");
    Serial.print("output: ");Serial.print(m2pid_output); Serial.print(" ");
    Serial.print("kp: ");Serial.print(M2_PID.GetKp(), 4);Serial.print(" ");
    Serial.print("ki: ");Serial.print(M2_PID.GetKi(), 4);Serial.print(" ");
    Serial.print("kd: ");Serial.print(M2_PID.GetKd(), 4);Serial.println();
    */
    
    // Report the back-emf based speed back
    Serial.print(F("!EMF:"));
    Serial.print(m1_measured, 0);
    Serial.print(F(","));
    Serial.println(m2_measured, 0);

}

void MotorPID::setSpeeds(int16_t m1value, int16_t m2value)
{
    m1_setpoint = constrain(m1value, M_MIN, M_MAX);
    m2_setpoint = constrain(m2value, M_MIN, M_MAX);

    if (m1_setpoint == 0)
    {
        // Turn PID off and force output to 0
        M1_PID.SetMode(MANUAL);
        m1pid_output = 0;
    }
    else
    {
        // Turn PID on
        M1_PID.SetMode(AUTOMATIC);
    }
        
    if (m2_setpoint == 0)
    {
        // Turn PID off and force output to 0
        M2_PID.SetMode(MANUAL);
        m2pid_output = 0;
    }
    else
    {
        // Turn PID on
        M2_PID.SetMode(AUTOMATIC);
    }

    Serial.println(0x6); // ACK
}


#endif
