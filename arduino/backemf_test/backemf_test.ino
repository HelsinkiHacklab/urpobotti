
#define PPS_SAMPLE_TIME 100
#define M1_ENABLE_PIN 4
#define M2_ENABLE_PIN 7
#define M1_PWM_PIN 5
#define M2_PWM_PIN 6
#define M1_FWD_MEAS A2
#define M1_BWD_MEAS A3
#define M2_FWD_MEAS A0
#define M2_BWD_MEAS A1


int16_t m1_speed;
int16_t m2_speed;
int16_t m1_measured;
int16_t m2_measured;


// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);


void setSpeeds(int16_t m1value, int16_t m2value)
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

    if (m1_speed > -1)
    {
        readpin = M1_FWD_MEAS;
    }
    else
    {
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
    m1_measured = avgtmp / counts;
    
    Serial.println(F("M1 measurements"));
    print_measured();
    Serial.print(F("Averaged="));
    Serial.println(m1_measured);
}


void measure_motor2()
{
    int readpin = 0;
    uint8_t hi_idx;
    uint8_t lo_idx;

    if (m2_speed > -1)
    {
        readpin = M2_FWD_MEAS;
    }
    else
    {
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
    m2_measured = avgtmp / counts;
    
    Serial.println(F("M2 measurements"));
    print_measured();
    Serial.print(F("Averaged="));
    Serial.println(m2_measured);
}


void setup()
{
    Serial.begin(115200);
    // set up the ADC
    ADCSRA &= ~PS_128;  // remove bits set by Arduino library
    // you can choose a prescaler from above.
    // PS_16, PS_32, PS_64 or PS_128
    ADCSRA |= PS_64;    // set our own prescaler to 64 

    pinMode(M1_FWD_MEAS, INPUT);
    pinMode(M1_BWD_MEAS, INPUT);
    pinMode(M2_FWD_MEAS, INPUT);
    pinMode(M2_BWD_MEAS, INPUT);

    pinMode(M1_ENABLE_PIN, OUTPUT);
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M2_ENABLE_PIN, OUTPUT);
    pinMode(M2_PWM_PIN, OUTPUT);
    setSpeeds(0,0);


    //setSpeeds(127,0);
    //setSpeeds(-127,0);
    //setSpeeds(-80,0);
    //setSpeeds(80,0);
    //setSpeeds(0,127);
    //setSpeeds(0,-127);
    //setSpeeds(0,-80);
    setSpeeds(0,80);

    Serial.println(F("Booted"));
}



// vmin 1400mV (127,127)
// vmin 880mV (80,80)
unsigned long last_measurement;
void loop()
{
    if ((millis() - last_measurement) > 20)
    {
        last_measurement = millis();
        measure_motor1();
        measure_motor2();
    }
}
