
#define EMF_SAMPLE_INTERVAL 20 // 20ms -> 50Hz
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







/**
 * Below here is the tuning example mostly intact
 */

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=1.3,ki=0.75,kd=0.0;


double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = false;

void setup()
{
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;

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

  setpoint = 100;
  myPID.SetOutputLimits(-127, 127);

}

unsigned long last_measurement;
void loop()
{
  // Limit the control-loop to our back-emf measurement interval
  if ((millis() - last_measurement) > EMF_SAMPLE_INTERVAL)
  {
      last_measurement = millis();
  }
  else
  {
      return;
  }

  
  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    //input = analogRead(0);
    //measure_motor1();
    //input = m1_measured;
    measure_motor2();
    input = m2_measured;
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     //analogWrite(0,output); 
     //setSpeeds(output,0);
     setSpeeds(0,output);
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);

    Serial.print("Autotune ");
    Serial.print("kp: ");Serial.print(aTune.GetKp(), 4);Serial.print(" ");
    Serial.print("ki: ");Serial.print(aTune.GetKi(), 4);Serial.print(" ");
    Serial.print("kd: ");Serial.print(aTune.GetKd(), 4);Serial.println();
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp(), 4);Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi(), 4);Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd(), 4);Serial.println();
  }

    Serial.print("Autotune ");
    Serial.print("kp: ");Serial.print(aTune.GetKp(), 4);Serial.print(" ");
    Serial.print("ki: ");Serial.print(aTune.GetKi(), 4);Serial.print(" ");
    Serial.print("kd: ");Serial.print(aTune.GetKd(), 4);Serial.println();
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}
