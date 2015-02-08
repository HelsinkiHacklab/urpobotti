// Get this from https://code.google.com/p/arduino-new-ping/
#include <NewPing.h>

// Based on the 15 sensors example...

#define SONAR_NUM     3 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 35 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
float cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(4, 5, MAX_DISTANCE),
  NewPing(6, 7, MAX_DISTANCE)
};

void setup()
{
  Serial.begin(115200);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
  {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  Serial.println(F("ultrasounder booted"));
}


void loop() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) // Loop through all the sensors.
  {
    if (millis() >= pingTimer[i])         // Is it this sensor's time to ping?
    {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of your code would go here.
}

void echoCheck() // If ping received, set the sensor distance to array. This is called from interrupt, be quick!
{
  if (sonar[currentSensor].check_timer())
  {
    cm[currentSensor] = (float)sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }
}

void oneSensorCycle() // Sensor ping cycle complete, do something with the results.
{
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  {
    Serial.print("!PING:");
    Serial.print(i);
    Serial.print(",");
    Serial.println(cm[i]);
  }
}


