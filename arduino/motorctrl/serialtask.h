#ifndef SERIALTASK_H
#define SERIALTASK_H

#include <Arduino.h>
#include <Task.h>

#define SERIAL_PARSE_BUFFER_SIZE 27 // 25 + crlf

// We might actually want use the canrun 
class SerialReader : public Task
{
    public:
        // Create a new blinker for the specified pin and rate.
        SerialReader();
        virtual void run(uint32_t now);
        virtual bool canRun(uint32_t now);
        // TODO: control methods for setting desired speeds etc and receiving new-data notifications from the AHRS task

    private:
        void process_command();
        char parsebuffer[SERIAL_PARSE_BUFFER_SIZE+1]; // +1 for null terminator
        uint8_t incoming_position;
};

SerialReader::SerialReader()
: Task()
{
    // Do we need to contruct something ?
    incoming_position = 0;
    parsebuffer[0] = 0x0;
}

bool SerialReader::canRun(uint32_t now)
{
    return Serial.available();
}

void SerialReader::run(uint32_t now)
{
    for (uint8_t d = Serial.available(); d > 0; d--)
    {
        parsebuffer[incoming_position] = Serial.read();
        // Check for line end and in such case do special things
        if (   parsebuffer[incoming_position] == 0xA // LF
            || parsebuffer[incoming_position] == 0xD) // CR
        {
            // Command received, parse it
            parsebuffer[incoming_position] = 0x0; // null-terminate the string
            // Check that we did not miss another CR/LF
            if (   incoming_position > 0
                && (   parsebuffer[incoming_position-1] == 0xD // CR
                    || parsebuffer[incoming_position-1] == 0xA) // LF
               )
            {
                parsebuffer[incoming_position-1] = 0x0; // null-terminate the string
            }
            process_command();
            // Reset position
            incoming_position = 0;
            parsebuffer[0]=0x0;
            // PONDER: Explicitly clear the buffer with memset ??
            //memset(&parsebuffer, 0x0, sizeof(parsebuffer));
            return;
        }

        incoming_position++;

        // Sanity check buffer sizes
        if (incoming_position > SERIAL_PARSE_BUFFER_SIZE)
        {
            Serial.println(0x15); // NACK
            Serial.print(F("PANIC: No end-of-line seen and incoming_position="));
            Serial.print(incoming_position, DEC);
            Serial.println(F(" clearing buffers"));
            // Reset position
            incoming_position = 0;
            parsebuffer[0]=0x0;
            // PONDER: Explicitly clear the buffer with memset ??
            //memset(&parsebuffer, 0x0, sizeof(parsebuffer));
            return;
        }
    }
}

void SerialReader::process_command()
{
    /*
    Serial.print(F("DEBUG: parsing: "));
    Serial.println(parsebuffer);
    */
    int16_t m1value;
    int16_t m2value;
    if (sscanf(parsebuffer, "SPDS:%d,%d", &m1value, &m2value) == 2)
    {
        // TODO: Use the motorctrl to set speed in PPS
        /*
        Serial.print(F("DEBUG: setting speeds "));
        Serial.print(m1value);
        Serial.print(",");
        Serial.println(m2value);
        */
        motorctrl.setSpeeds(m1value, m2value);
        return;
    }
    if (sscanf(parsebuffer, "BRKS:%d,%d", &m1value, &m2value) == 2)
    {
        // TODO: Use the motorctrl to set brakes
        motorctrl.setBrakes(m1value, m2value);
        return;
    }
    if (sscanf(parsebuffer, "BRK1:%d,%d", &m1value, &m2value) == 2)
    {
        motorctrl.set1Brake(m1value, m2value);
        return;
    }

    int16_t Kpint, Kpdec, Kiint, Kidec, Kdint, Kddec;
    // Arduino sscanf does not support floats, use fixed point
    if (sscanf(parsebuffer, "TUN1:%d,%d.%d,%d.%d,%d.%d", &m1value, &Kpint, &Kpdec, &Kiint, &Kidec, &Kdint, &Kddec) == 7)
    {
        double Kp,Kd,Ki;
        Serial.print(F("DEBUG: Kpint="));
        Serial.print(Kpint, DEC);
        Serial.print(F(" Kpdec="));
        Serial.println(Kpdec, DEC);

        Kp = ((Kpint*100)+Kpdec)/100;
        Kd = ((Kiint*100)+Kidec)/100;
        Ki = ((Kdint*100)+Kddec)/100;
        Serial.print(F("Setting tuning values for M"));
        Serial.print(m1value, DEC);
        Serial.print(F(" Kp="));
        Serial.print(Kp, DEC);
        Serial.print(F(" Kd="));
        Serial.print(Kd, DEC);
        Serial.print(F(" Ki="));
        Serial.println(Ki, DEC);
        if (m1value == 1)
        {
            m1pid.SetTunings(Kp, Kd, Ki);
        }
        if (m1value == 2)
        {
            m2pid.SetTunings(Kp, Kd, Ki);
        }
        return;
    }


    // Do we have other command to parse ?

    // If we did not parse a command return NACK
    Serial.println(0x15); // NACK
}



#endif
