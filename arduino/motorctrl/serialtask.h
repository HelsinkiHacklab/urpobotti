#ifndef SERIALTASK_H
#define SERIALTASK_H

#include <Arduino.h>
#include <Task.h>


/**
 * Parses ASCII [0-9A-F] hexadecimal to byte value
 */
inline byte ardubus_hex2byte(byte hexchar)
{
    if (   0x40 < hexchar
        && hexchar < 0x47) // A-F
    {
        return (hexchar - 0x41) + 10; 
    }
    if (   0x2f < hexchar
        && hexchar < 0x3a) // 0-9
    {
        return (hexchar - 0x30);
    }
    return 0x0; // Failure.
    
}

inline byte ardubus_hex2byte(byte hexchar0, byte hexchar1)
{
    return (ardubus_hex2byte(hexchar0) << 4) | ardubus_hex2byte(hexchar1);
}

inline int ardubus_hex2int(byte hexchar0, byte hexchar1, byte hexchar2, byte hexchar3)
{
    return ardubus_hex2byte(hexchar0, hexchar1) << 8 | ardubus_hex2byte(hexchar2, hexchar3);
}



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
    if (!Serial.dtr())
    {
        // This works poorly for teensy 3.2
        /*
        Serial.println(F("No DTR detected, rebooting"));
        CPU_RESTART
        while(1);
        */
        Serial.println(F("No DTR detected, stopping motors"));
        motorctrl.setSpeeds(0,0);
        // Wait for connection and print the message
        while (!Serial.dtr())
        {
            ; // Wait for connection
        }
        Serial.println();
        Serial.println(F("Board: motorctrl initializing"));
        
        
    }
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
            parsebuffer[0] = 0x0;
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
    /**
     *  This fscks up the stack on Teensy3.2
     *  
    if (sscanf(parsebuffer, "S:%d,%d", &m1value, &m2value) == 2)
    {
        // TODO: Use the motorctrl to set speed in PPS
        Serial.print(F("DEBUG: setting speeds "));
        Serial.print(m1value);
        Serial.print(",");
        Serial.println(m2value);
        //motorctrl.setSpeeds(m1value, m2value);
        Serial.println("D8");
        return;
    }
     */

    if (parsebuffer[0] == 'S')
    {
        m1value = ardubus_hex2int(parsebuffer[1],parsebuffer[2],parsebuffer[3],parsebuffer[4]);
        m2value = ardubus_hex2int(parsebuffer[5],parsebuffer[6],parsebuffer[7],parsebuffer[8]);
        motorctrl.setSpeeds(m1value, m2value);
        return;
    }

    // Do we have other command to parse ?

    // If we did not parse a command return NACK
    Serial.println(0x15); // NACK
}



#endif
