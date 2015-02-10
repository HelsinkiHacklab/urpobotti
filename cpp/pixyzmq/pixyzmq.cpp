//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#define ZMQ_PUB_PORT "tcp://*:7580"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "pixy.h"

#include "zmq.h"

#define BLOCK_BUFFER_SIZE    25

// Pixy Block buffer // 
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

int str_to_msg(char* send, zmq_msg_t* msg)
{
    int err;
    int len = strlen(send);
    err = zmq_msg_init_size(msg, len);
    if (err != 0){
        printf("ERROR: zmq_msg_init_size failed with %s\n", zmq_strerror(zmq_errno()));
        return err;
    }
    memcpy(zmq_msg_data(msg), send, len);
    return 0;
}

int send_int_as_hex(uint16_t send, void* publisher, bool more)
{
    char buf[8];
    int err;
    zmq_msg_t msg;

    err = sprintf(buf, "0x%04x", send);
    if (err < 0)
    {
        printf("ERROR: Failed to format message\n");
        return -1;
    }
    err = str_to_msg(buf, &msg);
    if (err != 0)
    {
        zmq_msg_close(&msg);
        return err;
    }
    if (more)
    {
        err = zmq_send(publisher, &msg, ZMQ_SNDMORE);
    }
    else
    {
        err = zmq_send(publisher, &msg, 0);
    }
    zmq_msg_close(&msg);
    if (err != 0)
    {
        printf("ERROR: zmq_send failed with %s\n", zmq_strerror(zmq_errno()));
        return err;
    }
    return 0;
}

int main(int argc, char * argv[])
{
	// ZMQ variables
	int err;
	void* context = zmq_init(1);
	void* publisher = zmq_socket(context, ZMQ_PUB);

	// Prepare ZMQ context and publisher
	err = zmq_bind(publisher, ZMQ_PUB_PORT);
	if(err != 0){
		printf("zmq_bind failed with %s\n", zmq_strerror(zmq_errno()));
		zmq_close(publisher);
		zmq_term(context);
		exit(1);
	}

    zmq_msg_t msgpart;


// Pixy example below
  int      i = 0;
  int      index;
  int      blocks_copied;
  int      pixy_init_status;
  char     buf[128];

  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  printf("Hello Pixy:\n libpixyusb Version: %s\n", __LIBPIXY_VERSION__);

  // Connect to Pixy //
  pixy_init_status = pixy_init();

  // Was there an error initializing pixy? //
  if(!pixy_init_status == 0)
  {
    // Error initializing Pixy //
    printf("pixy_init(): ");
    pixy_error(pixy_init_status);

    return pixy_init_status;
  }

  // Request Pixy firmware version //
  {
    uint16_t major;
    uint16_t minor;
    uint16_t build;
    int      return_value;

    return_value = pixy_get_firmware_version(&major, &minor, &build);

    if (return_value) {
      // Error //
      printf("Failed to retrieve Pixy firmware version. ");
      pixy_error(return_value);
		zmq_close(publisher);
		zmq_term(context);

      return return_value;
    } else {
      // Success //
      printf(" Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
    }
  }

#if 0
  // Pixy Command Examples //
  {
    int32_t response;
    int     return_value;

    // Execute remote procedure call "cam_setAWB" with one output (host->pixy) parameter (Value = 1)
    //
    //   Parameters:                 Notes:
    //
    //   pixy_command("cam_setAWB",  String identifier for remote procedure
    //                        0x01,  Length (in bytes) of first output parameter
    //                           1,  Value of first output parameter
    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
    //                   &response,  Pointer to memory address for return value from remote procedure call
    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
    //

    // Enable auto white balance //
    pixy_command("cam_setAWB", UINT8(0x01), END_OUT_ARGS,  &response, END_IN_ARGS);

    // Execute remote procedure call "cam_getAWB" with no output (host->pixy) parameters
    //
    //   Parameters:                 Notes:
    //
    //   pixy_command("cam_setAWB",  String identifier for remote procedure
    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
    //                   &response,  Pointer to memory address for return value from remote procedure call
    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
    //

    // Get auto white balance //
    return_value = pixy_command("cam_getAWB", END_OUT_ARGS, &response, END_IN_ARGS);

    // Set auto white balance back to disabled //
    pixy_command("cam_setAWB", UINT8(0x00), END_OUT_ARGS,  &response, END_IN_ARGS);
  }
#endif

  printf("Detecting blocks...\n");
  while(run_flag)
  {
    // Wait for new blocks to be available //
    while(!pixy_blocks_are_new()); 

    // Get blocks from Pixy //
    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);

    if(blocks_copied < 0) {
      // Error: pixy_get_blocks //
      printf("pixy_get_blocks(): ");
      pixy_error(blocks_copied);
    }

    // Display received blocks //
    for(index = 0; index != blocks_copied; ++index)
    {
        switch (blocks[index].type)
        {
            case 0: // Normal
            {
                blocks[index].print(buf);
                printf("Sending NORM: %s\n", buf);

                // Topic
                err = str_to_msg("NORM", &msgpart);
                if (err != 0)
                {
                    zmq_msg_close(&msgpart);
                    break;
                }                
                err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
                zmq_msg_close(&msgpart);
                if (err != 0)
                {
                    break;
                }

                err = send_int_as_hex(blocks[index].signature, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].x, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].y, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].width, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].height, publisher, false);
                if (err != 0)
                {
                    break;
                }

                printf("Done\n");

                break;
            }
            case 1: // ColorCode
            {

                blocks[index].print(buf);
                printf("Sending CC: %s\n", buf);

                // Topic
                err = str_to_msg("CC", &msgpart);
                if (err != 0)
                {
                    zmq_msg_close(&msgpart);
                    break;
                }                
                err = zmq_send(publisher, &msgpart, ZMQ_SNDMORE);
                zmq_msg_close(&msgpart);
                if (err != 0)
                {
                    break;
                }

                err = send_int_as_hex(blocks[index].signature, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].x, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].y, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].width, publisher, true);
                if (err != 0)
                {
                    break;
                }
                err = send_int_as_hex(blocks[index].height, publisher, true);
                if (err != 0)
                {
                    break;
                }

                err = sprintf(buf, "%d", blocks[index].angle);
                if (err < 0)
                {
                    break;
                }
                err = str_to_msg(buf, &msgpart);
                if (err != 0)
                {
                    zmq_msg_close(&msgpart);
                    break;
                }                
                err = zmq_send(publisher, &msgpart, 0);
                zmq_msg_close(&msgpart);
                if (err != 0)
                {
                    break;
                }
                
                printf("Done\n");


                break;
            }
        }
    }
    i++;
  }
  pixy_close();


    zmq_close(publisher);
    zmq_term(context);

}
