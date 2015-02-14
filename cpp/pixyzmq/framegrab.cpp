//example of using libpixyusb to grab a 1280x40 block (maximum camera resolution)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "pixy.h" 

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort //
  printf("\nBye!\n");
  exit(0);
}

int main(int argc, char * argv[])
{
  int      pixy_init_status;

  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  // Connect to Pixy //
  pixy_init_status = pixy_init();
  printf("initialized Pixy - %d\n", pixy_init_status);
  if(pixy_init_status != 0)
  {
    // Error initializing Pixy
    pixy_error(pixy_init_status);
    return pixy_init_status;
  }

  // getFrame Example //
  { //local
    unsigned char current_frame[72000]; // ~largest possible given current hardware
    unsigned char *pixels;  //returned pointer to video frame buffer
    int32_t response, fourcc;
    int8_t renderflags;
    int return_value, res;
    uint16_t width, height;
    uint32_t  numPixels;

//  stop blob processing    
    return_value = pixy_command("stop", END_OUT_ARGS, &response, END_IN_ARGS);
    printf("STOP returned %d response %d\n", return_value, response);

    response = 0;
    return_value = pixy_command("cam_getFrame",  // String id for remote procedure
                                 0x01, 0x00,      // mode 0 = 1280x800 25 fps
                                 0x02,   0,        // xoffset
                                 0x02,   0,         // yoffset
                                 0x02, 1280,       // width
                                 0x02, 40,       // height (56 max @ 1280 w)
                                 0,              // separator
                                 &response,      // pointer to mem address for return value
                                &fourcc,         //contrary to docs, the next 5 args are needed
                                &renderflags,
                                &width,
                                &height,
                                &numPixels,
                                 &pixels,        // pointer to mem address for returned frame
                                 0);

    printf("getFrame returned %d response %d\n", return_value, response);
    printf("returned w %d h %d npix %d\n",width,height,numPixels);

// quit now if not successful:
    if(return_value != 0) return return_value;

// save this block
   memcpy(&current_frame, pixels,numPixels);

   // display average and 8x8 pixel dump

   unsigned int i,j,ind,start;
   unsigned long avg=0;

   for(i=0; i<numPixels; i++) avg += current_frame[i];
   avg = avg/numPixels;
   printf("average pixel value %d\n",avg);

// dump a few raw pixels

   start=(height/2)*width+width/2; //roughly in middle of frame
   for (i=0; i<8; i++) {
        for (j=0; j<8; j++) {
            ind = i*width + j + start;
            printf(" %02x",current_frame[ind]);
        }
        printf("\n");
    }
    // Sleep for 1/10 sec //
   while(1) usleep(100000); //(exit on ^C)
  } //end local
}