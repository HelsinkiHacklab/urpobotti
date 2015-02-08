#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pixy.h> 

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


  return 0;
}
