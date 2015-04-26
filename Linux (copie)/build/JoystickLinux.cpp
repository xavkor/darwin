#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#include <stdio.h>
#include <math.h>

#include "Joystick.h"

Joystick::Joystick()
{
   jd = -1;
   DEBUG_PRINT = false;

   for(int i = 0; i < numOfAxes; ++i)
      axis[i] = 0;
   for(int i = 0; i < numOfButtons; ++i)
      button[i] = false;
}

Joystick::~Joystick()
{
   if (jd != -1)
      close(jd);
}

// Initializes the joystick with the first joystick found.
// Returns true on success and false if no joystick was found.
bool Joystick::init()
{
   jd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);

   if (jd != -1)
      return true;
   return false;
}

// Polls the joystick state. Returns true on success and false on error (e.g. the joystick was disconnected).
bool Joystick::update()
{
   if (jd == -1)
      return false;

   struct js_event e;
   ssize_t ret;
   while((ret = read(jd, &e, sizeof(struct js_event))) > 0)
   {
      // button press or release event
      if (e.type & JS_EVENT_BUTTON && e.number < numOfButtons)
      {
         button[e.number] = e.value ? true : false;
         if(DEBUG_PRINT)
         {
            fprintf(stderr, "\r                                                           \r");
            fprintf(stderr,"button[%d]",e.number);/**/
         }
      }
      // axis position changed
      else if (e.type & JS_EVENT_AXIS && e.number < numOfAxes)
      {
         axis[e.number] = (float)e.value/32767.0;
         if(DEBUG_PRINT)
         {
            fprintf(stderr, "\r                                                           \r");
            fprintf(stderr,"axis[%d] = %lf",e.number,axis[e.number]);/**/
         }
      }
   }
   return true;
}
