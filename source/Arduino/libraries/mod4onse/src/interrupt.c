
#include "interrupt.h"

void wspeedIRQ()
{
  if ((unsigned long) (millis() - lastWindIRQ) > 10)
  {
    lastWindIRQ = millis();
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

void rainIRQ()
{
  //raintime = millis(); // grab current time

  if ((unsigned long)(millis() - rainlast) > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    lastrain += 0.2; //0.011;

    rainlast = millis(); // raintime; // set up for next event
  }
}
