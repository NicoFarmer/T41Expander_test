
#include "Arduino.h"
#include "application.h"


CApplication application;

                                 
void setup()
{
 
  while(!Serial && millis() < 4000);

  application.init();
}

void loop()
{
  application.run();
}


