
#include "Arduino.h"
#include "application.h"

CApplication application;

extern "C" int main(void)
{
  while(!Serial && millis() < 4000);
  application.init();

  while (1)
  {
     application.run();
  }
	
}

                       
