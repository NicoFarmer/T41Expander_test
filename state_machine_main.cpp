#include "application.h"
#include "state_machine_main.h"

CStateMachineMain::CStateMachineMain()
{

}

const char* CStateMachineMain::getName()
{
  return "StateMachineMain";
}

#define DUREE_MATCH (30000)
void CStateMachineMain::step()
{
  switch(m_state)
  {
    case STATE_INIT:
      gotoState(STATE_ATTENTE);
    break;

    case STATE_ATTENTE : 
      if (m_state_timer >= 1000) {
        gotoState(STATE_MATCH);
        application.m_strategie.m_StateMachineExemple1.start();
        application.m_strategie.m_StateMachineExemple2.start();
      }
    break;

    case STATE_MATCH : 
      if ((m_state_timer%1000) == 0) Serial.printf("...Heure du match : %ld\r\n", m_state_timer);
      if (m_state_timer >= DUREE_MATCH)   gotoState(STATE_FIN_MATCH);
    break;

    case STATE_FIN_MATCH : 
      Serial.printf("C'est la fin du match !!\r\n");
      Serial.printf("On eteint tous les actionneurs");
      application.m_strategie.m_StateMachineExemple1.stop();
      application.m_strategie.m_StateMachineExemple2.stop();

      gotoState(STATE_FINISHED);
    break;


    case STATE_FINISHED : 
    break;
  }
}



