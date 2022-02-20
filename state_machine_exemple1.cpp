#include "application.h"
#include "state_machine_exemple1.h"

CStateMachineExemple1::CStateMachineExemple1()
{

}

const char* CStateMachineExemple1::getName()
{
  return "StateMachineExemple1";
}


void CStateMachineExemple1::step()
{
  switch(m_state)
  {
    case STATE_INIT:
      gotoState(STATE_1);
    break;

    case STATE_1 : 
      if (m_state_timer >= 1000) gotoState(STATE_2);
    break;

    case STATE_2 : 
      if (m_sm_timer >= 15000)   gotoState(STATE_FINISHED);
      else if (m_state_timer >= 2000) gotoState(STATE_1);
    break;

    case STATE_FINISHED : 
    break;
  }
}



