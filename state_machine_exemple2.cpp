#include "application.h"
#include "state_machine_exemple2.h"

CStateMachineExemple2::CStateMachineExemple2()
{

}

const char* CStateMachineExemple2::getName()
{
  return "StateMachineExemple2";
}


void CStateMachineExemple2::step()
{
  switch(m_state)
  {
    case STATE_INIT:
      if (application.m_strategie.m_StateMachineExemple1.isFinished()) gotoState(STATE_1);
    break;

    case STATE_1 : 
      if (m_state_timer >= 1000) gotoState(STATE_2);
    break;

    case STATE_2 : 
      if (m_sm_timer >= 10000)   gotoState(STATE_FINISHED);
      else if (m_state_timer >= 2000) gotoState(STATE_1);
    break;

    case STATE_FINISHED : 
    break;
  }
}



