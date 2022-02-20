#include "application.h"
#include "state_machine_base.h"

// ctor
CStateMachineBase::CStateMachineBase() :
    m_active_debug(true)
{

}

// dtor
CStateMachineBase::~CStateMachineBase()
{
  
}

// _________________________________________________
void CStateMachineBase::run()
{
  m_state_timer = millis() - m_state_timer_start;
  m_sm_timer = millis() - m_sm_timer_start;

  step();
}

// _________________________________________________
bool CStateMachineBase::isActive()
{
  return ( (m_state != STATE_NOT_STARTED) && (m_state != STATE_FINISHED) );
}

// _________________________________________________
bool CStateMachineBase::isFinished()
{
  return (m_state == STATE_FINISHED);
}

// _________________________________________________
void CStateMachineBase::init()
{
  m_state         = STATE_NOT_STARTED;
  m_state_timer   = 0;
  m_sm_timer      = 0;

  if (m_active_debug) {
    Serial.printf("[%ld] %s : init\r\n", millis(), getName());
  }
}

// _________________________________________________
void CStateMachineBase::start()
{
  if (m_active_debug) {
    Serial.printf("[%ld] %s : start\r\n", millis(), getName());
  }

   gotoState(STATE_INIT);
   m_sm_timer_start = millis();
}

// _________________________________________________
void CStateMachineBase::stop()
{
  m_state = STATE_NOT_STARTED;
  if (m_active_debug) {
    Serial.printf("[%ld] %s : stop\r\n", millis(), getName());
  }
  m_state_timer_start = millis();
}


// _________________________________________________
void CStateMachineBase::gotoState(int next_state)
{
  m_state_timer   = 0;
  m_state         = next_state;

  if (m_active_debug) {
    Serial.printf("[%ld] %s : gotoState %d\r\n", millis(), getName(), next_state);
  }
  m_state_timer_start = millis();
}




