#include "strategie.h"

CStrategie::CStrategie()
{

}

// _________________________________________________
void CStrategie::init()
{
  m_StateMachineMain.init();
  m_StateMachineExemple1.init();
  m_StateMachineExemple2.init();
}

// _________________________________________________
void CStrategie::run()
{
  m_StateMachineMain.run();
  m_StateMachineExemple1.run();
  m_StateMachineExemple2.run();
}
