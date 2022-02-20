#ifndef STRATEGIE_H
#define STRATEGIE_H

#include "state_machine_main.h"
#include "state_machine_exemple1.h"
#include "state_machine_exemple2.h"

class CStrategie
{
public:
    CStrategie();

    void init();
    void run();

    CStateMachineMain     m_StateMachineMain;
    CStateMachineExemple1 m_StateMachineExemple1;
    CStateMachineExemple2 m_StateMachineExemple2;

};

#endif // STRATEGIE_H
