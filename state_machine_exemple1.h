#ifndef STATE_MACHINE_EXEMPLE1_H_
#define STATE_MACHINE_EXEMPLE1_H_

#include "state_machine_base.h"

class CStateMachineExemple1 : public CStateMachineBase
{
public:
    CStateMachineExemple1();

    /*virtual*/ void step();

    /*virtual*/ const char* getName();

    enum {
      STATE_1 = 1,
      STATE_2
      
    };
};

#endif // STATE_MACHINE_EXEMPLE1_H_
