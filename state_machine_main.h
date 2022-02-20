#ifndef STATE_MACHINE_MAIN_H_
#define STATE_MACHINE_MAIN_H_

#include "state_machine_base.h"

class CStateMachineMain : public CStateMachineBase
{
public:
    CStateMachineMain();

    /*virtual*/ void step();

    /*virtual*/ const char* getName();

    enum {
      STATE_ATTENTE = 1,
      STATE_MATCH,
      STATE_FIN_MATCH
      
    };
};

#endif // STATE_MACHINE_MAIN_H_
