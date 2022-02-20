#ifndef STATE_MACHINE_BASE_H_
#define STATE_MACHINE_BASE_H_

class CStateMachineBase
{
public:
    CStateMachineBase();
    virtual ~CStateMachineBase();

  
    virtual void step()=0;
    virtual const char* getName()=0;
    
    virtual void run();
    virtual void init();
    virtual void start();
    virtual void stop();
    virtual bool isActive();
    virtual bool isFinished();
    
    virtual void gotoState(int next_state);
    
protected :
    enum {
      STATE_NOT_STARTED    = -1,
      STATE_FINISHED       = -2,
      STATE_INIT           = 0
    };
    
    int   m_state;
    long  m_state_timer;          // durée depuis le début de l'état
    long  m_sm_timer;             // durée depuis le commencement de la mahchine d'état
    long  m_state_timer_start;    // Heure à laquelle l'état à été démarré
    long  m_sm_timer_start;       // Heure à laquelle la machine d'état à démarré

    
    bool  m_active_debug;   // active les messages de debug
};

#endif // STATE_MACHINE_BASE_H_
