#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

class CDebugSerial
{
public:
    CDebugSerial();

    void analyze(char c);
    
    void affiche_menu();
    void clear_screen();

};

#endif // DEBUG_SERIAL_H
