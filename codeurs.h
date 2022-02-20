#ifndef CODEURS_H
#define CODEURS_H

#include <QuadEncoder.h>

class CCodeurs
{
public:
    CCodeurs();

    void init();

    long read_CodeurGauche();
    long read_CodeurDroit();
    long read_Codeur3();
    long read_Codeur4();

    void read_Codeurs_G_D(long *G, long *D);
    void read_Codeurs_3_4(long *_3, long *_4);
    

private : 
  QuadEncoder m_encoder_G;
  QuadEncoder m_encoder_D;
  QuadEncoder m_encoder_3;
  QuadEncoder m_encoder_4;
};

#endif // CODEURS_H
