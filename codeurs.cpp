#include "codeurs.h"

#define ACTIVE_PULLS_UPS (0)

CCodeurs::CCodeurs() :
  m_encoder_G(1, 1,   2,  ACTIVE_PULLS_UPS),  // Configuration codeur1 : pin0 et 1 / Pull ups ?
  m_encoder_D(2, 3,   4,  ACTIVE_PULLS_UPS),  // Configuration codeur2 : pin2 et 3 / Pull ups ?
  m_encoder_3(3, 7,   30,  ACTIVE_PULLS_UPS),  // Configuration codeur2 : pin2 et 3 / Pull ups ?
  m_encoder_4(4, 31,  33, ACTIVE_PULLS_UPS)   // Configuration codeur2 : pin2 et 3 / Pull ups ?
{

}

// _________________________________________________
void CCodeurs::init()
{
    m_encoder_G.setInitConfig();
    m_encoder_G.init();

    m_encoder_D.setInitConfig();
    m_encoder_D.init();

   m_encoder_3.setInitConfig();
   m_encoder_3.init();

   m_encoder_4.setInitConfig();
   m_encoder_4.init();

}

// _________________________________________________
long CCodeurs::read_CodeurGauche()
{
    return m_encoder_G.read();
}

// _________________________________________________
long CCodeurs::read_CodeurDroit()
{
  return m_encoder_D.read();
}

// _________________________________________________
long CCodeurs::read_Codeur3()
{
  return m_encoder_3.read();
}

// _________________________________________________
long CCodeurs::read_Codeur4()
{
    return m_encoder_4.read();
}


// _________________________________________________
void CCodeurs::read_Codeurs_G_D(long *G, long *D)
{
    if (G) *G= read_CodeurGauche();
    if (D) *D= read_CodeurDroit();
}

// _________________________________________________
void CCodeurs::read_Codeurs_3_4(long *_3, long *_4)
{
    if (_3) *_3= read_Codeur3();
    if (_4) *_4= read_Codeur4();
}
