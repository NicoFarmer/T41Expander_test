#include "moteurs.h"

#include "application.h"

CMoteurs::CMoteurs()
{

}

// _________________________________________________
void CMoteurs::CdeMoteurDroit(float cde_pourcent)
{
    //! TODO
    Serial.printf("CdeMoteurDroit(%f)\n", cde_pourcent);
}

// _________________________________________________
void CMoteurs::CdeMoteurGauche(float cde_pourcent)
{
    //! TODO
    Serial.printf("CdeMoteurGauche(%f)\n", cde_pourcent);
}

// _________________________________________________
void CMoteurs::CdeMoteurs(float cde_gauche, float cde_droit)
{
    CdeMoteurGauche(cde_gauche);
    CdeMoteurDroit(cde_droit);
}
