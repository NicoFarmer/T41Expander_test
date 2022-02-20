#ifndef MOTEURS_H
#define MOTEURS_H


class CMoteurs
{
public:
    CMoteurs();

    void CdeMoteurDroit(float cde_pourcent);
    void CdeMoteurGauche(float cde_pourcent);
    void CdeMoteurs(float cde_gauche, float cde_droit);
};

#endif // MOTEURS_H
