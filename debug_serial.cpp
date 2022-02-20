#include "application.h"
#include "debug_serial.h"

CDebugSerial::CDebugSerial()
{
}

// _________________________________________________
// Fonction appelée à chaque fois qu'un octet arrive sur la RS232
void CDebugSerial::analyze(char c)
{
//  Serial.printf("%c / %d / 0x%x\n", c, c, c);
//  Serial.printf("%c\r\n", c);
  switch(c)
  {
    case 0x09 : // TABULATION
      clear_screen();
      affiche_menu();
      break;

    case 0x1b : // ESCAPE
      application.m_moteurs.CdeMoteurs(0, 0);
      break;

    case 'a' : 
      Serial.printf("%c\r\n", c);
      application.m_moteurs.CdeMoteurGauche(10);
      break;

    case 'z' : 
      Serial.printf("%c\r\n", c);
      application.m_moteurs.CdeMoteurDroit(10);
      break;

    case 's' : 
      Serial.printf("%c\r\n", c);
      application.m_strategie.m_StateMachineMain.start();
      //application.m_StateMachineExemple1.start();
      //application.m_StateMachineExemple2.start();
      break;

    case 'c':
      Serial.printf("%c\r\n", c);
      application.step_simu_codeur_quadenc(1000);
        break;

      
    default : 
      Serial.printf("%c ??\r\n", c);
      break;
  }
}


// _________________________________________________
void CDebugSerial::clear_screen()
{
  //Serial.printf("\033[2J");
  Serial.printf("\033c");
}
// _________________________________________________
void CDebugSerial::affiche_menu()
{
  Serial.printf("Liste des commandes : \r\n");
  Serial.printf("Tab...... Clear screen + affiche menu\r\n");
  Serial.printf("ESC...... Arret moteurs\r\n");
  Serial.printf("a........ Pilotage moteur G 10%\r\n");
  Serial.printf("z........ Pilotage moteur D 10%\r\n");
  Serial.printf("w........ Etat des capteurs\r\n");
  Serial.printf("s........ Start State Machine\r\n");
  Serial.printf("c........ Simu capteur encodeur\r\n");
  Serial.printf("Choix: ");
}

