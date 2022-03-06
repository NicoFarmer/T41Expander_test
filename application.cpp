#include <FlexiTimer2.h>
#include "application.h"
#include <Servo.h>
#include <Wire.h>
#include "TeensyID.h"
#include "InternalTemperature.h"
#include "EEPROM.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

Servo servo1;
Servo servo2;
Servo servo3;

CApplication::CApplication()
{
}

static boolean tick = 0;
void flash()
{
  tick = 1;
}


//___________________________________________________________________________
void CApplication::init(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  
  analogWriteFrequency(PIN_Mot1_PWM, 20000);  // PWM de commande moteur à 20kHz
  analogWriteFrequency(PIN_Mot2_PWM, 20000);
  analogWriteFrequency(PIN_Mot3_PWM, 20000);
  analogWriteResolution(10);  // 1024 pas de PWM
  pinMode(PIN_Mot1_PWM, OUTPUT);
  pinMode(PIN_Mot1_PWM, OUTPUT);
  pinMode(PIN_Mot1_PWM, OUTPUT);

  servo1.attach(PIN_SPWM1, 1000, 2000);
  servo2.attach(PIN_SPWM2, 1000, 2000);
  servo3.attach(PIN_SPWM3, 1000, 2000);


  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
 // Wire.setClock(100000);
  Wire.begin();

 pinMode(PIN_GPIO_2, OUTPUT);
 pinMode(PIN_GPIO_3, OUTPUT);

  FlexiTimer2::set(PERIODE_TICK, 1.0/1000, flash); // call every 10ms "ticks"
  FlexiTimer2::start();

  m_codeurs.init();
  m_debug_serial.affiche_menu();
  m_strategie.init();


  // Changement dynamique de la fréquence d'horloge
  // (peut être changée en live en plein milieu d'exécution du programme)
  // Possibilité d'ajuster la fréquence en fonction de la température du CPU
  const int CPU_freq_MHz = 150;
  set_arm_clock(CPU_freq_MHz*1000000);

 // Ecriture en EEPROM
/*
  EEPROM.update(0, 0x69);
  EEPROM.update(1, 0xA5);
  EEPROM.update(2, 0x96);
  EEPROM.update(3, 0x5A);
*/  

  m_hid_packetCount = 0;
  m_switch_clock_count = 0;
}


//___________________________________________________________________________
void CApplication::run(void)
{
  if (tick==1)
  {
    tick = 0;
    Sequenceur();
  }

  while(Serial.available() > 0) {
    char incomingByte = Serial.read();
    m_debug_serial.analyze(incomingByte);
  }

  read_raw_hid();
}

//___________________________________________________________________________
 /*!
   \brief Sequenceur de taches

   \param --
   \return --
*/
void CApplication::Sequenceur(void)
{
  static unsigned int cpt5msec = 0;
  static unsigned int cpt10msec = 0;
  static unsigned int cpt20msec = 0;
  static unsigned int cpt50msec = 0;
  static unsigned int cpt100msec = 0;
  static unsigned int cpt200msec = 0;
  static unsigned int cpt500msec = 0;
  static unsigned int cpt1sec = 0;
  static bool led1= false;
  static bool led2 = false;
  static bool led3 = false;
  static bool led_in = false;
  static bool high_speed_frequency = false;

  // ______________________________
  cpt5msec++;
  if (cpt5msec >= TEMPO_5msec) {
    cpt5msec = 0;
   m_asservissement.asserv(); 
}
  // ______________________________
  cpt10msec++;
  if (cpt10msec >= TEMPO_10msec) {
    cpt10msec = 0;
    step_simu_codeur_quadenc(-1000);
}

  // ______________________________
  cpt20msec++;
  if (cpt20msec >= TEMPO_20msec) {
    cpt20msec = 0;

    m_strategie.run();

 }


  // ______________________________
  cpt50msec++;
  if (cpt50msec >= TEMPO_50msec) {
    cpt50msec = 0;
}

  // ______________________________
  cpt100msec++;
  if (cpt100msec >= TEMPO_100msec) {
    cpt100msec = 0;

    servo1.write(50);
    servo2.write(100);
    servo3.write(180);

    analogWrite(PIN_Mot1_PWM, 10);
    analogWrite(PIN_Mot2_PWM, 512);
    analogWrite(PIN_Mot3_PWM, 1020);

    //m_power_switch.configDirections(0xFF,0);
    //m_power_switch.refreshOutputs();
  }

  // ______________________________
  cpt200msec++;
  if (cpt200msec >= TEMPO_200msec) {
    cpt200msec = 0;
    digitalWrite(PIN_LED3, led3);
    led3 = ! led3;
 }
  // ______________________________
  cpt500msec++;
  if (cpt500msec >= TEMPO_500msec) {
    cpt500msec = 0;
    Serial.printf("CodeurG=%ld / CodeurD=%ld / Codeur3=%ld /Codeur4=%ld\n\r", m_codeurs.read_CodeurGauche(), m_codeurs.read_CodeurDroit(), m_codeurs.read_Codeur3(), m_codeurs.read_Codeur4());

    digitalWrite(PIN_LED2, led2);
    led2 = ! led2;

    digitalWrite(LED_BUILTIN, led_in);
    led_in = ! led_in;

  }
  // ______________________________
  cpt1sec++;
  if (cpt1sec >= TEMPO_1sec) {
    cpt1sec = 0;
    Serial.printf("Internal Temperature = %f\n\r", InternalTemperature.readTemperatureC());
    Serial.printf("Internal EEPROM size = %d\n\r", EEPROM.length());
    Serial.printf("EEPROM[0 to 3] = 0x%x / 0x%x / 0x%x / 0x%x / 0x%x\n\r", EEPROM.read(0), EEPROM.read(1), EEPROM.read(2), EEPROM.read(3), EEPROM.read(4));

  // Ajustement de la fréquence d'horloge en fonction de la température CPU
  if (high_speed_frequency)
  {
    if (InternalTemperature.readTemperatureC() > 55) { 
      set_arm_clock(150*1000000);
      high_speed_frequency = false;
      m_switch_clock_count++;
      Serial.printf("Go to Low Speed CPU Frequency\n\r");
    }
  }
  else
  {
    if (InternalTemperature.readTemperatureC() < 42) {
      set_arm_clock(600*1000000);
      high_speed_frequency = true;
      m_switch_clock_count++;
      Serial.printf("Go to High Speed CPU Frequency\n\r");
    }
  }
  
  Serial.printf("F_CPU_ACTUAL=%u\n\r",F_CPU_ACTUAL);  // affiche la fréquence du CPU

  // Test I2C
  Wire.beginTransmission(0x50);
  Wire.write(0x55);  // address high byte
  Wire.write(0x12);  // address low byte
  Wire.endTransmission();

  digitalWrite(PIN_LED1, high_speed_frequency==true);
  //led1 = ! led1;
  
  write_raw_hid();

  read_internal_configuration();

  
  }

}





void CApplication::step_simu_codeur_quadenc(long nbre_steps)
{
   static unsigned char state=1;

   for (long i=0; i< abs(nbre_steps); i++)
   {
       if (nbre_steps > 0)
       {
           switch(state)
           {
               case 1 : 
                digitalWrite(PIN_GPIO_2, 1);
                digitalWrite(PIN_GPIO_3, 0);
                break;
        
               case 2 : 
                digitalWrite(PIN_GPIO_2, 1);
                digitalWrite(PIN_GPIO_3, 1);
                break;
        
               case 3 : 
                digitalWrite(PIN_GPIO_2, 0);
                digitalWrite(PIN_GPIO_3, 1);
                break;
        
        
               default : 
                digitalWrite(PIN_GPIO_2, 0);
                digitalWrite(PIN_GPIO_3, 0);
                state = 0;
                break;
           }
       }
       else if (nbre_steps < 0)
       {
           switch(state)
           {
               case 1 : 
                digitalWrite(PIN_GPIO_2, 0);
                digitalWrite(PIN_GPIO_3, 1);
                break;
        
               case 2 : 
                digitalWrite(PIN_GPIO_2, 1);
                digitalWrite(PIN_GPIO_3, 1);
                break;
        
               case 3 : 
                digitalWrite(PIN_GPIO_2, 1);
                digitalWrite(PIN_GPIO_3, 0);
                break;
        
        
               default : 
                digitalWrite(PIN_GPIO_2, 0);
                digitalWrite(PIN_GPIO_3, 0);
                state = 0;
                break;
           }
       }
       delayMicroseconds(1);
       state++;
   } // for i

}


void CApplication::write_raw_hid()
{
    // first 2 bytes are a signature
    long coder= m_codeurs.read_CodeurGauche();
    m_rawhid_buffer[0] = (coder>>24)&0xFF;
    m_rawhid_buffer[1] = (coder>>16)&0xFF;
    m_rawhid_buffer[2] = (coder>>8)&0xFF;
    m_rawhid_buffer[3] = coder&0xFF;

    coder= m_codeurs.read_CodeurDroit();
    m_rawhid_buffer[5] = (coder>>24)&0xFF;
    m_rawhid_buffer[6] = (coder>>16)&0xFF;
    m_rawhid_buffer[7] = (coder>>8)&0xFF;
    m_rawhid_buffer[8] = coder&0xFF;

    coder= m_codeurs.read_Codeur3();
    m_rawhid_buffer[10] = (coder>>24)&0xFF;
    m_rawhid_buffer[11] = (coder>>16)&0xFF;
    m_rawhid_buffer[12] = (coder>>8)&0xFF;
    m_rawhid_buffer[13] = coder&0xFF;
    
    coder= m_codeurs.read_Codeur4();
    m_rawhid_buffer[16] = (coder>>24)&0xFF;
    m_rawhid_buffer[17] = (coder>>16)&0xFF;
    m_rawhid_buffer[18] = (coder>>8)&0xFF;
    m_rawhid_buffer[19] = coder&0xFF;
 

    m_rawhid_buffer[25] = highByte(m_switch_clock_count);
    m_rawhid_buffer[26] = lowByte(m_switch_clock_count);

    
    // fill the rest with zeros
    for (int i=27; i<62; i++) {
      m_rawhid_buffer[i] = 0;
    }
    // and put a count of packets sent at the end
    m_rawhid_buffer[62] = highByte(m_hid_packetCount);
    m_rawhid_buffer[63] = lowByte(m_hid_packetCount);
    // actually send the packet
    int n = RawHID.send(m_rawhid_buffer, 100);
    if (n > 0) {
      Serial.print(F("Transmit rawhid packet "));
      Serial.println(m_hid_packetCount);
      m_hid_packetCount = m_hid_packetCount + 1;
    } else {
      Serial.println(F("Unable to transmit rawhid packet"));
    }
}



void CApplication::read_raw_hid()
{
  int n;
  n = RawHID.recv(m_rawhid_buffer, 0); // 0 timeout = do not wait
  if (n > 0) {
    // the computer sent a message.  Display the bits
    // of the first byte on pin 0 to 7.  Ignore the
    // other 63 bytes!
    Serial.print(F("Received packet, first byte: "));
    Serial.println((int)m_rawhid_buffer[0]);
  }
}

void CApplication::read_internal_configuration()
{
  uint8_t serial[4];
  uint8_t mac[6];
  uint32_t uid[4];
  uint8_t uuid[16];

  teensySN(serial);
  teensyMAC(mac);
  kinetisUID(uid);
  teensyUUID(uuid);
  Serial.printf("USB Serialnumber: %u \n", teensyUsbSN());
  Serial.printf("Array Serialnumber: %02X-%02X-%02X-%02X \n", serial[0], serial[1], serial[2], serial[3]);
  Serial.printf("String Serialnumber: %s\n", teensySN());
  Serial.printf("Array MAC Address: %02X:%02X:%02X:%02X:%02X:%02X \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("String MAC Address: %s\n", teensyMAC());
  Serial.printf("Array 128-bit UniqueID from chip: %08X-%08X-%08X-%08X\n", uid[0], uid[1], uid[2], uid[3]);
  Serial.printf("String 128-bit UniqueID from chip: %s\n", kinetisUID());
  Serial.printf("Array 128-bit UUID RFC4122: %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7], uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
  Serial.printf("String 128-bit UUID RFC4122: %s\n", teensyUUID());
}



