#include <Arduino.h>
#include <espCan.h>
#include <CRAC_utility.h>
#include <ident_crac.h>
#include <buffer_circulaire.h>
#include "math.h"
#include <mouvement.h>
#include <timerAsserBas.h>
#include <moteur.h>
#include "pins_arduino.h"
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

unsigned long lastTimeStamp = 0;
#define EVENTS 0
#define BUTTONS 0
#define JOYSTICKS 0
#define SENSORS 0

void notify();

void onConnect();
// Code à exécu
void onDisConnect();

void removePairedDevices();
void printDeviceAddress();
void setup()
{
  Serial.begin(115200);
  Moteur_Init();
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin("C8:F0:9E:2C:07:B2");
  removePairedDevices(); // This helps to solve connection issues
  Serial.print("This device MAC is: ");
  printDeviceAddress();
  Serial.println("");
  //PS4.setLed(255, 0, 255);
}

void loop()
{
}

void removePairedDevices()
{
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++)
  {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void printDeviceAddress()
{
  const uint8_t *point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++)
  {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5)
    {
      Serial.print(":");
    }
  }
}

void onConnect()
{
  Serial.println("Connected!");
  PS4.setLed(255, 0, 255);
}

void notify()
{
#if EVENTS
  boolean sqd = PS4.event.button_down.square,
          squ = PS4.event.button_up.square,
          trd = PS4.event.button_down.triangle,
          tru = PS4.event.button_up.triangle;
  if (sqd)
    Serial.println("SQUARE down");
  else if (squ)
    Serial.println("SQUARE up");
  else if (trd)
    Serial.println("TRIANGLE down");
  else if (tru)
    Serial.println("TRIANGLE up");
#endif

#if BUTTONS
  boolean sq = PS4.Square(),
          tr = PS4.Triangle(),
          cr = PS4.Cross(),
          ci = PS4.Circle();
  if (sq)
    Serial.print(" SQUARE pressed");
  if (tr)
    Serial.print(" TRIANGLE pressed");
  if (sq | tr)
    Serial.println();
  if (cr)
    Serial.print(" CROSS pressed");
  if (ci)
    Serial.print(" CIRCLE pressed");

#endif

  uint8_t r2_value = PS4.L2Value(); // L2 pour reculer
  uint8_t l2_value = PS4.R2Value(); // R2 pour avancer
  int16_t lx_value = PS4.LStickX(); // Joystick gauche pour tourner

  // Convertir les valeurs de gâchette (0 à 255) en valeurs PWM (0 à 2048)
  int pwm_l2 = -(l2_value / 255.0) * 2048; // ne pas oublier de remettre le - pour utilser l'autre gachette //l2
  int pwm_r2 = (r2_value / 255.0) * 2048; //R2

  int pwm_Mot = pwm_r2;

  if ( -pwm_l2 > pwm_r2){
    pwm_Mot = pwm_l2;
  }

  // Coefficients pour ajuster la vitesse des moteurs en fonction de la valeur des joysticks pour la direction
  // de 0 a 127, on tourne a droite et de 0 a -127, on tourne a gauche


  float coeff_D = 1 + (lx_value / 127.0); //-
  float coeff_G = 1 - (lx_value / 127.0); //+
  coeff_D = constrain(coeff_D, 0, 1);
  coeff_G = constrain(coeff_G, 0, 1);

  // write_PWMD(pwm_r2);
  // write_PWMG(pwm_l2);

  // // Utiliser les valeurs PWM pour contrôler les moteurs
  // //Marche avant
  write_PWMD(pwm_Mot * coeff_D);
  write_PWMG(pwm_Mot * coeff_G);

 // Serial.printf("lx: %4d\n", PS4.LStickX()); // Afficher la valeur du joystick gauche

  // Only needed to print the message properly on serial monitor. Else we dont need it.
  if (millis() - lastTimeStamp > 50)
  {
#if JOYSTICKS
    Serial.printf("lx:%4d,ly:%4d,rx:%4d,ry:%4d\n",
                  PS4.LStickX(),
                  PS4.LStickY(),
                  PS4.RStickX(),
                  PS4.RStickY());
#endif
#if SENSORS
    Serial.printf("gx:%5d,gy:%5d,gz:%5d,ax:%5d,ay:%5d,az:%5d\n",
                  PS4.GyrX(),
                  PS4.GyrY(),
                  PS4.GyrZ(),
                  PS4.AccX(),
                  PS4.AccY(),
                  PS4.AccZ());
#endif
    lastTimeStamp = millis();
  }
}

void onDisConnect()
{
  Serial.println("Disconnected!");
}

// void setup() {
//   Serial.begin(921600);
//   Moteur_Init();
// }

// void loop() {

//   write_PWMD(2048);
//   write_PWMG(1740);
//   delay(1000);
// }
