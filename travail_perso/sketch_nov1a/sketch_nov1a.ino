#include <DynamixelShield.h>
#include <TimerThree.h>
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeRGBLineFollower.h"

#define PERIOD 500000 //500 ms
#define VITESSE_RPM 80   // tours par minute (négatif = sens inverse)
#define DUREE_MAX 5150    //3 seconde, durée de rotation
//angle 180°

#define BLANC 1
#define NOIR  0
#define COL(s) ((s == BLANC) ? "B" : "N")


using namespace ControlTableItem;
DynamixelShield dxl;
MeRGBLineFollower RGBLineFollower(PORT_7, ADDRESS2);

const uint8_t DXL_ID_1 = 5;
const uint8_t DXL_ID_2 = 4;
// const float   VITESSE_RPM = 20.0;          // tours par minute (négatif = sens inverse)
//const unsigned long DUREE_MAX_1 = 3000;       // durée de rotation : 3 secondes

volatile unsigned long debut = 0;
volatile bool enMarche = false;
volatile int sens = -1;
volatile bool detection = false;
volatile float cible = 0;

uint8_t lfSensor =0;
int16_t offset = 0;
uint8_t nbBlancs = 0;
volatile int nb_tours_2 = 0; //max 20 (50cm sur le bras, rayon de 1.75cm de l'engrenage)

//void periodic(void);
void arret_moteur_1(void);
void marche_moteur_1(int p);
void moteur_2(void);
void trouve(void);

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  RGBLineFollower.begin();

  //Timer3.initialize(PERIOD); 
  //Timer3.attachInterrupt(periodic);


  dxl.torqueOff(DXL_ID_1);                   // obligatoire pour changer de mode
  dxl.setOperatingMode(DXL_ID_1, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_1);

  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  //dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, 50);
  dxl.torqueOn(DXL_ID_2);

  dxl.setGoalVelocity(DXL_ID_1, VITESSE_RPM, UNIT_RPM);
  debut = millis();
  enMarche = true;
  Serial1.println("Moteur en marche");

  //dxl.setGoalPosition(DXL_ID_2,0,UNIT_DEGREE);
  cible = dxl.getPresentPosition(DXL_ID_2,UNIT_DEGREE);
}

void loop() {
  // put your main code here, to run repeatedly:


  RGBLineFollower.loop();                          
  lfSensor = RGBLineFollower.getPositionState();   
  offset   = RGBLineFollower.getPositionOffset();  

  nbBlancs = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (bitRead(lfSensor, i) == NOIR) nbBlancs++;
  }
  trouve();

  if (detection || nb_tours_2 >= 25){
    arret_moteur_1();
    enMarche = false;
  }

  else if ( ((millis()-debut) >= DUREE_MAX) && enMarche) {
    arret_moteur_1();
    enMarche= false;
    moteur_2();
    marche_moteur_1(sens);
    enMarche= true;
    debut = millis();

  }


}



//void periodic(){
//  RGBLineFollower.loop();                          
//  lfSensor = RGBLineFollower.getPositionState();   
//  offset   = RGBLineFollower.getPositionOffset();  

//  nbBlancs = 0;
//  for (uint8_t i = 0; i < 4; i++) {
//    if (bitRead(lfSensor, i) == NOIR) nbBlancs++;
//  }

//  for (uint8_t i = 0; i < 4; i++) {
//    Serial1.print(COL(bitRead(lfSensor, i)));
//    Serial1.print("\t");
//  }
//  Serial1.print("offset=");
//  Serial1.println(offset);
//}

void trouve(){
  if (nbBlancs == 4){
    detection = true;
  }
}

void arret_moteur_1(){
  dxl.setGoalVelocity(DXL_ID_1,0,UNIT_RPM);
  enMarche = false;
  Serial1.println("Moteur arrete");
}



void marche_moteur_1(int p){
  dxl.setGoalVelocity(DXL_ID_1, p*VITESSE_RPM, UNIT_RPM);
  sens = -sens;
  Serial1.println("Marche moteur 1");
}



void moteur_2(){
  cible += 90.0;
  dxl.setGoalPosition(DXL_ID_2,cible,UNIT_DEGREE);
  nb_tours_2 +=1;
  Serial1.println("Marche moteur 2");
}