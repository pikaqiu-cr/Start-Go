#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeRGBLineFollower.h"

#define LF_SIZE 4
#define BLANC 1
#define NOIR  0
#define COL(s) ((s == BLANC) ? "B" : "N")

// --- Réglages ---
const unsigned long PERIODE_MESURE_MS = 5000;   // 500 ms = 20 mesures par seconde

MeRGBLineFollower RGBLineFollower(PORT_7, ADDRESS2);

uint8_t lfSensor = 0;            // 0000 S4 S3 S2 S1
int16_t offset = 0;              // écart de position calculé par la librairie
uint8_t nbBlancs = 0;            // nombre de capteurs sur du blanc
unsigned long dernierInstant = 0;

void setup() {
  Serial1.begin(9600);
  RGBLineFollower.begin();
}

void loop() {
  unsigned long maintenant = millis();

  // Exécute le bloc toutes les PERIODE_MESURE_MS
  if (maintenant - dernierInstant >= PERIODE_MESURE_MS) {
    dernierInstant += PERIODE_MESURE_MS;

    mesurer();
    decider();
    afficher();
  }

  // ici, on pourra faire autre chose sans être bloqué (moteurs, etc.)
}

void mesurer() {
  RGBLineFollower.loop();                          // met à jour les mesures
  lfSensor = RGBLineFollower.getPositionState();   // états B/N
  offset   = RGBLineFollower.getPositionOffset();  // écart de position

  nbBlancs = 0;
  for (uint8_t i = 0; i < LF_SIZE; i++) {
    if (bitRead(lfSensor, i) == BLANC) nbBlancs++;
  }
}

void decider() {
  if (nbBlancs == LF_SIZE) {
    // Les 4 capteurs sont sur la feuille : on peut dessiner
    // TODO : commander les moteurs
  } else if (nbBlancs == 0) {
    // Plus de feuille sous le capteur : arrêter ou revenir en arrière
    // TODO : arrêter les moteurs
  } else {
    // Bord de la feuille : corriger la position
    // Les bits indiquent de quel côté se trouve le bord
    // TODO : corriger le mouvement selon "offset"
  }
}

void afficher() {
  for (uint8_t i = 0; i < LF_SIZE; i++) {
    Serial1.print(COL(bitRead(lfSensor, i)));
    Serial1.print("\t");
  }
  Serial1.print("offset=");
  Serial1.println(offset);
}