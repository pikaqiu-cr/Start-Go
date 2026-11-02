#include <DynamixelShield.h>

using namespace ControlTableItem;

DynamixelShield dxl;

const uint8_t DXL_ID = 1;
const float   VITESSE_RPM = 20.0;          // tours par minute (négatif = sens inverse)
const unsigned long DUREE_MS = 3000;       // durée de rotation : 3 secondes

unsigned long debut = 0;
bool enMarche = false;

void setup() {
  Serial1.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  dxl.torqueOff(DXL_ID);                   // obligatoire pour changer de mode
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);

  // Démarrage
  dxl.setGoalVelocity(DXL_ID, VITESSE_RPM, UNIT_RPM);
  debut = millis();
  enMarche = true;
  Serial1.println("Moteur en marche");
}

void loop() {
  // On vérifie le temps écoulé sans bloquer le programme
  if (enMarche && (millis() - debut >= DUREE_MS)) {
    dxl.setGoalVelocity(DXL_ID, 0, UNIT_RPM);   // arrêt
    enMarche = false;
    Serial1.println("Moteur arrete");
  }
}