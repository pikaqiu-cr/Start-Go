#include <DynamixelShield.h>

using namespace ControlTableItem;

DynamixelShield dxl;

const uint8_t DXL_ID = 1;                 // ID du moteur (1 par défaut)
const float   DXL_PROTOCOL_VERSION = 2.0;

void setup() {
  Serial1.begin(115200);                   // règle le moniteur série sur 115200 bauds
  delay(1000);

  dxl.begin(57600);                       // vitesse par défaut des XL330
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);


  // Étape 2 : configuration
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  Serial1.println("Couple active.");
}

void loop() {
  // Étape 3 : un petit mouvement aller-retour
  Serial1.println("Position 1024");
  dxl.setGoalPosition(DXL_ID, 1024);      // valeur brute : 0 à 4095
  delay(1500);
  Serial1.print("Position lue : ");
  Serial1.println(dxl.getPresentPosition(DXL_ID));

  Serial1.println("Position 2048");
  dxl.setGoalPosition(DXL_ID, 2048);
  delay(1500);
  Serial1.print("Position lue : ");
  Serial1.println(dxl.getPresentPosition(DXL_ID));
}