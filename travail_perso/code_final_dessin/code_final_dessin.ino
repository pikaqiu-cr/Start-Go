#include <DynamixelShield.h>
#include <Arduino.h>
#include <MeMegaPi.h>
//#include <Servo.h>

using namespace ControlTableItem;
DynamixelShield dxl;
//Servo pen;

const uint8_t DXL_ID_1 = 4;
const uint8_t DXL_ID_2 = 3;

const float D_PIGNON_X = 2.0;   // en cm
const float D_PIGNON_Y = 2.5;

const float COUNTS_PER_REV = 4096.0f;
const float CPCM_X = COUNTS_PER_REV / (PI * D_PIGNON_X);   // counts/cm
const float CPCM_Y = COUNTS_PER_REV / (PI * D_PIGNON_Y);   // vérifie que c'est voulu

const float X_MAX = 10.0;   // en cm
const float Y_MAX = 10.0;

int32_t offsetX = 0, offsetY = 0;
float curX = 0, curY = 0;   // position visée, en counts, relative à l'origine

const float V_SEUIL = 0.5;               // tr/min
const unsigned long TIMEOUT_MS = 5000;

bool a_dessiner = true;

const char* rle = "3:3:1:3:1:1:2:1:1";
int v[200];
int n = 0;
long somme = 0;             // pixels déjà traités

// Prototypes
static long presentCounts(uint8_t id);
static void setGoals(float cx, float cy);
void penUp();
void penDown();
void waitStopped();
void initialisation();
void deplacement_relatif(float dx, float dy);
void changerLigne();
void dessin_carre_pixel();
void traiterPixel(bool noir);
void dessin();

static float taille_pixel_X() { return (X_MAX / v[0]) * CPCM_X; }
static float taille_pixel_Y() { return (Y_MAX / v[1]) * CPCM_Y; }

void setup() {
  Serial1.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);

  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_1);

  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_2);

  // pen.attach(broche_du_servo);   // à ajouter quand tu gères le crayon

  char* p = (char*)rle;
  while (*p && n < 200) {
    v[n++] = strtol(p, &p, 10);
    if (*p == ':') p++;
  }

  initialisation();
}

void loop() {
  //Serial1.print("ID1 present: "); Serial1.println(dxl.getPresentPosition(DXL_ID_1));
  //Serial1.print("offsetX: ");     Serial1.println(offsetX);
  //Serial1.print("taille_pixel_X: "); Serial1.println(taille_pixel_X());

  //deplacement_relatif(taille_pixel_X(), 0);   // un seul mouvement en X

  //Serial1.print("ID1 apres: "); Serial1.println(dxl.getPresentPosition(DXL_ID_1));
  //Serial1.print("erreur lib: "); Serial1.println(dxl.getLastLibErrCode());
  if (a_dessiner) {
    dessin();
    a_dessiner = false;
  }
}

void initialisation() {
  offsetX = presentCounts(DXL_ID_1);
  offsetY = presentCounts(DXL_ID_2);
  curX = curY = 0;
  setGoals(0, 0);
}

static long presentCounts(uint8_t id) {
  return (long)lroundf(dxl.getPresentPosition(id));
}

static void setGoals(float cx, float cy) {
  dxl.setGoalPosition(DXL_ID_1, offsetX + cx, UNIT_RAW);
  dxl.setGoalPosition(DXL_ID_2, offsetY + cy, UNIT_RAW);
}

void penUp()   { /* pen.write(...); */ }
void penDown() { /* pen.write(...); */ }

void waitStopped() {
  unsigned long t0 = millis();
  while (millis() - t0 < 50) { }
  while (millis() - t0 < TIMEOUT_MS) {
    if (fabs(dxl.getPresentVelocity(DXL_ID_1, UNIT_RPM)) < V_SEUIL &&
        fabs(dxl.getPresentVelocity(DXL_ID_2, UNIT_RPM)) < V_SEUIL) return;
  }
}

// Déplacement relatif depuis la position VISÉE, avec attente intégrée
void deplacement_relatif(float dx, float dy) {
  curX += dx;
  curY += dy;
  setGoals(curX, curY);
  waitStopped();
}

// Retour au début de la ligne suivante (crayon levé avant l'appel)
void changerLigne() {
  curX = 0;
  curY += taille_pixel_Y();
  setGoals(curX, curY);
  waitStopped();
}

// Contour d'un pixel, finit au début du pixel suivant (à droite)
void dessin_carre_pixel() {
  float tx = taille_pixel_X(), ty = taille_pixel_Y();
  penDown();
  deplacement_relatif(tx, 0);
  deplacement_relatif(0, ty);
  deplacement_relatif(-tx, 0);
  deplacement_relatif(0, -ty);
  penUp();
  deplacement_relatif(tx, 0);
}

void traiterPixel(bool noir) {
  if (noir) dessin_carre_pixel();
  else      deplacement_relatif(taille_pixel_X(), 0);

  somme++;
  // saut de ligne APRÈS le dernier pixel de la ligne (sauf à la toute fin)
  if (somme % v[0] == 0 && somme < (long)v[0] * v[1]) changerLigne();
}

void dessin() {
  somme = 0;
  bool noir = (v[2] == 1);          // couleur de la première série
  penUp();
  for (int i = 3; i < n; i++) {
    for (int k = 0; k < v[i]; k++) traiterPixel(noir);
    noir = !noir;                   // la couleur alterne à chaque série
  }
  penUp();
}