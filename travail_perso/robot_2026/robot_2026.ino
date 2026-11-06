#include <DynamixelShield.h>
#include <TimerThree.h>
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeRGBLineFollower.h"

using namespace ControlTableItem;
DynamixelShield dxl;
MeRGBLineFollower RGBLineFollower(PORT_7, ADDRESS2);

#define SPIN 1   // 1 = Serial1
#define MYAVAIL(sp) ((sp==0)?(Serial.available()):((sp==1)?(Serial1.available()):((sp==2)?(Serial2.available()):(Serial3.available()))))
#define MYREAD(sp) ((sp==0)?(Serial.read()):((sp==1)?(Serial1.read()):((sp==2)?(Serial2.read()):(Serial3.read()))))
#define MYREADSTRING(sp) ((sp==0)?(Serial.readString()):((sp==1)?(Serial1.readString()):((sp==2)?(Serial2.readString()):(Serial3.readString()))))

#define AVAIL() MYAVAIL(SPIN)
#define READ() MYREAD(SPIN)
#define READSTRING() MYREADSTRING(SPIN)



// ---------- Bouton ----------
#define BOUTON 51                 // à vérifier : voir remarques
#define RELACHE HIGH
#define APPUYE LOW
#define MODE_ETEINT 0
#define MODE_MARCHE 1
#define PERIODE_BOUTON_US 50000  // lecture du bouton toutes les 50 ms

volatile boolean mode = MODE_ETEINT;
volatile boolean state_prec = RELACHE;
boolean modePrecedent = MODE_ETEINT;

// ---------- Moteurs ----------
const uint8_t DXL_ID_1 = 3;   // axe X (balayage puis dessin)
const uint8_t DXL_ID_2 = 4;   // axe Y
const uint8_t DXL_ID_3 = 2;   // stylo : à vérifier
bool moteur1EnVitesse = true; // mode actuel du moteur 1

// ---------- Détection ----------
#define VITESSE_RPM 150
#define DUREE_MAX 2746.4
#define BLANC 1
#define NOIR  0

bool detection = false;
bool enMarche = false;
int sens = -1;
unsigned long debut = 0;
float cible = 0;
int nb_tours_2 = 0;
const float DECALAGE_STYLO_CM = 7.0;

uint8_t lfSensor = 0;
int16_t offset = 0;
uint8_t nbBlancs = 0;

// ---------- Dessin ----------
const float D_PIGNON_X = 2.0;    // cm
const float D_PIGNON_Y = 2.5;
const float COUNTS_PER_REV = 4096.0f;
const float CPCM_X = COUNTS_PER_REV / (PI * D_PIGNON_X);
const float CPCM_Y = COUNTS_PER_REV / (PI * D_PIGNON_Y);
const float X_MAX = 15.0;        // cm
const float Y_MAX = 15.0;

int32_t offsetX = 0, offsetY = 0;
float curX = 0, curY = 0;

const float V_SEUIL = 0.5;
const unsigned long TIMEOUT_MS = 5000;

const float PEN_UP_POS   = 1307.0;
const float PEN_DOWN_POS = 1032.0;
const float PEN_TOL      = 3.0;

int dirX = 1;

#define MAX_VAL 200

//const char* rle = "50:64:0:24:2:44:10:38:14:34:18:31:20:30:20:29:22:27:23:27:24:26:24:25:25:25:26:24:26:24:26:24:26:24:26:24:25:26:24:26:24:26:24:27:22:29:20:31:18:30:22:27:24:25:26:22:30:20:30:19:31:20:30:20:29:22:28:22:28:24:1:3:16:1:1:32:16:33:17:33:18:32:18:32:18:31:20:30:20:29:22:27:24:26:25:23:28:21:30:19:32:16:36:13:38:10:41:9:42:7:44:6:44:5:46:3:48:2:48:2:48:2:48:2:48:1:250";
int v[MAX_VAL];
int n = 0;
bool rleCharge = false;   // vrai quand une image valide est en mémoire

// ---------- État du programme ----------
enum Etat { ATTENTE, RECHERCHE, DESSIN };
Etat etat = ATTENTE;

// ---------- Prototypes ----------
static long presentCounts(uint8_t id);
char * lireChaine();
bool chargerRLE(const char* chaine);
static void setGoals(float cx, float cy);
void waitStopped();
void waitPenStopped();
void movePen(float goal);
void penUp();
void penDown();
void initialisation();
void deplacement_relatif(float dx, float dy);
void changerLigne();
void dessin_carre_pixel();
void traiterPixel(bool noir);
void dessin();
void recherche();
void trouve();
void arret_moteur_1();
void marche_moteur_1(int p);
void moteur_2();
void passerEnModePosition();
void demarrer();
void arretUrgence();
void finDeCycle(const char* message);
void changerMode();
void fctPeriodique();

static float taille_pixel_X() { return (X_MAX / v[0]) * CPCM_X; }
static float taille_pixel_Y() { return (Y_MAX / v[1]) * CPCM_Y; }

// =====================================================
void setup() {
  pinMode(BOUTON, INPUT_PULLUP);
  Serial1.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
  RGBLineFollower.begin();

  // Moteur 1 : en vitesse pour le balayage
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_1);
  moteur1EnVitesse = true;

  // Moteur 2 : en position étendue
  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_2);

  // Stylo
  dxl.torqueOff(DXL_ID_3);
  dxl.setOperatingMode(DXL_ID_3, OP_POSITION);
  dxl.torqueOn(DXL_ID_3);
  penUp();

  // Lecture du RLE
  //char* rle = lireChaine();
  //char* p = (char*)rle;
  //while (*p && n < 200) {
  //  v[n++] = strtol(p, &p, 10);
  //  if (*p == ':') p++;
  //}

  // Surveillance du bouton
  Timer3.initialize(PERIODE_BOUTON_US);
  Timer3.attachInterrupt(fctPeriodique);

  Serial1.setTimeout(200);   // readString s'arrête 200 ms après le dernier caractère (1000 ms par défaut)
  Serial1.println("Pret : envoyer le RLE puis appuyer sur le bouton");
  //Serial1.println("Pret : appuyer sur le bouton");
}

void loop() {
  // 1) Réaction à un appui sur le bouton
  // Réception d'une image, uniquement quand le robot est au repos
  if (etat == ATTENTE && mode == MODE_ETEINT) {
    char* s = lireChaine();
    if (s != NULL) {
      if (chargerRLE(s)) {
        Serial1.print("RLE charge : ");
        Serial1.print(v[0]); Serial1.print("x"); Serial1.println(v[1]);
      }
    }
  }
  if (mode != modePrecedent) {
    modePrecedent = mode;
    if (mode == MODE_MARCHE) {
      demarrer();
    } else {
      arretUrgence();
    }
  }

  if (mode != MODE_MARCHE) return;   // rien à faire tant qu'on n'a pas appuyé

  // 2) Déroulement normal
  switch (etat) {
    case RECHERCHE:
      recherche();
      break;
    case DESSIN:
      dessin();
      if (mode == MODE_MARCHE) finDeCycle("Dessin termine");  // sinon : arrêt par bouton
      break;
    default:
      break;
  }
}

// lecture rle 

bool chargerRLE(const char* chaine) {
  n = 0;
  rleCharge = false;
  const char* p = chaine;

  while (*p) {
    char* fin;
    long val = strtol(p, &fin, 10);
    if (fin == p) break;                       // plus de chiffre (fin de ligne, espace...)
    if (n >= MAX_VAL) { Serial1.println("RLE trop long"); return false; }
    v[n++] = (int)val;
    p = fin;
    if (*p == ':') p++; else break;
  }

  if (n < 4 || v[0] <= 0 || v[1] <= 0 || (v[2] != 0 && v[2] != 1)) {
    Serial1.println("RLE invalide (en-tete)");
    return false;
  }

  long total = 0;
  for (int i = 3; i < n; i++) total += v[i];
  if (total != (long)v[0] * v[1]) {
    Serial1.println("RLE invalide (somme != largeur x hauteur)");
    return false;
  }

  rleCharge = true;
  return true;
}


// =====================================================
// BOUTON (routine du timer : ne touche JAMAIS aux moteurs)
// =====================================================
void changerMode() {
  tone(8, 440, 100);
  mode = !mode;
}

void fctPeriodique() {
  boolean buttonState = digitalRead(BOUTON);
  if ((state_prec != buttonState) && (buttonState == APPUYE)) {
    changerMode();
  }
  state_prec = buttonState;
}

// =====================================================
// DÉMARRAGE / ARRÊT
// =====================================================
void demarrer() {
  if (!rleCharge) {
    Serial1.println("Aucun RLE charge : envoyez d'abord l'image");
    mode = MODE_ETEINT;
    modePrecedent = MODE_ETEINT;
    return;
  }
  penUp();

  // Le moteur 1 doit être en vitesse pour le balayage
  if (!moteur1EnVitesse) {
    dxl.torqueOff(DXL_ID_1);
    dxl.setOperatingMode(DXL_ID_1, OP_VELOCITY);
    dxl.torqueOn(DXL_ID_1);
    moteur1EnVitesse = true;
  }

  detection = false;
  nb_tours_2 = 0;
  sens = -1;
  cible = dxl.getPresentPosition(DXL_ID_2, UNIT_DEGREE);

  etat = RECHERCHE;
  dxl.setGoalVelocity(DXL_ID_1, VITESSE_RPM, UNIT_RPM);
  debut = millis();
  enMarche = true;
  Serial1.println("Recherche de la feuille");
}

void arretUrgence() {
  if (moteur1EnVitesse) {
    dxl.setGoalVelocity(DXL_ID_1, 0, UNIT_RPM);
  } else {
    dxl.setGoalPosition(DXL_ID_1, dxl.getPresentPosition(DXL_ID_1, UNIT_RAW), UNIT_RAW);
  }
  dxl.setGoalPosition(DXL_ID_2, dxl.getPresentPosition(DXL_ID_2, UNIT_RAW), UNIT_RAW);
  enMarche = false;
  etat = ATTENTE;
  penUp();
  Serial1.println("Arret");
}

// Fin normale (dessin terminé ou feuille non trouvée) : prêt pour un nouvel appui
void finDeCycle(const char* message) {
  etat = ATTENTE;
  enMarche = false;
  mode = MODE_ETEINT;
  modePrecedent = MODE_ETEINT;
  Serial1.println(message);
}

// =====================================================
// DÉTECTION
// =====================================================
void recherche() {
  RGBLineFollower.loop();
  lfSensor = RGBLineFollower.getPositionState();
  offset   = RGBLineFollower.getPositionOffset();

  nbBlancs = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (bitRead(lfSensor, i) == NOIR) nbBlancs++;   // ta logique d'origine
  }
  trouve();

  if (detection) {
    arret_moteur_1();
    waitStopped();
    passerEnModePosition();

    dirX = -sens; 
    initialisation();            // origine du dessin = position de détection
    etat = DESSIN;
    Serial1.println("Feuille trouvee");
  }
  else if (nb_tours_2 >= 25) {
    arret_moteur_1();
    finDeCycle("Feuille non trouvee");
  }
  else if (enMarche && (millis() - debut) >= DUREE_MAX) {
    arret_moteur_1();
    moteur_2();
    marche_moteur_1(sens);
    enMarche = true;
    debut = millis();
  }
}

void trouve() {
  if (nbBlancs == 4) detection = true;
}

void arret_moteur_1() {
  dxl.setGoalVelocity(DXL_ID_1, 0, UNIT_RPM);
  enMarche = false;
  Serial1.println("Moteur arrete");
}

void marche_moteur_1(int p) {
  dxl.setGoalVelocity(DXL_ID_1, p * VITESSE_RPM, UNIT_RPM);
  sens = -sens;
  Serial1.println("Marche moteur 1");
}

void moteur_2() {
  cible += 90.0;
  dxl.setGoalPosition(DXL_ID_2, cible, UNIT_DEGREE);
  nb_tours_2 += 1;
  Serial1.println("Marche moteur 2");
}

void passerEnModePosition() {
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_EXTENDED_POSITION);
  // cible = position actuelle, pour éviter un saut vers une ancienne consigne
  dxl.setGoalPosition(DXL_ID_1, dxl.getPresentPosition(DXL_ID_1, UNIT_RAW), UNIT_RAW);
  dxl.torqueOn(DXL_ID_1);
  moteur1EnVitesse = false;
}

// =====================================================
// DESSIN
// =====================================================
void initialisation() {
  offsetX = presentCounts(DXL_ID_1);
  offsetY = presentCounts(DXL_ID_2);


  offsetY += (long)lroundf(DECALAGE_STYLO_CM * CPCM_Y);
  if (dirX < 0) {
    // on se décale d'une largeur de dessin vers la gauche pour partir du bord
    offsetX -= (long)lroundf(taille_pixel_X() * v[0]);
  }

  curX = curY = 0;
  setGoals(0, 0);     // va au point de départ
  waitStopped();
}

static long presentCounts(uint8_t id) {
  return (long)lroundf(dxl.getPresentPosition(id));
}

static void setGoals(float cx, float cy) {
  dxl.setGoalPosition(DXL_ID_1, offsetX + cx, UNIT_RAW);
  dxl.setGoalPosition(DXL_ID_2, offsetY + cy, UNIT_RAW);
}

void waitStopped() {
  unsigned long t0 = millis();
  while (millis() - t0 < 50) { }
  while (millis() - t0 < TIMEOUT_MS) {
    if (fabs(dxl.getPresentVelocity(DXL_ID_1, UNIT_RPM)) < V_SEUIL &&
        fabs(dxl.getPresentVelocity(DXL_ID_2, UNIT_RPM)) < V_SEUIL) return;
  }
}

void waitPenStopped() {
  unsigned long t0 = millis();
  while (millis() - t0 < 50) { }
  while (millis() - t0 < TIMEOUT_MS) {
    if (fabs(dxl.getPresentVelocity(DXL_ID_3, UNIT_RPM)) < V_SEUIL) return;
  }
}

void movePen(float goal) {
  float t = dxl.getPresentPosition(DXL_ID_3, UNIT_RAW);
  if (fabs(t - goal) > PEN_TOL) {
    dxl.setGoalPosition(DXL_ID_3, goal, UNIT_RAW);
    waitPenStopped();
  }
}

void penUp()   { movePen(PEN_UP_POS); }
void penDown() { movePen(PEN_DOWN_POS); }

void deplacement_relatif(float dx, float dy) {
  curX += dx;
  curY += dy;
  setGoals(curX, curY);
  waitStopped();
}

void changerLigne() {
  curX = 0;
  curY += taille_pixel_Y();
  setGoals(curX, curY);
  waitStopped();
}

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
  if (somme % v[0] == 0 && somme < (long)v[0] * v[1]) changerLigne();
}

void dessin() {
  somme = 0;
  bool noir = (v[2] == 1);
  penUp();
  for (int i = 3; i < n; i++) {
    for (int k = 0; k < v[i]; k++) {
      if (mode != MODE_MARCHE) return;   // arrêt demandé par le bouton
      traiterPixel(noir);
    }
    noir = !noir;
  }
  penUp();
}

char * lireChaine(void) {
  String st;
  static char * s = NULL;
  if (s != NULL) {
    free(s);
    s = NULL;
  }
  if (AVAIL()) {
    st = READSTRING();
    s = strdup(st.c_str());
  }
  return s;
}
