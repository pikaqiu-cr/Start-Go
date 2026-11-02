#include <TimerThree.h>


//variable diode qui peut être supprimée (si pas d'impact la suppression de pinMode(DIODE,OUTPUT) ?)

#define BOUTON 2
#define DIODE 13
#define RELACHE HIGH
#define APPUYE LOW
#define MODE_ETEINT 0
#define MODE_CLIGNOTEMENT 1

volatile boolean mode = MODE_ETEINT;
volatile boolean state_prec = RELACHE;


void traitement (void);
void fctPeriodique (void);
void changerMode (void);


void setup() {
  // initialize the LED pin as an output:
  pinMode(DIODE, OUTPUT);
  pinMode(BOUTON,INPUT_PULLUP);
  Serial.begin(9600);
  Timer3.initialize(250000); //tous les 250 ms
  Timer3.attachInterrupt(fctPeriodique); // blinkLED to run every 0.15 seconds
}
void loop() {
 
  if (mode == MODE_CLIGNOTEMENT){
    traitement();
  }
}




void traitement (void) {
}


void changerMode (void){
 tone (8,440,100);
  mode = !mode;
}


void fctPeriodique () {
  boolean buttonState = digitalRead(BOUTON);
  if ((state_prec != buttonState) && (buttonState == APPUYE)){
    changerMode ();
  }
  state_prec = buttonState;
}
