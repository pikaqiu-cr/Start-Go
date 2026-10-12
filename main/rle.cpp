#include <Arduino.h>
#include "rle.h"

// exemples 
const char * t33 = "3:3:1:3:1:1:2:1:1";
const char * invaders = "10:10:0:14:2:7:4:5:6:3:2:1:2:1:2:2:8:4:1:2:1:5:1:1:2:1:1:3:1:1:1:2:1:1:1:11";
const char * pion = "50:64:0:24:2:44:10:38:14:34:18:31:20:30:20:29:22:27:23:27:24:26:24:25:25:25:26:24:26:24:26:24:26:24:26:24:25:26:24:26:24:26:24:27:22:29:20:31:18:30:22:27:24:25:26:22:30:20:30:19:31:20:30:20:29:22:28:22:28:24:1:3:16:1:1:32:16:33:17:33:18:32:18:32:18:31:20:30:20:29:22:27:24:26:25:23:28:21:30:19:32:16:36:13:38:10:41:9:42:7:44:6:44:5:46:3:48:2:48:2:48:2:48:2:48:1:250"; 


static int stoi(const char * s,int li,int i) {
  // renvoie la valeur du nombre situé dans s entre les indices li (compris) et i (compris)
  int res = 0; 

  while (li<=i) {
    if ((s[li] != '$') && (s[li] != '#') && (s[li] != '\n')) {
      res = 10 * res + s[li] - '0' ;
    }
    li++; 
  }
  return res;
}

void printImg(T_img img) {
	// Affiche une image sur le port série SPOUT 
  int i=0; 

  for(i=0;i<img.format.largeur*img.format.hauteur;i++) {
    #ifdef BYTEMAP
      PRINT(COLPIX(img.pixels[i]));
    #else 
      // Il faut sélectionner le bon octet et le bon bit 
      // octet : i/8, bit : i%8
      PRINT(COLPIX(bitRead(img.pixels[i/8],i%8)));
    #endif 
    if (i%img.format.largeur == (img.format.largeur-1)) NL();
  }

}

T_img getImg(const char * s) {
	// Récupère l'image depuis une chaine au format RLE 
  int i=0,li=0, nbD=0, j=0, lj=0, nextVal, lastCol; // i courant, last i 
  int nbPixels; 
  #ifdef BITMAP
    int nbBytes; 
    byte nextByte;
  #endif

  static T_img img = {};
  img.format = checkRLE(s); 
  nbPixels = img.format.largeur * img.format.hauteur; 
  lastCol = img.format.couleur;
  
  if (img.format.largeur == 0) {
    PRINTLN("Chaine RLE incorrecte !");
    return img; 
  }

  // Libération de la mémoire si elle était déjà utilisée 
  if (img.pixels != NULL) free(img.pixels);

  #ifdef BYTEMAP
    img.pixels = calloc(nbPixels, sizeof(byte));
    RDLPRINTLN("allocation du tableau de pixels (bytemap)",nbPixels);
  #else 
    // il faut allouer 8 fois moins de cases
    // Exemple pour 100 bits, on a besoin de 100 /8 = 12,5 soit 13 octets 
    // On le calcule avec 100/8 + ((100%8)>0?1:0)
    nbBytes = nbPixels/8  + ((nbPixels%8)>0?1:0); 
    RDLPRINTLN("nbBytes",nbBytes);
    img.pixels = calloc(nbBytes, sizeof(byte));
    RDLPRINTLN("allocation du tableau de pixels (bitmap)",nbBytes);
  #endif

  // On parcourt la chaine pour remplir la tableau de pixels 
  while (nbD <3) {
    if (s[++i] == ':') nbD++;
  }
  li=++i; 
  // on se place sur la première spécification (juste après le 3ème ':')
  // j dénote l'indice du prochain pixel à écrire dans le tableau
  // li dénote l'indice de la prochaine valeur dans la chaine RLE 
  // i-1 dénote la fin de cette valeur dans la chaine RLE (avant le ':')

  while (s[i]!= '\0') {  
    while ( (s[++i] != ':') && (s[i]!= '\0')); // que faire si on ne trouve pas de ':' : on va déborder !!
    nextVal = stoi(s,li,i-1); 
    RDLPRINTLN("nextVal",nextVal);
    // On remplit la matrice de pixels 
    #ifdef BYTEMAP
      for(j=0;j<nextVal;j++) {
        img.pixels[lj+j] = lastCol; 
      }
    #else 
      for(j=0;j<nextVal;j++) {
        // Il faut remplir les j bits pertinents à partir du bit lj
        // On considère que le premier pixel est dans le bit de poids FAIBLE du premier octet !
        // connaisant le numéro du bit lj+j, l'octet à utiliser est (lj+j)/8
        // le bit est (lj+j)%8
        // Il faut faire des opérations d'affectation de bit avec bitSet en partant d'octets tous égaux à zéro  
        // On ne fera des bitSet que pour les couleurs noires, soit pour les couleurs ayant lastCol à BLACK
        nextByte = img.pixels[(lj+j)/8];
        if (lastCol == BLACK) {
          // lastCol dénote la couleur d'affichage courante
          // On ne met des 1 que pour les couleurs impaires
          bitSet(nextByte,(lj+j)%8); 
        }
        img.pixels[(lj+j)/8] = nextByte;
      } 
    #endif 
    lj+=j;
    lastCol = 1 - lastCol; 
    if (s[i]!= '\0') li = ++i; // on risque de sortir !! 
  }

  for(i=0;i<img.format.largeur*img.format.hauteur;i++) {
    RDLPRINT("i",i);  
    RDLPRINTLN(" col",img.pixels[i]); 
  }
  
    #ifdef BITMAP
      #ifdef RLEDEBUG
        PRINTLN("Bits encodés :"); 
        for(i=0;i<nbBytes; i++) 
          printByte(img.pixels[i]);
        NL();
      #endif
    #endif

  return img; 
}

T_format checkRLE(const char * s) {
  // vérifie la validité d'un encodage et renvoie le format de l'image. 
  // Pour cela, il faut que le produit des deux premières valeurs soit égal à la somme des suivantes. 
  int i=0,li=0; // i courant, last i 
  int largeur, hauteur, taille, nextVal, somme=0; 
  char couleur; 
  T_format f = {0}; 

  while (s[++i] != ':' ); // on avance jusqu'au prochain symbole ':'
  largeur = stoi(s,li,i-1); // on convertit la valeur 
  li = ++i; // on prépare le décodage du champ suivant 
  RDLPRINTLN("largeur",largeur);

  while (s[++i] != ':' );  
  hauteur = stoi(s,li,i-1); 
  li = ++i; 
  RDLPRINTLN("hauteur",hauteur); 

  while (s[++i] != ':' );  
  couleur = stoi(s,li,i-1); 
  li = ++i; 
  RDLPRINTLN("couleur",couleur); 

  while (s[i]!= '\0') {  
    while ( (s[++i] != ':') && (s[i]!= '\0')); // que faire si on ne trouve pas de ':' : on va déborder !!
    nextVal = stoi(s,li,i-1); 
    somme+= nextVal; 
    if (s[i]!= '\0') li = ++i; // on risque de sortir !! 
    RDLPRINTLN("nextVal",nextVal);
  }

  RDLPRINTLN("somme",somme);
  if (somme == largeur * hauteur) {
    f.largeur = largeur;
    f.hauteur = hauteur;
    f.couleur = couleur; 
  }
  
  return f; 
}

void printByte(unsigned char b){
	// Affiche les octets d'un bit dans le sens usuel en commençant par le bit de poids fort
  byte i=0; 
  int bytes[8]={0}; 
  while(b>0) {
    bytes[i++] = b%2;
    b=b/2; 
  } 
 
  // pb : i ne peut pas descendre en dessous de 0 !
  for(i=0;i<8;i++)
    PRINT(bytes[7-i]); 

}

char * lireChaine(void) {
	// Lit une chaine sur le port série 
  String st; 
  static char * s = NULL; 

   if (s != NULL) {
    free(s);
    s = NULL; 
  }

  if (AVAIL()) {
    st = READSTRING(); // en espérant que ce String soit libéré automatiquement...  
    RDLPRINTLN("st",st);
    s = strdup(st.c_str()); 
    //https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/WString.h
    // https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Stream.cpp
    //if (s[strlen(s)-1]=='\n') s[strlen(s)-1] = '\0';
  }
  
  return s;
}


/*
// Ne fonctionne pas... Permettrait d'éviter d'utiliser des String 
char * lireChaine2(void) {
  char c;
  static char * buffer = NULL; // la taille du buffer pourrait poser pb..
  int bufsize;
  int i=0;
  unsigned long start ;
  // lit une chaine sur le port série SPIN en attendant au maximum 1s 
  // https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/Stream.cpp
  // cf. timedRead et readString 

  MYPRINTLN("lireChaine");

  if (buffer != NULL) {
    free(buffer);
    buffer = NULL; 
  }

  buffer = malloc(MAXLEN * sizeof(char));
  buffer[0] = '\0';
  bufsize = MAXLEN; 
  printDN("taille du buffer",bufsize);

  start = millis(); 
  while(millis() - start < READ_TIMEOUT) {
    MYPRINTLN("attente serial");
    while (AVAIL() ) {
      c = READ();
      printD("i",i);
      printD("lu",(int)c);
      printDN(" lu",c);
      buffer[i++] = c; 
      if (i==bufsize) {
        // plus de place dans le buffer ! 
        bufsize += RESIZELEN; 
        buffer = realloc(buffer,bufsize);
        printDN("agrandissement du buffer",bufsize);
      }
    }
  }
  if (buffer[i-1]=='\n') buffer[i-1] = '\0'; 
  else buffer[i] = '\0'; 

  return buffer; // Il faut libérer ! 
}
*/
