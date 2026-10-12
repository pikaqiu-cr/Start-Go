#define BITMAP // matrice de bits, 8 pixels par octet 
//#define BYTEMAP // matrices d'octets, 1 pixel par octet  

/////  NE PAS EDITER SOUS CETTE LIGNE ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//format RLE utilisé pour le SNGO Robots: 
// <taille_pixels_largeur>:<taille_pixels_hauteur>:<encodage RLE>
// Encodage RLE standard fourni sous la forme <N><C><N><C><N><C><N><C> avec N un entier (possiblement sur plusieurs chiffres), et <C> une couleur B(lanc) ou N(noir)
// Le codage est réalisé de haut en bas et de gauche à droite, dans le sens de lecture classique occidental

// plus compact : suite de <N>:<N> où chaque nombre représente la quantité d'éléments de la même couleur qui se suivent 
// <taille_pixels_largeur>:<taille_pixels_hauteur>:<encodage RLE>
// La première couleur pourrait être ommise si on considère que seul le motif est important, 
// qu'il soit affiché en noir sur fond blanc ou en inversion de couleur
// on décide de fournir tout de même la première couleur: 0 WHITE ou 1 BLACK

/*
Cf. https://mrdorancomputing.com/data-representation/rle/

10 1 
4 1 2 0 4 1 
3 1 4 0 3 1 
2 1 6 0 2 1 
1 1 2 0 1 1 2 0 1 1 2 0 1 1 
1 1 8 0 1 1 
3 1 1 0 2 1 1 0 3 1 
2 1 1 0 1  1 2 0 1 1 1 0 2 1 
1 1 1 0 1 1 1 0 2 1 1 0 1 1 1 0 1 1 
10 1 

sans découper les lignes 
14 1 2 0 7 1 4 0 5 1 6 0 3 1 2 0 1 1 2 0 1 1 2 0 2 1 8 0 4 1 1 0 2 1 1 0 5 1 1 0 1 1 2 0 1 1 1 0 3 1 1 0 1 1 1 0 2 1 1 0 1 1 1 0 11 1

sans fournir les couleurs : 
14:2:7:4:5:6:3:2:1:2:1:2:2:8:4:1:2:1:5:1:1:2:1:1:3:1:1:1:2:1:1:1:11
*/

#include "print.h" // macros PRINT

#ifndef _RLE_H
#define _RLE_H

// exemples 
extern const char * t33; 
extern const char * invaders; 
extern const char * pion;

//#define BITMAP // matrice de bits, 1 bit par pixel 
// Si cette constante est définie, on utilisera un tableau de bits plutot qu'un tableau d'octets

#ifndef BITMAP
  #define BYTEMAP // matrices d'octets, 1 pixel par octet  
#endif 


#define BLACK 1 
#define WHITE 0
#define COLPIX(i) ((i==WHITE)?'.':'X')

typedef struct {
  int largeur; 
  int hauteur; 
  int couleur;
} T_format; 

typedef struct {
  T_format format; // geometrie de l'image 
  byte * pixels; // fournit le tableau des pixels de l'image
} T_img; 
 
void printImg(T_img img); 	// Affiche une image sur le port série SPOUT 
T_img getImg(const char * s); 	// Récupère l'image depuis une chaine au format RLE 
T_format checkRLE(const char * s);   // vérifie la validité d'un encodage et renvoie le format de l'image. Pour cela, il faut que le produit des deux premières valeurs soit égal à la somme des suivantes. 
void printByte(unsigned char b); 	// Affiche les octets d'un bit dans le sens usuel en commençant par le bit de poids fort
char * lireChaine(void); // Lit une chaine sur le port série 

#endif 
