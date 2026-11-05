# Start-Go

Update dessin 05 Novembre : voir fichier travail_perso > code_final_dessin
A l'execution, le code dessine le rle fournit à la ligne 31 du programme : les cases noirs sont remplacés par des carrés noir. Les variables X_Max et Y_MAX donnent la dimension de la zone de travail (en centimètres), tu peux les changer si tu veux.

Il me restera à faire de mieux tracer les pixels. Pour l'éxecuter tu as juste à le upload sur l'arduino et switcher le petit interrupteur (comme d'habitude) en appuyant sur le bouton reset avant. Tu peux mettre le rle que tu veux. Tu places le stylo au dessus de la feuille. Il dessine mais il ya  des problèmes de stabilité, éthan va les régler.











Parties à faire : 
  - Bouton activation du code à la réception de l'image
  - Réception image capteur
  - Transmission moteur rail 1 et 2
  - Mouvement du stylo
  - ... 
Deja fait:
  - le stylo peut chercher le feuille(avec le capteur)
  - la transformation de .rle
  - les deux moteurs peuvent faire des deplacements controlees par le code
  - 

Pas encore:
  - detection de la taille de feuille (pour determiner le point d'origine)
    si le feuille est parallele a l'axe c'est mieux
  - des tests pour calculer des parametres de vitesse, d'acceleration ...
  - la transformation entre le chemin et le mouvement des moteurs
  - ...
