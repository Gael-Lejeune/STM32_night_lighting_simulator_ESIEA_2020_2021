#include "stm32l0xx.h"


//le port pour les caractères sera la port GPIOA
#define PORT_CHAR GPIOA

//on utilise 4 fils pour les caractères connectés sur PA6 à PA9
//les caracrtères seront envoyés à l'afficheur par groupes de 4 bits
#define DB7 9 //PA9
#define DB6 8 //PA8
#define DB5 7 //PA7
#define DB4 6 //PA6

//Les pins RS et E seront affectées au port B
#define Port_RS  GPIOB
#define Port_E  GPIOB
#define rs 4 //PB4
#define E 5 //PB5

//Ecrit un caractère sur l'écran
void Ecrire(char f);

//Actualiser l'écran
void toggle_e();

//Ecrire un char puis actualiser l'écran
void D_set_E_Toggle (char f);

//Ecrire sur la sortie du LCD
void EcrireFonction(char f);

//Ecrire un carractère sur le LCD
void EcrireCaractere(char c);

//Ecrire une chaine de caractères sur le LCD
void EcrireChaine(char ch[]);

//Initialisation du LCD
void lcdinit4();

//Afficher sur le LCD
void Affichage_LCD();
