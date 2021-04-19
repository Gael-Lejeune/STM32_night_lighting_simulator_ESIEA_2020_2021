#include "lcd.h"

void Ecrire(char f){

//mettre les sorties à 0
PORT_CHAR->ODR&=~(1<<DB4);
PORT_CHAR->ODR&=~(1<<DB5);
PORT_CHAR->ODR&=~(1<<DB6);
PORT_CHAR->ODR&=~(1<<DB7);

//si le bit 7 de f est à 1 mettre le bit DB7 à 1
if((f & 0b10000000)!=0) PORT_CHAR->ODR|=(1<<DB7);

//si le bit 6 de f est à 1 mettre le bit DB6 à 1
if((f & 0b01000000)!=0) PORT_CHAR->ODR|=(1<<DB6);

//si le bit 5 de f est à 1 mettre le bit DB5 à 1
if((f & 0b00100000)!=0) PORT_CHAR->ODR|=(1<<DB5);

//si le bit 7 de f est à 1 mettre le bit DB4 à 1
if((f & 0b00010000)!=0) PORT_CHAR->ODR|=(1<<DB4);
}

void toggle_e(){
	//mettre la pin E du LCD à 1
	Port_E->ODR|=(1<<E);

	//attendre un peu
	  LL_mDelay(50);//delay=50us

	//mettre la pin E du LCD à 0
	Port_E->ODR&=~(1<<E);	//mettre E à 0
}

void D_set_E_Toggle (char f){

Ecrire(f);

toggle_e();

}

void EcrireFonction(char f){
	char nibble_pfort;
	char nibble_pfaible;
	//mettre la pin RS du LCD à 0
	Port_RS->ODR&=~(1<<rs);

	//écrire le nibble de poids fort de f
	D_set_E_Toggle(f);

	//inverser le nibble de poids fort et le nibble de poids faible de f
	nibble_pfort=(f&0b11110000)>>4;
	nibble_pfaible=f&0b00001111;
	int swap=nibble_pfort|(nibble_pfaible<<4);

	//écrire le nibble
	D_set_E_Toggle(swap);

	//attendre un peu
	LL_mDelay(50);//delay=50us

}

void EcrireCaractere(char c){
	//mettre la pin RS du LCD à 1
	 Port_RS->ODR|=(1<<rs);

    //écrire le nibble de poids fort de c
	 D_set_E_Toggle(c); //call D_set_E_Toggle

    //inverser le nibble de poids fort et le nibble de poids faible de c
	char nibble_pfort=(c&0b11110000)>>4;
	char nibble_pfaible=c&0b00001111;
	int swap=nibble_pfort|(nibble_pfaible<<4);

	//écrire le nibble
	 D_set_E_Toggle(swap);

	 //attendre un peu
	  LL_mDelay(50); //delay=50us
}

void EcrireChaine(char ch[]){
int  i=0;
while(ch[i]!='\0') {EcrireCaractere (ch[i]); i++;}

}

void lcdinit4(){

	  LL_mDelay(1000000);//delay=1000ms

	//Activer les sorties du microcontrôleur
	PORT_CHAR->MODER&=~(0b11<<(2*DB4));
	PORT_CHAR->MODER|=(0b01<<(2*DB4));
	PORT_CHAR->MODER&=~(0b11<<(2*DB5));
	PORT_CHAR->MODER|=(0b01<<(2*DB5));
	PORT_CHAR->MODER&=~(0b11<<(2*DB6));
	PORT_CHAR->MODER|=(0b01<<(2*DB6));
	PORT_CHAR->MODER&=~(0b11<<(2*DB7));
	PORT_CHAR->MODER|=(0b01<<(2*DB7));


	Port_E->MODER&=~(0b11<<(2*E));
	Port_E->MODER|=(0b01<<(2*E));

	Port_RS->MODER&=~(0b11<<(2*rs));
	Port_RS->MODER|=(0b01<<(2*rs));

	  LL_mDelay(20000);//delay=20ms

     Ecrire(0x30);
     toggle_e();
     LL_mDelay(5000);//delay=5ms

	Ecrire(0x30);
     toggle_e();
     LL_mDelay(200);//delay=200us

     Ecrire(0x30);
     toggle_e();
     LL_mDelay(200);//delay=200us

	Ecrire(0x20);
     toggle_e();
     LL_mDelay(200);//delay=200us



//SET FUNCTION: choix du mode de fonctionnement
EcrireFonction(0x28);

//DISPLAY OFF
EcrireFonction(0x8);

//DISPLAY CLEAR
EcrireFonction(1);
LL_mDelay(1600);//delay=1.6ms


//ENTRY MODE SET
EcrireFonction(0x6);

//DISPLAY ON

EcrireFonction(0xC);


}

void Affichage_LCD(char * ligne1, char *ligne2){
	/*effacer l'affichage*/
	// LCD <- 01, display clear
	EcrireFonction(0x80);
	EcrireFonction(1);
	LL_mDelay(1600);//delay=1.6ms

	/**ecriture sur la premiere ligne**/
	/*positionner le curseur avec la fonction SET DD RAM ADDRESS */
	EcrireFonction(0x80);
	EcrireChaine(ligne1);

	/**ecriture sur la deuxieme ligne**/
	/*positionner le curseur avec la fonction SET DD RAM ADDRESS*/
    EcrireFonction(0xC0);
	EcrireChaine(ligne2);
}



