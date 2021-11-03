#include "mbed.h"
#include <math.h>

#define PWMperiode 1e-3 //1ms

//serial Putty
Serial foutPC(USBTX,USBRX);
//Pin pwm vitesse du moteur
PwmOut E1(P2_2);
PwmOut E2(P2_3);
//Pin sens du moteur
DigitalOut M1(P0_5);
DigitalOut M2(P0_4);

/*
	C1 (5)		: Blanc 	| P0.23 <=> p15
	C2 (A2)		: Violet	| P0.24 <=> p16
	C3 (A0)		: Bleu		| P0.25 <=> p17
	C4 (11)		: Vert		| P0.26 <=> p18
	C5 (A3)		: Jaune		| P1.30 <=> p19
	C6 (4)		: Orange	| P1.31 <=> p20
*/
//Tableau capteurs photorésistances
DigitalIn sensors[6] = {P0_23, P0_24, P0_25, P0_26, P1_30, P1_31};
//Tableau temps de descente de chaque capteur
int temps_us[6];
//Tableau flag savoir si le temps de descente a été récupéré
bool flagTps[6] = {false,false,false,false,false,false};
//Variable choix de la direction pour robot
int direction = 0;
//Variable réglage "fort" ou non de la direction
int count_follow = 0;

//Initialisation des sortie PWM des moteurs
void initPWM(){
	E1.period(PWMperiode);
	E2.period(PWMperiode);
}

//Première partie pour un cycle de lecture des capteurs
//Chargement de la capacité des capteurs
void sensorsOut10us(){
	char i;
	//On passe les capteurs en Out
	DigitalOut sensors[6] = {P0_23, P0_24, P0_25, P0_26, P1_30, P1_31};
	//On impose la valeur 1 à chaque capteur
	for(i=0; i<6; i++)
		sensors[i] = 1;
	//On attend quelques microsecondes pour charger la capacité
	wait_us(10);
}

//Deuxème partie pour un cycle de lecture des capteurs
//Déchargement de la capacité des capteurs et mesure du temps
void sensorsIn(){
	//On réinitialise le tableau flagTps
	char i;
	for(i=0; i<6; i++)
		flagTps[i] = false;
	//Timer pour calculer le temps de décharge
	Timer time;
	//On passe les capteurs en In
	AnalogIn sensors[6] = {P0_23, P0_24, P0_25, P0_26, P1_30, P1_31}; //p15, p16, p17, p18, p19, p20
	//On démarre le timer après le passage des capteurs à IN
	time.start();
	//On calcule les temps de descente pour chaque capteur
	bool verification = false;
	while(!verification){
		
		//Gestion condition d'arrêt while
		for(i=0; i<6; i++){
			//Mise à jour condition d'arrêt while
			if(flagTps[i] == true){
				verification = true;
			}
			else{
				verification = false;
				break;
			}
		}
			
		//Récupération des temps de descente des capteurs
		for(i=0; i<6; i++){
			if(sensors[i].read() < 0.5 && flagTps[i] == false){
					temps_us[i] = time.read_us();
					flagTps[i] = true;
			}
		}
	}
	//On arrête le timer
	time.stop();
}


//Règle la direction à prendre par le robot
//négatif -> vers la gauche
//positif -> vers la droite
int set_direction(){
	int seuil = 800;
	//lorsque les 2 capteurs du centre sont sur la ligne blanche -> avancer tout droit
	if (temps_us[2] < seuil and temps_us[3] < seuil){
		direction = 0;
	}
	//lorsque le 3ème capteur est le seul sur la ligne blanche -> tourner legerement vers la gauche
	else if( (temps_us[1] > seuil and temps_us[2] < seuil and temps_us[3] > seuil) || (temps_us[1] > seuil and temps_us[2] < seuil and temps_us[3] < seuil)){
		direction = -1;
	}
	//lorsque le 2ème capteur est le seul sur la ligne blanche -> tourner moyennement vers la gauche
	else if( (temps_us[0] > seuil and temps_us[1] < seuil and temps_us[2] > seuil) || (temps_us[0] < seuil and temps_us[1] < seuil and temps_us[2] > seuil) ){
		direction = -2;
	}
	//lorsque le 1er capteur est le seul sur la ligne blanche -> tourner fortement vers la gauche
	else if(temps_us[0] < seuil and temps_us[1] > seuil){
		direction = -3;
	}
	//lorsque le 4ème capteur est le seul sur la ligne blanche -> tourner legerement vers la droite
	else if( (temps_us[2] > seuil and temps_us[3] < seuil and temps_us[4] > seuil) || (temps_us[2] > seuil and temps_us[3] < seuil and temps_us[4] < seuil) ){
		direction = 1;
	}
	//lorsque le 5ème capteur est le seul sur la ligne blanche -> tourner moyennement vers la droite
	else if( (temps_us[3] > seuil and temps_us[4] < seuil and temps_us[5] > seuil) || (temps_us[3] > seuil and temps_us[4] < seuil and temps_us[5] < seuil) ){
		direction = 2;
	}
	//lorsque le 6ème capteur est le seul sur la ligne blanche -> tourner fortement vers la droite
	else if(temps_us[4] > seuil and temps_us[5] < seuil){
		direction = 3;
	}
	//lorsque tous les capteurs sont sur du noir -> suivre la derniere direction
	
	//on renvoie la direction choisie
	return direction;
}

//Règle la puissance des moteurs en fonction de la direction à prendre
void follow_line(int dir){
	//Vitesse à appliquer à chaque moteur
	float vitesse_droite;
	float vitesse_gauche;

	//La première partie, "forte" correction de la trajectoire (80%)
	//Ensuite, correction plus atténuée (20%)
	//Évite une forte "oscillation" autour de la ligne
	
	//Premier réglage de la direction 
	if(count_follow  < 40){
		if (dir == 0){
			vitesse_droite = 0.6;
			vitesse_gauche = 0.6;
		}
		else if (dir == -1){
			vitesse_droite = 0.7;
			vitesse_gauche = 0.4;
		}
		else if (dir == -2){
			vitesse_droite = 0.5;
			vitesse_gauche = 0.1;
		}
		else if (dir == -3){
			vitesse_droite = 0.4;
			vitesse_gauche = 0;
		}
		else if (dir == 1){
			vitesse_droite = 0.4;
			vitesse_gauche = 0.7;
		}
		else if (dir == 2){
			vitesse_droite = 0.1;
			vitesse_gauche = 0.5;
		}
		else if (dir == 3){
			vitesse_droite = 0;
			vitesse_gauche = 0.4;
		}
		count_follow++;
	}
	
	//Réglage suivant plus faible
	else if(count_follow > 40 && count_follow < 50){
		switch (dir){
			case 0:
				vitesse_droite = 0.6;
				vitesse_gauche = 0.6;
				break;
			case -1:
				vitesse_droite = 0.7;
				vitesse_gauche = 0.6;
				break;
			case -2:
				vitesse_droite = 0.6;
				vitesse_gauche = 0.2;
				break;
			case -3:
				vitesse_droite = 0.5;
				vitesse_gauche = 0.2;
				break;
			case 1:
				vitesse_droite = 0.6;
				vitesse_gauche = 0.7;
				break;
			case 2:
				vitesse_droite = 0.2;
				vitesse_gauche = 0.6;
				break;
			case 3:
				vitesse_droite = 0.2;
				vitesse_gauche = 0.5;
				break;
			default:
				break;
		}
		count_follow++;
	}
	else
		count_follow = 0;
	
	//mise a jour des caracteristiques des moteurs
	E1.pulsewidth(PWMperiode*vitesse_droite);
	E2.pulsewidth(PWMperiode*vitesse_gauche);
}

void print_temps(){
	//affichage du temps des capteurs
	char i;
	for(i=0;i<6;i++){
		foutPC.printf("Temps du capteurs n°%d : %d\n\r",i+1,temps_us[i]);
	}
	foutPC.printf("\n\n\n");
}

int main(){
	initPWM();

	M1 = 0;
	M2 = 0;
	
	while(1){			
		//On charge la capacité de chaque capteur
		sensorsOut10us();
		//On mesure le temps de décharge
		sensorsIn();

		//Permet de voir la valeur renvoyée par les capteurs
		//print_temps();
		//wait(1.5);
		
		//Permet de suivre la ligne
		follow_line(set_direction());
		//E1.pulsewidth(PWMperiode*0.5);
		//E2.pulsewidth(PWMperiode*0.5);		
	}
}

