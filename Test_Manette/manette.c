#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <SDL/SDL.h>

int main()
{
	int i;
	int continuer = 1;	
	SDL_Event event;

	if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
		return EXIT_FAILURE;
	
	SDL_Joystick *joystick;
	joystick = SDL_JoystickOpen(0);

	SDL_JoystickEventState(SDL_ENABLE);
	
	printf("Il y a %d joystick.\n\n",SDL_NumJoysticks());	
	
	for(i = 0; i < SDL_NumJoysticks();i++)
	{
		printf("Nom du joystick numero %d : %s\n",i+1,SDL_JoystickName(i));
	}

	printf("- nombre de boutons : %d\n",SDL_JoystickNumButtons(joystick));
	printf("- nombre d'axes : %d\n",SDL_JoystickNumAxes(joystick));
	printf("- nombre chapeaux : %d\n",SDL_JoystickNumHats(joystick));

	while(continuer)
	{	
		SDL_WaitEvent(&event);	
		switch(event.type)
		{

			case SDL_JOYBUTTONUP:
				switch(event.jbutton.button)
				{
					case 0:
						printf("BoutonUp 0\n");						
						continuer = 0;
						break;
					case 1:
						printf("BoutonUp 1\n");
						break;
					case 2:
						printf("BoutonUp 2\n");
						break;
					case 3:
						printf("BoutonUp 3\n");
						break;
					case 4:
						printf("BoutonUp 4\n");
						break;
					case 5:
						printf("BoutonUp 5\n");
						break;
					case 6:
						printf("BoutonUp 6\n");
						break;
					case 7:
						printf("BoutonUp 7\n");
						break;
					case 8:
						printf("BoutonUp 8\n");
						break;
					case 9:
						printf("BoutonUp 9\n");
						break;
					case 10:
						printf("BoutonUp 10\n");
						break; 	
				}
				break;

			case SDL_JOYBUTTONDOWN:
				switch(event.jbutton.button)
				{
					case 0:
						printf("Bouton 0\n");
						break;
					case 1:
						printf("Bouton 1\n");
						break;
					case 2:
						printf("Bouton 2\n");
						break;
					case 3:
						printf("Bouton 3\n");
						break;
					case 4:
						printf("Bouton 4\n");
						break;
					case 5:
						printf("Bouton 5\n");
						break;
					case 6:
						printf("Bouton 6\n");
						break;
					case 7:
						printf("Bouton 7\n");
						continuer = 0;
						break;
					case 8:
						printf("Bouton 8\n");
						break;
					case 9:
						printf("Bouton 9\n");
						break;
					case 10:
						printf("Bouton 10\n");
						break; 
				}
			break;

			/* Tests pour savoir quel axe est utilisé 
			 * le test de > 3000 et -3000 permet d'avoir une zone morte*/															
			
			case SDL_JOYAXISMOTION:
				if(event.jaxis.axis == 0 && event.jaxis.value < -3000)			// si joystick gauche à gauche
					printf("L'axe 0 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 0 && event.jaxis.value > 3000)			// si joystick gauche à doite
					printf("L'axe 0 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 1 && event.jaxis.value < -3000)			// si joytsick gauche à en haut
					printf("L'axe 1 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 1 && event.jaxis.value > 3000)			// si joytsick gauche à en bas
					printf("L'axe 1 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 2 && event.jaxis.value < -3000)			// si joystick droite à gauche
					printf("L'axe 2 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 2 && event.jaxis.value > 3000)			// si joystick droite à doite
					printf("L'axe 2 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 3 && event.jaxis.value < -3000)			// si joytsick droite à en haut
					printf("L'axe 3 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 3 && event.jaxis.value > 3000)			// si joytsick droite à en bas
					printf("L'axe 3 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 4 && event.jaxis.value < -3000)			// si joytsick droite à en haut
					printf("L'axe 4 est incliné de %d\n",event.jaxis.value);

				if(event.jaxis.axis == 5 && event.jaxis.value > 3000)			// si joytsick droite à en bas
					printf("L'axe 5 est incliné de %d\n",event.jaxis.value);

				break;

			default:
				printf("Attente d'un appui\n");
		}
	}

	SDL_JoystickClose(joystick);
	SDL_Quit();
	return EXIT_SUCCESS;
}
