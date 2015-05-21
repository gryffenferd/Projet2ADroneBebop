#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <SDL/SDL.h>
#include <pthread.h>

int continuer = 1;

void *thread_1(void *arg)
{
    SDL_Surface *ecran = NULL;
    SDL_Event event; // Cette variable servira plus tard à gérer les événements

    SDL_Init(SDL_INIT_VIDEO);

   ecran = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE);
   SDL_WM_SetCaption("Gestion des événements en SDL", NULL);

	SDL_EnableKeyRepeat(10, 5);

	while(continuer)
	{
		SDL_WaitEvent(&event);	/* Récupération de l'événement dans event */
		switch(event.type)  /* Test du type d'événement */
		{
			case SDL_QUIT:  /* Si c'est un événement de type "Quitter" */
				continuer = 0;
				break;

			case SDL_KEYDOWN:
				switch (event.key.keysym.sym)
				{
					case SDLK_ESCAPE:
						continuer = 0;
						break;
					
					case SDLK_a:
						printf("J'appui sur a\n");
						break;
	
					case SDLK_SPACE:
						printf("J'appui sur space\n");
				}
				break;
		}
	}

    SDL_Quit();

    /* Pour enlever le warning */
    (void) arg;
    pthread_exit(NULL);
}

int main(void)
{	
	pthread_t thread1;

	printf("Avant la création du thread.\n");	

    if (pthread_create(&thread1, NULL, thread_1, NULL)) {
    perror("pthread_create");
    return EXIT_FAILURE;
    }
	
	while(continuer)
	{
		printf("Dans le main\n");
		sleep(1);
	}

	return EXIT_SUCCESS;

}
