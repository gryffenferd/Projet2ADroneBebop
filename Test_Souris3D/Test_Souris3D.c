/*|--------------------------------------------------------------------------

 MAGELLAN X-Window  example application               Version 4.00 15.10.97

 Copyright (C) 1984, 2002 3Dconnexion GmbH / Inc.
 An der Hartmuehle 8
 D - 82229 Seefeld

 mailto:development@3dconnexion.com

----------------------------------------------------------------------------|*/
#include <stdlib.h>
#include <stdio.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <SDL/SDL.h>
#include <pthread.h>

#include "xdrvlib.h"

int continuer=1;

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


int main(int argc, char * argv[])
{

Display *display;
Window root, window;

int screennumber,width,height;
XSizeHints *sizehints;
XWMHints *wmhints;
XClassHint *classhints;

char *WinName = "Magellan 3D Controller";
XTextProperty WindowName;

XEvent report;
MagellanFloatEvent MagellanEvent;

XComposeStatus compose;
KeySym keysym;

char MagellanBuffer[ 256 ];

float z=0; // y
float r=0; // b
float y=0; // c
float x=0; // a

/****************** Open a Window ******************************************/
 sizehints  = XAllocSizeHints();
 wmhints    = XAllocWMHints();
 classhints = XAllocClassHint();
 if ( (sizehints==NULL) || (wmhints==NULL) || (classhints==NULL) )
  {
   fprintf( stderr, "Can't allocate memory! Exit ... \n" );
   exit( -1 );
  }

 display = XOpenDisplay( NULL );
 if ( display == NULL )
  {
   fprintf( stderr, "Can't open display! Exit ... \n");
   exit( -1 );
  }

 screennumber = DefaultScreen(display);
 width  = DisplayWidth(display,screennumber);
 height = DisplayHeight(display,screennumber);
 root   = DefaultRootWindow(display);
 window = XCreateSimpleWindow( display, root, 0,0, width/5*3,height/8, 20,
			       BlackPixel(display,screennumber),
			       WhitePixel(display,screennumber) );

 XStringListToTextProperty( &WinName, 1, &WindowName );

 XSetWMProperties( display, window, &WindowName, NULL, argv,
		   argc, sizehints, wmhints, classhints );

 /************************* Create 3D Event Types ***************************/
 if ( !MagellanInit( display, window ) )
  {
   fprintf( stderr, "No driver is running. Exit ... \n" );
   exit(-1);
  }

 /************************* Main Loop ***************************************/
 XSelectInput( display, window, KeyPressMask | KeyReleaseMask );

 pthread_t thread1;

	printf("Avant la création du thread.\n");	

    if (pthread_create(&thread1, NULL, thread_1, NULL)) {
    perror("pthread_create");
    return EXIT_FAILURE;
	}

 while(continuer)
  {
	printf("Les coordonnées sont:\n- x : %lf\n- y : %lf\n- z : %lf\n- r : %lf\n",x,y,z,r);
   XNextEvent( display, &report );

   switch( report.type )
    {			
     case ClientMessage :
      switch( MagellanTranslateEvent( display, &report, &MagellanEvent, 1.0, 1.0 ) )
       {
        case MagellanInputMotionEvent :
        	MagellanRemoveMotionEvents( display );
		
			z = MagellanEvent.MagellanData[ MagellanY ];
			r = MagellanEvent.MagellanData[ MagellanB ];
			y = MagellanEvent.MagellanData[ MagellanC ];
			x = MagellanEvent.MagellanData[ MagellanA ];
 		break;
       }
      break;
     }
  }
 MagellanClose( display );
 return TRUE;
}


