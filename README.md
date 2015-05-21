# Projet 2A Drone Bebop Parrot
Projet 2A sur le drone Bebop et l'interface de contrôle graphique

Il y a différent programme pour tester les fonctionnalités du drone.

## Programmes :
> Les programmes sont classé par ordre chronologique.

* __Landing__ : programme qui permet au drone de se poser.

* __Take Off__ : programme qui permet le démarrage du drone. Il effectue un vole stationnaire à environ 1 mètre du sol.

* __Test Event__ : programme d'essai pour utiliser des événements claviers.

* __Pilotage Start Stop v1__ : on utilise la touche "s" et "l" pour soit Take off le drone, soit Landing le drone. On relance le réseau à chaque commande et le stop après

* __Pilotage Start Stop v2__ : même chose que la version précédante à l'exeption qu'on ne relance plus le réseau après chaque commande.

* __Pilotage Start Stop v3__ : même chose que la version précédante avec un code plus concie et plus simple.

* __Pilotage Drone Clavier__ : programme pour piloter le drone avc le clavier. 
    * 'z':avancer
    * 's':reculer
    * 'q':droite
    * 'd':gauche
    * 'a':rotation gauche
    * 'e':rotation droite
    * 'Flèche Haut':monter
    * 'Flèche Bas':desendre
    * 'k': décoller
    * 'Espace':attérir
    * 'Esc' : quitter le programme  

* __Test Manette__ : programme pour tester le bon fonctionnement de la manette. Donne les boutons sur lesquels on appuie et la valeur des axes.

* __Pilotage Drone manette Clavier__ : programme pour piloter le drone avec, au choix, soit le clavier, soit la souris. 'c' pour utiliser le clavier et 'm' pour utiliser la manette.
> Test effectué avec une manette PS3 / Xbox360
    * Joystick gauche : contrôle avancer, reculer , droite , gauche.
    * Start : décoller
    * Select / Back : quitter le programme
    * X / A: monter
    * Carré / X : descendre
    * O / B attérir
    * L1 / LB: rotation à gauche
    * R1 / RB: rotation à droite
   
* __Pilotage Drone CM RIU__ : programme pour piloter le drone avec au choix le clavier (C) et la manette (M) avec Récupération des informations utiles (RIU) (pitch,yaw,roll,altitude, vitesse, date) sans la vidéo.

* __Test Souris__ : programme pour tester le bon fonctionnement de la souris 3D Space Explorer.

## Lancer un programme:
  1. se mettre dans le dossier voulu
  2. ouvrir une console
  3. `make` (compilation)
  4. `make run` (lancer le programme)
  5. `make clean` (effacer le programme.o)
