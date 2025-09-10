# Projet de simulation robotique

Ce projet est inclus dans un environnement de simulation de déplacement d'un robot dans un labyrinthe. Il comprend 4 challenges : 
- Le robot doit faire le plus de tours possibles dans un labyrinthe en un temps donné. Il doit éviter les obstacles à l'aide de capteurs de distance. Il n'a pas accès à la carte du labyrinthe, ni au GPS.
- Le robot doit explorer l'entièreté d'un labyrinthe en un temps donné. Il a accès aux capteurs de distances et à un GPS.
- Le robot doit explorer un labyrinthe pour en trouver tous les checkpoints, et déterminer le chemin le plus court pour tous les rejoindre. Il a accès aux capteurs de distances et à un GPS.
- Le robot doit à nouveau explorer l'entièreté d'un labyrinthe, mais sans GPS. Il doit se repérer à l'aide des équations de modèle de mouvement.  

Les capteurs et les moteurs ont du bruit dans leurs mesures. Le robot doit donc être capable de gérer l'incertitude dans ses déplacements et dans la perception de son environnement.  
Le code source du robot a été écrit en Python. Il se trouve dans le dossier `agent`. Pour lancer la simulation, exécuter le script `startCX` (où X est le numéro du challenge), puis le script `challengeX.py` dans le dossier `agent`.


# CiberRato Robot Simulation Environment <br/> Universidade de Aveiro / IEETA 

## Information

CiberRato Robot Simulation Environment simulates the movement
of robots inside a labyrinth.  Robots objective is to go from their
starting position to beacon area and then return to their start position.

The MicroRato competition
[http://microrato.ua.pt/], held annually at Aveiro University, 
uses these these tools for its Explorer league.

## Contents

* simulator -           The simulator source code
* Viewer -              The Visualizer source code
* logplayer -           The logplayer source code
* GUISample -           Graphical robot agent (C++) source code
* robsample -           robot agent (C) source code
* jClient -             robot agent (Java) source code
* pClient -             robot agent (Python) source code
* Labs -                examples of labyrinths used in previous competitions
* startAll -            script that runs the simulator, the visualizer and 5 GUISamples
* startSimViewer -      script that runs the simulator and the Viewer

## Install

The source code was compiled with gcc/g++ - Gnu Project C/C++ Compiler
(gcc version  9.3.0) using the Qt libraries (release 5.12.8) on Ubuntu 20.04.

It is required to have the development version of gcc/g++, cmake, Qt libraries
release 5.x installed in the system prior to compilation.
On Ubuntu 20.04 run the following:
```bash
sudo apt-get install build-essential cmake qtmultimedia5-dev
```

Then in the repository base dir, execute:
```bash
mkdir build
cd build
cmake ..
make
```

To run the simulator, Viewer and C++ agent, execute (at the repository base dir):
```bash
./startAll
```


## Authors

* Nuno Lau,
  University of Aveiro,
  nunolau@ua.pt

* Artur C. Pereira,
  University of Aveiro,
  artur@ua.pt

* Andreia Melo,
  University of Aveiro,
  abmelo@criticalsoftware.com

* Antonio Neves,
  University of Aveiro,
  an@ua.pt

* Joao Figueiredo,
  University of Aveiro
  joao.figueiredo@ieeta.pt

* Miguel Rodrigues,
  University of Aveiro,
  miguel.rodrigues@ua.pt

* Eurico Pedrosa,
  University of Aveiro,
  efp@ua.pt

 Copyright (C) 2001-2024 Universidade de Aveiro
