Système de défense aérien anti-drone

 Objectifs

Ce projet a pour but de détecter en temps réel la position d’un drone à partir d’un flux vidéo, puis de piloter deux moteurs via une carte Arduino afin de suivre cette position. Ce système utilise des calculs de distance basés sur les dimensions réelles de la cible et la focale de la caméra.

Ce projet a été réalisé dans le cadre d’un TIPE en CPGE

Fonctionnalités principales

- Traitement vidéo avec OpenCV pour détecter un objet cible
- Extraction des coordonnées (x, y, z) de l’objet par calcul géométrique
- Conversion des positions en angles de commande (θ et φ)
- Envoi des données à une carte Arduino via le port série
- Pilotage en position de deux moteurs à partir des données reçues

Technologies utilisées

- Python 
- OpenCV
- NumPy
- PySerial
- Arduino IDE

