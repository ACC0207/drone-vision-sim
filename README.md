# drone-vision-sim

Simulation de drone en Python (moderngl) pour le projet du cours CSC_51073 *Analyse d'Image et Vision par Ordinateur* (IP Paris). Le drone porte deux caméras frontales (stéréo) et doit percevoir les obstacles et les ouvertures avec des méthodes du cours (coins de Harris, géométrie épipolaire, RANSAC, triangulation) pour rejoindre une cible.

Détails des choix, de l'avancement et des prochaines étapes : [PROJET.md](PROJET.md).

## Installation

```bash
pip install -r requirements.txt
```

Les modèles 3D ne sont pas versionnés (taille et licence Sketchfab). Placer dans `Assets/models/` :
- `DJI_Avata2_Sketchfab.glb` (drone)
- `Nickel_factory.glb` (usine, optionnel)

## Lancer

```bash
python scene.py
```

## Commandes

| Touche | Action |
|---|---|
| Z/Q/S/D (W/A/S/D en QWERTY) | avancer, gauche, reculer, droite |
| Espace / Ctrl | monter / descendre |
| Shift | aller plus vite |
| Souris (après clic dans une vue) | lacet du drone, inclinaison des caméras |
| Échap | libérer la souris, puis quitter |
| Tab | caméra de suivi |

Le panneau de gauche permet de choisir les vues (caméra gauche, caméra droite, caméra fixe, suivi), de régler le vol et de changer de bâtiment.

## Fichiers

| Fichier | Rôle |
|---|---|
| `scene.py` | point d'entrée : scène, drone, caméras, boucle principale |
| `building.py` | bâtiment procédural et ses collisions |
| `collision.py` | collisions du drone (sol, sphères, boîtes) |
| `gltf_loader.py` | chargeur GLB |
| `ui_panel.py` | panneau latéral ImGui |
| `Docs/Approche_vision_drone.md` | approche vision détaillée |
