# Partie vision du drone : perception des obstacles

**Objectif de la vision** : à partir des caméras embarquées, détecter les obstacles, leur distance et leur forme, et décider si le drone peut passer par une ouverture (ex. un trou carré à distance d). La génération des obstacles, les collisions et le moteur physique sont hors périmètre. La cible est une coordonnée connue. « Cours N » renvoie à `Slides/CSC_51073_EPN.pdf`.

## 1. Capteur : deux caméras frontales (gauche / droite)

- Deux caméras orientées vers l'avant, séparées d'une base `B` (largeur du drone), mêmes intrinsèques `K`.
- `K` depuis `Camera.fovy` et la résolution : `f = (h/2)/tan(fovy/2)`, `(cx, cy) = (w/2, h/2)`. Pour fovy = 60° et h = 720 : f ≈ 624 px.
- Rendu hors écran moderngl de chaque caméra → images numpy. Attention aux conventions (OpenGL : −Z devant, Y en haut ; slides : +Z devant, Y en bas).
- Option recommandée pour exploiter la géométrie épipolaire : caméras légèrement convergentes, lignes épipolaires obliques, `F` estimée par 8 points + RANSAC (Lab 4) et comparée à la `F` exacte connue par la simulation.

## 2. Chaîne de traitement par frame

1. **Coins** : Harris sur les deux images (Cours 2). Les arêtes et coins d'un obstacle ou d'un trou carré sont exactement ce que Harris détecte.
2. **Appariement épipolaire** : pour chaque coin `x` de gauche, recherche le long de `l' = F·x` dans l'image droite, SSD sur patch + test du ratio (Cours 3, 4).
3. **Triangulation** : chaque paire → point 3D dans le repère du drone (Cours 4).
4. **Structure** : regrouper les points en surfaces planes par RANSAC (même principe que pour les homographies, Cours 3) : un mur = un plan.
5. **Ouvertures** : dans un plan, les coins qui délimitent une région vide forment le contour du trou. Avec ses coins 3D, on a directement sa largeur, sa hauteur et sa distance.
6. **Décision** : le drone passe si `largeur > largeur_drone + marge` et `hauteur > hauteur_drone + marge` ; la cible de navigation devient alors le centre du trou, approché perpendiculairement au plan.

## 3. Mesurer une ouverture

- **Stéréo** : profondeur `Z = f·B/d` (d = disparité en pixels). Taille réelle d'un segment vu sur `w` pixels à la distance Z : `W = w·Z/f`. Plus simplement : distance entre les deux coins 3D triangulés.
- **Précision** : une erreur d'un pixel sur la disparité donne `ΔZ ≈ Z²/(f·B)`. Avec f = 624 px, B = 0,3 m et Z = 5 m : ΔZ ≈ 0,13 m, soit environ 3 % d'erreur sur la taille du trou. L'erreur croît en Z² : la décision est plus fiable en se rapprochant, et doit être réévaluée à chaque frame.
- **Monoculaire (comparaison)** : avec une seule image, on ne peut pas séparer taille et distance (un petit trou proche ressemble à un grand trou loin). Il faut deux points de vue : stéréo, ou mouvement du drone avec pose connue.

## 3 bis. Le trou mène-t-il quelque part ?

Un trou assez grand peut être une niche (fond fermé) ou un passage. Indices, du plus fiable au plus fragile :

1. **Profondeur derrière l'ouverture (géométrie, Cours 4).** Les coins vus *à travers* le trou sont triangulés comme les autres. S'ils sont bien plus loin que le plan du mur (au moins la longueur du drone + marge), il y a de l'espace derrière ; s'ils sont juste derrière le plan, c'est une niche.
2. **Parallaxe en se déplaçant.** En bougeant latéralement, ce qui est derrière le trou se décale moins que les bords du trou (plus loin). Cela confirme la profondeur même avec peu de points.
Piège : **absence de points ≠ espace libre.** Un fond uni ou le ciel ne donnent aucun coin. L'état « inconnu » doit rester distinct de « libre » ; dans ce cas le drone se rapproche ou se décale pour lever le doute.

**Choix de l'ouverture.** Chaque ouverture candidate reçoit : passable (oui/non), profondeur libre derrière, angle entre sa direction et celle de la cible. Règle déterministe : parmi les ouvertures passables avec espace libre derrière, prendre celle la plus alignée avec la cible ; s'il n'y en a qu'une, la prendre même si elle s'écarte de la cible (exploration) ; s'il n'y en a aucune, chercher en tournant. Avec plusieurs obstacles, cela revient à une recherche de chemin (A*) sur la carte des espaces libres et inconnus.

## 3 ter. Carte 3D (approche type DJI / Skydio)

- Ces drones utilisent **plusieurs paires stéréo** (DJI : paires avant, arrière, dessous selon les modèles ; Skydio : six caméras fisheye couvrant 360°), pas une caméra unique.
- Chaîne : carte de profondeur stéréo → points 3D dans le repère monde (avec la pose du drone, estimée par odométrie visuelle-inertielle) → carte de voxels libres / occupés / inconnus (type OctoMap) → planificateur de chemin dans le libre, exploration des frontières libre/inconnu.
- Pour le projet : paire stéréo frontale, points issus des coins appariés (éventuellement complétés par une profondeur dense, séance 6), lancer de rayon caméra → point pour marquer libre les voxels traversés et occupé le voxel final, A* sur la grille.
- Une seule caméra est possible (profondeur par le mouvement, multi-vues entre frames, ou profondeur monoculaire apprise, séance 5), mais avec la contrainte de la translation, l'échelle à fixer et une profondeur moins fiable.

## 4. Étapes de réalisation

0. Rendu des deux caméras, calcul de `K` et des poses ; vérification par projection d'un point connu.
1. Scène test : un mur avec un trou carré de taille connue, texturé pour fournir des coins.
2. Harris + appariement épipolaire + triangulation ; comparaison au z-buffer.
3. RANSAC plan → détection du mur et des coins du trou → largeur, hauteur, distance ; erreur vs vraies dimensions selon la distance.
4. Règle de passage et navigation déterministe vers le centre du trou puis vers la cible.
5. Extensions : plusieurs obstacles et ouvertures, accumulation des points dans une carte monde (grille de voxels) et planification A* ; suivi des coins entre frames pour estimer le déplacement du drone (matrice essentielle, séance 7 SfM).

## 5. Point pratique

Les surfaces unies n'ont pas de coins à l'intérieur ; seules les arêtes donnent des points. C'est suffisant pour mesurer un trou (ses quatre coins), mais pour estimer correctement le plan du mur, il faut le texturer. Éviter aussi les motifs répétitifs (quadrillage régulier) qui provoquent de faux appariements.
