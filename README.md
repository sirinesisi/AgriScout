# AgriScout

Robot mobile **autonome** qui parcourt un champ de blé **simulé**, repère les plantes saines et malades, les place sur une carte (RViz) et produit un rapport chiffré comparé à la vérité terrain.

**Stack :** ROS 2 Jazzy · Gazebo Harmonic · RViz2 · OpenCV · Python · TurtleBot3 Waffle Pi

> Projet personnel, 100 % logiciel (aucun matériel). Auteur : *[ton nom, ta formation]*

## Démo

<!-- Ajoute tes captures dans docs/ avec ces noms, puis supprime ce commentaire -->
![Fenêtre AgriScout](docs/agriscout_window.png)
![Carte dans RViz](docs/rviz_map.png)

Vidéo : *[lien]*

## Comment ça marche

```
Caméra (/camera/image_raw)
   -> color_detector.py : segmentation HSV (vert = saine, jaune = malade)
   -> field_scout.py    : position de chaque plante (modèle sténopé + odométrie), fusion des observations
   -> /plant_map (RViz) + rapport_agriscout.csv/.txt (comparaison à la vérité terrain)

auto_drive.py : pilote automatique (suivi d'allée, demi-tour, changement d'allée)
```

- **Localisation d'une plante** : la profondeur vient de la hauteur de la boîte dans l'image (taille réelle connue), le cap vient de sa position horizontale, puis on passe dans le repère `odom` avec la pose du robot. Les détections à moins de 0,35 m l'une de l'autre sont fusionnées (moyenne glissante, vote majoritaire saine/malade).
- **Boîtes ignorées** : coupées par le bord de l'image, de forme anormale (plantes collées) ou hors de la plage de distance fiable. Elles apparaissent en gris dans la fenêtre.
- **Pilote automatique** : vitesse constante réduite en fin d'allée, et loi de commande sur le cap et l'écart latéral

  `w = -K_cap · cap − K_lat · (y − y_allée)`

  Machine à états : `LANE` (suivi de l'allée) → `TURN_A` → `SHIFT` (décalage vers l'allée suivante) → `TURN_B`. Arrêt si un obstacle est détecté devant (lidar). À la fin, il publie `/mission/status = done` et AgriScout génère le rapport tout seul.

## Installation

Testé sur Ubuntu 24.04, ROS 2 Jazzy, Gazebo 8.11 (Harmonic), Python 3.12.

```bash
sudo apt install ros-jazzy-turtlebot3-gazebo ros-jazzy-cv-bridge python3-opencv python3-yaml
mkdir -p ~/agri_ws/src && cd ~/agri_ws/src
git clone https://github.com/<TON_UTILISATEUR>/agri-scout.git agri_robot
cd ~/agri_ws && colcon build --packages-select agri_robot
```

**Réglage de la caméra du TurtleBot3 (obligatoire).** Le modèle `waffle_pi` doit avoir une caméra à 10 images/s et un champ de vision de 100° (`1.75` rad, valeur utilisée par `FOV` dans `field_scout.py`) :

```bash
F=/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf
sudo cp $F $F.backup
sudo sed -i 's|<update_rate>30</update_rate>|<update_rate>10</update_rate>|; s|<horizontal_fov>1.085595</horizontal_fov>|<horizontal_fov>1.75</horizontal_fov>|' $F
```

## Lancer

Générer le champ (une fois, ou pour changer ses paramètres) :

```bash
python3 tools/make_field_wheat.py
```

Puis, dans 3 terminaux (dans cet ordre) :

```bash
# Terminal 1 : simulation
source /opt/ros/jazzy/setup.bash && source ~/agri_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch agri_robot field_world.launch.py gui:=false

# Terminal 2 : détection, carte et rapport
source /opt/ros/jazzy/setup.bash
cd ~/agri_ws/src/agri_robot/agri_robot && python3 field_scout.py

# Terminal 3 : pilote automatique
source /opt/ros/jazzy/setup.bash
cd ~/agri_ws/src/agri_robot/agri_robot && python3 auto_drive.py
```

Le robot parcourt les allées seul. À la fin, le rapport s'affiche et est écrit dans `~/agri_ws/rapport_agriscout.txt` et `.csv`. Pour voir la carte : `rviz2`, Fixed Frame = `odom`, puis *Add → By topic → /plant_map → MarkerArray*.

`N_LANES` et `LANE_SPACING` doivent avoir **les mêmes valeurs** dans `tools/make_field_wheat.py` et `agri_robot/auto_drive.py`.

## Résultats

Le rapport compare chaque plante détectée à `worlds/plants_truth.csv` (une détection est associée à une vraie plante si elle est à moins de 0,4 m).

| Champ | Plantes | Couverture | Exactitude saine/malade (plantes vues) | Précision « malade » | Malades retrouvées | Fausses détections |
|---|---|---|---|---|---|---|
| Plantes sphériques, conduite manuelle lente | 20 (10 malades) | 20/20 (100 %) | 100 % | 100 % | 10/10 | 0 |
| **Blé**, pilote automatique | 40 (18 malades) | 18/40 (45 %) | 61 % | 46 % | 6/18 (33 %) | 23 |

Détail du champ de blé (rapport du 2026-10-25) : 41 détections pour 40 plantes, dont 18 associées à une vraie plante. Les 7 erreurs de classification sont toutes des plantes saines classées « malade ». Le « rappel 100 % » affiché par le rapport ne porte que sur les 6 malades effectivement vues ; sur les 18 malades du champ, 6 seulement sont retrouvées.

### Analyse

Sur le champ simple, la chaîne caméra → détection → position → carte fonctionne parfaitement. Sur le champ de blé, elle se dégrade nettement, alors que le pilote automatique, lui, parcourt bien les allées. Causes probables (**non vérifiées**) :

- **Profondeur mal estimée.** La distance d'une plante est déduite de la hauteur de sa boîte dans l'image, en supposant une taille connue (`CROWN_D`). Pour du blé fin et haut, la zone colorée ne couvre qu'une partie de la plante : la distance est surestimée et les plantes sont placées trop loin. Plusieurs fausses détections, vues 100 à 200 fois, sont à y ≈ -1,8 alors que la rangée est à y = -0,8.
- **Vue de biais.** Depuis le milieu de l'allée, les rangées sont vues presque de côté, donc les plantes d'une même rangée se chevauchent dans l'image et sont ignorées (boîtes grises).
- **Association à la vérité terrain.** Avec des plantes espacées de 0,7 m et une tolérance de 0,4 m, une détection mal placée peut être associée à la mauvaise plante, ce qui gonfle les erreurs de classification.

## Limites

- **Simulation uniquement** ; la classification repose sur la couleur (HSV), pas sur un modèle d'apprentissage. Elle marche parce que le simulateur utilise des couleurs franches et maîtrisées.
- La profondeur suppose une taille de plante connue (`CROWN_D`), ce qui échoue sur le blé (voir Analyse), et l'odométrie n'est pas corrigée de la dérive.
- Pas de contournement d'obstacles : le robot s'arrête devant un obstacle au lieu de l'éviter (les pierres et les bosses peuvent le bloquer).
- La caméra du modèle TurtleBot3 a été modifiée (champ de vision élargi) pour voir les plantes sur les côtés des allées.

## Pistes d'amélioration

- Localiser les plantes en supposant les rangées connues (seul le cap compte, plus la taille), ou balayer les côtés de l'allée (caméra latérale ou arrêts avec rotation).
- Remplacer la détection par couleur par un modèle **YOLO** entraîné sur des images de feuilles (PlantVillage).
- Contournement d'obstacles et navigation avec Nav2.
- Textures réalistes de feuilles dans Gazebo, éclairage variable, plusieurs robots.

## Structure

```
agri_robot/
├── agri_robot/
│   ├── color_detector.py   détection par couleur
│   ├── field_scout.py      localisation, carte RViz, rapport
│   └── auto_drive.py       pilote automatique
├── launch/field_world.launch.py
├── worlds/                 champ généré (field.world) et vérité terrain (plants_truth.csv)
├── tools/                  générateur de champ, évaluation
├── docs/                   captures d'écran
└── setup.py, package.xml
```
