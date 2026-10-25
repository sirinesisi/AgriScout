#!/usr/bin/env python3
"""Génère un champ réaliste (field.world) + plants_truth.csv correspondant.

Contenu : sol de terre, chemins en terre claire (les allées), buttes de culture,
cultures basses (celles qui sont évaluées), grands arbres sur les bords,
pierres entre les allées, petites bosses dans les allées (terrain rugueux).

Les plantes sont modélisées avec une tige, des feuilles inclinées et un épi. La géométrie de détection de field_scout.py devra être recalibrée ensuite.
Lancer : python3 make_field.py
"""
import os
import random

# ---------------- PARAMÈTRES ----------------
N_LANES = 2             # = N_LANES dans auto_drive.py
LANE_SPACING = 2.4      # = LANE_SPACING dans auto_drive.py
ROW_OFFSET = 0.8        # rangées à +/- 0.8 m du centre de l'allée
X_START, X_STEP, N_PER_ROW = 1.2, 0.7, 10
JITTER = 0.08           # décalage aléatoire des plantes (m)
P_SICK = 0.4            # proportion de plantes malades
TREES_PER_SIDE = 4      # grands arbres de chaque côté du champ
ROCKS = 10              # pierres entre les allées et sur les bords
BUMPS_PER_LANE = 25     # bosses dans chaque allée (0 = sol plat)
WEEDS = 0               # mauvaises herbes (petites boules vertes, visuel seulement)
SEED = 42
# --------------------------------------------

WORLD_DIR = os.path.expanduser('~/agri_ws/src/agri_robot/worlds')
INSTALL_DIR = os.path.expanduser('~/agri_ws/install/agri_robot/share/agri_robot/worlds')

HEALTHY = [
    {"leaf": "0.10 0.48 0.06", "ear": "0.52 0.40 0.25"},
    {"leaf": "0.12 0.55 0.08", "ear": "0.56 0.43 0.26"},
    {"leaf": "0.08 0.42 0.05", "ear": "0.48 0.36 0.22"},
]
SICK = [
    {"leaf": "0.92 0.68 0.04", "ear": "0.68 0.43 0.08"},
    {"leaf": "0.86 0.58 0.04", "ear": "0.62 0.36 0.06"},
    {"leaf": "0.95 0.73 0.06", "ear": "0.72 0.45 0.08"},
]

HEADER = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.5 0.7 0.9 1</background>
      <shadows>false</shadows>
    </scene>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.30 0.40 0.15 1</ambient><diffuse>0.30 0.40 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

def make_plant(name, x, y, leaf, ear):
    """Construit une touffe de blé avec deux tiges, feuilles et épis."""
    visuals = []

    def visual(vname, px, py, pz, geometry, color, roll=0.0, pitch=0.0, yaw=0.0):
        visuals.append(
            f'        <visual name="{vname}">\\n'
            f'          <pose>{px:.3f} {py:.3f} {pz:.3f} {roll:.3f} {pitch:.3f} {yaw:.3f}</pose>\\n'
            f'          <geometry>{geometry}</geometry>\\n'
            f'          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>\\n'
            f'        </visual>\\n'
        )

    # Deux tiges légèrement décalées donnent une petite touffe.
    stalks = [
        (0.000, 0.000, 0.64, 0.010),
        (0.045, -0.025, 0.53, 0.008),
    ]
    for si, (sx, sy, height, radius) in enumerate(stalks):
        stem_color = "0.30 0.38 0.08" if "0.9" not in leaf else "0.55 0.42 0.08"
        visual(f"stem_{si}", sx, sy, height / 2,
               f"<cylinder><radius>{radius}</radius><length>{height}</length></cylinder>",
               stem_color)

        # Feuilles longues et étroites réparties autour de chaque tige.
        leaf_specs = [
            (0.13, 0.30, 0.70, 0.15),
            (0.23, 0.28, -0.72, 1.90),
            (0.32, 0.24, 0.78, 3.70),
            (0.40, 0.20, -0.80, 5.20),
        ]
        for li, (z, length, pitch, yaw) in enumerate(leaf_specs):
            side = 1 if li % 2 == 0 else -1
            px = sx + side * 0.055
            py = sy + (1 if li in (0, 3) else -1) * 0.045
            visual(f"leaf_{si}_{li}", px, py, z,
                   f"<cylinder><radius>0.009</radius><length>{length}</length></cylinder>",
                   leaf, pitch=pitch, yaw=yaw)

    # Deux épis, avec des grains distincts visibles au sommet.
    for ei, (sx, sy, height) in enumerate(((0.0, 0.0, 0.66), (0.045, -0.025, 0.55))):
        visual(f"ear_axis_{ei}", sx, sy, height + 0.055,
               "<cylinder><radius>0.010</radius><length>0.13</length></cylinder>", ear)
        for gi in range(6):
            gz = height + 0.015 + gi * 0.016
            side = -1 if gi % 2 else 1
            gx = sx + side * (0.012 + (gi % 3) * 0.002)
            gy = sy + (0.006 if gi % 2 else -0.006)
            visual(f"grain_{ei}_{gi}", gx, gy, gz,
                   "<sphere><radius>0.012</radius></sphere>", ear)

    return (
        f'    <model name="{name}">\\n'
        f'      <static>true</static>\\n'
        f'      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>\\n'
        f'      <link name="link">\\n'
        + "".join(visuals) +
        f'      </link>\\n    </model>\\n'
    )


BOX = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.3f} 0 0 {yaw:.2f}</pose>
      <link name="link">
{col}        <visual name="v">
          <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.3f}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
BOX_COL = """        <collision name="c">
          <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.3f}</size></box></geometry>
        </collision>
"""

TREE = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="link">
        <collision name="trunk_col">
          <pose>0 0 1.1 0 0 0</pose>
          <geometry><cylinder><radius>0.16</radius><length>2.2</length></cylinder></geometry>
        </collision>
        <visual name="trunk">
          <pose>0 0 1.1 0 0 0</pose>
          <geometry><cylinder><radius>0.14</radius><length>2.2</length></cylinder></geometry>
          <material><ambient>0.30 0.20 0.10 1</ambient><diffuse>0.30 0.20 0.10 1</diffuse></material>
        </visual>
        <visual name="crown1">
          <pose>0 0 3.1 0 0 0</pose>
          <geometry><sphere><radius>1.0</radius></sphere></geometry>
          <material><ambient>0.05 0.35 0.08 1</ambient><diffuse>0.05 0.35 0.08 1</diffuse></material>
        </visual>
        <visual name="crown2">
          <pose>0.5 0.3 2.7 0 0 0</pose>
          <geometry><sphere><radius>0.7</radius></sphere></geometry>
          <material><ambient>0.08 0.40 0.10 1</ambient><diffuse>0.08 0.40 0.10 1</diffuse></material>
        </visual>
        <visual name="crown3">
          <pose>-0.5 -0.3 2.8 0 0 0</pose>
          <geometry><sphere><radius>0.75</radius></sphere></geometry>
          <material><ambient>0.06 0.30 0.07 1</ambient><diffuse>0.06 0.30 0.07 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

BUMP = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><sphere><radius>{r:.3f}</radius></sphere></geometry></collision>
        <visual name="v">
          <geometry><sphere><radius>{r:.3f}</radius></sphere></geometry>
          <material><ambient>0.45 0.35 0.22 1</ambient><diffuse>0.45 0.35 0.22 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

WEED = """    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <pose>0 0 0.04 0 0 0</pose>
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>0.2 0.7 0.2 1</ambient><diffuse>0.2 0.7 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def box(name, x, y, z, sx, sy, sz, color, collide=False, yaw=0.0):
    col = BOX_COL.format(sx=sx, sy=sy, sz=sz) if collide else ''
    return BOX.format(name=name, x=x, y=y, z=z, sx=sx, sy=sy, sz=sz, c=color, col=col, yaw=yaw)


def main():
    rnd = random.Random(SEED)
    parts, truth = [HEADER], []
    x_min, x_max = X_START - 0.5, X_START + (N_PER_ROW - 1) * X_STEP + 0.5
    x_mid, x_len = (x_min + x_max) / 2, x_max - x_min
    y_last = (N_LANES - 1) * LANE_SPACING

    n = 0
    for lane in range(N_LANES):
        c = lane * LANE_SPACING
        # chemin de terre (visuel) + deux buttes de culture (visuel)
        parts.append(box(f'path_{lane + 1}', x_mid + 0.3, c, 0.002, x_len + 1.6, 0.9, 0.004, '0.55 0.45 0.30'))
        for side, row_y in (('a', c - ROW_OFFSET), ('b', c + ROW_OFFSET)):
            parts.append(box(f'bed_{lane + 1}{side}', x_mid, row_y, 0.03, x_len, 0.5, 0.06, '0.25 0.17 0.10'))
            for i in range(N_PER_ROW):
                n += 1
                x = X_START + i * X_STEP + rnd.uniform(-JITTER, JITTER)
                y = row_y + rnd.uniform(-JITTER, JITTER)
                sick = rnd.random() < P_SICK
                palette = rnd.choice(SICK if sick else HEALTHY)
                parts.append(make_plant(
                    name=f'plant_{n:02d}', x=x, y=y,
                    leaf=palette["leaf"], ear=palette["ear"]))
                truth.append((f'plant_{n:02d}', round(x, 2), round(y, 2), 'malade' if sick else 'saine'))

    # grands arbres sur les deux bords du champ
    for k in range(TREES_PER_SIDE):
        for side, yy in (('s', -3.4), ('n', y_last + 3.4)):
            x = 0.5 + k * (x_max / max(1, TREES_PER_SIDE - 1)) * 0.95 + rnd.uniform(-0.3, 0.3)
            parts.append(TREE.format(name=f'tree_{side}{k + 1}', x=x, y=yy + rnd.uniform(-0.3, 0.3)))

    # pierres : entre les allées (milieu de l'écart entre rangées) et sur les bords
    for k in range(ROCKS):
        if N_LANES > 1 and k % 2 == 0:
            lane = rnd.randrange(N_LANES - 1)
            yy = lane * LANE_SPACING + ROW_OFFSET + (LANE_SPACING - 2 * ROW_OFFSET) / 2
        else:
            yy = rnd.choice([-1.8, y_last + 1.8])
        s = rnd.uniform(0.15, 0.30)
        parts.append(box(f'rock_{k + 1:02d}', rnd.uniform(1.0, x_max), yy, s / 2.5, s, s * 0.8, s * 0.8,
                         '0.45 0.45 0.47', collide=True, yaw=rnd.uniform(0, 3.14)))

    # terrain rugueux : petites bosses (2 cm de haut) dans les allées
    for lane in range(N_LANES):
        for k in range(BUMPS_PER_LANE):
            r = rnd.uniform(0.08, 0.14)
            parts.append(BUMP.format(name=f'bump_{lane + 1}_{k + 1:02d}', x=rnd.uniform(1.0, x_max),
                                     y=lane * LANE_SPACING + rnd.uniform(-0.4, 0.4), z=-(r - 0.02), r=r))

    for k in range(WEEDS):
        parts.append(WEED.format(name=f'weed_{k + 1:02d}', x=rnd.uniform(1.0, x_max),
                                 y=rnd.uniform(-1.0, y_last + 1.0)))
    parts.append('  </world>\n</sdf>\n')

    world_txt = ''.join(parts)
    truth_txt = 'id,x,y,etat\n' + ''.join(f'{t[0]},{t[1]},{t[2]},{t[3]}\n' for t in truth)
    for d in (WORLD_DIR, INSTALL_DIR):          # on écrit aussi dans install/ : pas besoin de colcon build
        if d == INSTALL_DIR and not os.path.isdir(d):
            continue
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'field.world'), 'w', encoding='utf-8') as f:
            f.write(world_txt)
        with open(os.path.join(d, 'plants_truth.csv'), 'w', encoding='utf-8', newline='') as f:
            f.write(truth_txt)
        print('ecrit dans', d)
    sick_n = sum(1 for t in truth if t[3] == 'malade')
    print(f'OK : {len(truth)} plantes ({sick_n} malades), {N_LANES} allees, '
          f'{TREES_PER_SIDE * 2} arbres, {ROCKS} pierres, {BUMPS_PER_LANE * N_LANES} bosses.')


if __name__ == '__main__':
    main()
