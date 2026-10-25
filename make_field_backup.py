"""Génère le monde Gazebo 'field.world' : un champ avec 2 rangées de plantes.
Génère aussi 'plants_truth.csv' : la vérité terrain (plante saine ou malade).
Usage : python3 make_field.py
"""
import csv
import os
import random

random.seed(42)                 # même champ à chaque exécution
ROWS_Y = [-0.8, 0.8]            # position (y) des 2 rangées
N_PER_ROW = 10                  # plantes par rangée
X0, DX = 1.2, 0.7               # premier x et espacement
P_DISEASED = 0.3                # proportion de plantes malades

HEALTHY = "0.1 0.6 0.1 1"       # vert
DISEASED = "1.0 0.85 0.0 1"       # jaune vif (feuilles jaunies)

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
          <material><ambient>0.35 0.28 0.2 1</ambient><diffuse>0.35 0.28 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

FOOTER = """  </world>
</sdf>
"""


def plant(name, x, y, diseased):
    color = DISEASED if diseased else HEALTHY
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="link">
        <collision name="stem_col">
          <pose>0 0 0.1 0 0 0</pose>
          <geometry><cylinder><radius>0.08</radius><length>0.2</length></cylinder></geometry>
        </collision>
        <visual name="stem">
          <pose>0 0 0.1 0 0 0</pose>
          <geometry><cylinder><radius>0.03</radius><length>0.2</length></cylinder></geometry>
          <material><ambient>0.4 0.25 0.1 1</ambient><diffuse>0.4 0.25 0.1 1</diffuse></material>
        </visual>
        <visual name="crown">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><sphere><radius>0.18</radius></sphere></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(here, "worlds")
    os.makedirs(out_dir, exist_ok=True)

    models, truth = [], []
    k = 0
    for y in ROWS_Y:
        for i in range(N_PER_ROW):
            k += 1
            x = X0 + i * DX
            diseased = random.random() < P_DISEASED
            name = f"plant_{k:02d}"
            models.append(plant(name, x, y, diseased))
            truth.append((name, round(x, 2), round(y, 2), "malade" if diseased else "saine"))

    with open(os.path.join(out_dir, "field.world"), "w", encoding="utf-8") as f:
        f.write(HEADER + "".join(models) + "\n" + FOOTER)
    with open(os.path.join(out_dir, "plants_truth.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["id", "x", "y", "etat"])
        w.writerows(truth)

    n_mal = sum(1 for t in truth if t[3] == "malade")
    print(f"Champ généré : {len(truth)} plantes, dont {n_mal} malades.")
    print(f"Fichiers dans : {out_dir}")


if __name__ == "__main__":
    main()
