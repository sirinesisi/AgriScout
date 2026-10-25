#!/usr/bin/env python3
"""Compare la carte détectée avec la vérité terrain (plants_truth.csv).
Usage :
  ros2 topic echo /plant_map --once > ~/agri_ws/map.yaml
  python3 evaluate.py ~/agri_ws/map.yaml
"""
import csv
import math
import os
import sys

import yaml

TRUTH = os.path.expanduser('~/agri_ws/src/agri_robot/worlds/plants_truth.csv')
MATCH_DIST = 0.4     # une détection est associée à une vraie plante si < 0.4 m


def load_truth(path):
    with open(path, newline='', encoding='utf-8') as f:
        return [(r['id'], float(r['x']), float(r['y']), r['etat']) for r in csv.DictReader(f)]


def load_detections(path):
    if path.endswith('.csv'):
        with open(path, newline='', encoding='utf-8') as f:
            return [(float(r['x']), float(r['y']), r['etat']) for r in csv.DictReader(f)]
    with open(path, encoding='utf-8') as f:                  # sortie de 'ros2 topic echo'
        docs = [d for d in yaml.safe_load_all(f) if d]
    out = []
    for m in docs[0]['markers']:
        p = m['pose']['position']
        out.append((float(p['x']), float(p['y']), 'malade' if m['color']['r'] > 0.5 else 'saine'))
    return out


def main():
    det_path = os.path.expanduser(sys.argv[1]) if len(sys.argv) > 1 else os.path.expanduser('~/agri_ws/map.yaml')
    truth, det = load_truth(TRUTH), load_detections(det_path)

    # Association gloutonne : les paires les plus proches d'abord
    pairs = sorted((math.hypot(t[1] - d[0], t[2] - d[1]), i, j)
                   for i, t in enumerate(truth) for j, d in enumerate(det))
    used_t, used_d, match = set(), set(), []
    for dist, i, j in pairs:
        if dist < MATCH_DIST and i not in used_t and j not in used_d:
            used_t.add(i); used_d.add(j); match.append((truth[i], det[j]))

    tp = sum(1 for t, d in match if t[3] == 'malade' and d[2] == 'malade')
    fn = sum(1 for t, d in match if t[3] == 'malade' and d[2] == 'saine')
    fp = sum(1 for t, d in match if t[3] == 'saine' and d[2] == 'malade')
    tn = sum(1 for t, d in match if t[3] == 'saine' and d[2] == 'saine')
    n = len(match)

    print(f'Plantes réelles : {len(truth)} ({sum(1 for t in truth if t[3] == "malade")} malades)')
    print(f'Détections      : {len(det)}  | associées à une vraie plante : {n}')
    print(f'Couverture      : {n}/{len(truth)} = {100 * n / len(truth):.0f} %')
    if n:
        print(f'Exactitude (saine/malade, sur les plantes vues) : {100 * (tp + tn) / n:.0f} %')
    if tp + fp:
        print(f'Précision "malade" : {100 * tp / (tp + fp):.0f} %   (alertes justes)')
    if tp + fn:
        print(f'Rappel "malade"    : {100 * tp / (tp + fn):.0f} %   (malades retrouvées)')
    print(f'Matrice : malades trouvées={tp}, malades ratées (vues comme saines)={fn}, '
          f'fausses alertes={fp}, saines OK={tn}')
    print(f'Détections sans vraie plante proche : {len(det) - n}')
    missed = [t for i, t in enumerate(truth) if i not in used_t]
    print('Plantes jamais vues :', ', '.join(f'{t[0]}({t[3]})' for t in missed) or 'aucune')


if __name__ == '__main__':
    main()
