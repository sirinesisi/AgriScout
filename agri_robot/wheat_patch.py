#!/usr/bin/env python3
"""Adapte color_detector.py et field_scout.py au ble (plantes hautes et fines).
Lancer dans le dossier qui contient ces deux fichiers :  python3 wheat_patch.py
Sauvegardes : color_detector.py.bak et field_scout.py.bak
"""
import re
import shutil

def patch(path, edits):
    s = open(path, encoding='utf-8').read()
    shutil.copy(path, path + '.bak')
    for old, new, is_regex in edits:
        if new in s:
            continue                      # deja applique
        if is_regex:
            s, n = re.subn(old, new, s, count=1, flags=re.M)
            assert n == 1, f'{path}: motif introuvable : {old}'
        else:
            assert old in s or new in s, f'{path}: texte introuvable : {old}'
            s = s.replace(old, new, 1)
    open(path, 'w', encoding='utf-8').write(s)
    print('OK', path)

patch('color_detector.py', [
    ('(20, 160, 80), (32, 255, 255)', '(15, 170, 90), (32, 255, 255)', False),   # malade : teinte 15-32, saturation >= 170
    ('MIN_AREA = 150', 'MIN_AREA = 60', False),                                   # tiges fines = petites surfaces
    ('kernel = np.ones((5, 5), np.uint8)', 'kernel = np.ones((3, 3), np.uint8)', False),
    ('        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)\n',
     '        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)\n'
     '        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 7), np.uint8))   # recolle tiges+feuilles\n', False),
])
patch('field_scout.py', [
    (r'^CROWN_D = .*$', "CROWN_D = 0.58              # hauteur visible d'une plante de ble (m)", True),
    ('0.75 <= w / h <= 1.6', '0.12 <= w / h <= 0.9', False),                        # plante haute et fine
])
