#!/usr/bin/env python3
"""AgriScout : cartographie des plantes malades, fenêtre caméra en direct et rapport final.

Lancer : python3 field_scout.py   (Gazebo doit tourner)
Fenêtre : vert = plante saine, rouge = plante malade (avec distance estimée),
          gris = plante vue mais ignorée (coupée par le bord, trop loin ou collée à une autre).
Touche  : 'r' dans la fenêtre = sauvegarder le rapport tout de suite.
Rapport : ~/agri_ws/rapport_agriscout.csv et .txt (sauvegardé aussi toutes les 15 s et à Ctrl+C).
RViz    : topic /plant_map (MarkerArray), sphère verte = saine, rouge = malade.
"""
import csv
import math
import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from color_detector import detect

FOV = 1.75                  # champ de vision horizontal (rad) : doit égaler celui du modèle Gazebo
CROWN_D = 0.36              # diamètre réel d'une couronne (m) = 2 x 0.18
MIN_RANGE, MAX_RANGE = 0.5, 3.0   # distances fiables (m)
MERGE_DIST = 0.35           # 2 détections plus proches = même plante (m)
MATCH_DIST = 0.4            # détection <-> vraie plante (pour le rapport)

OUT_DIR = os.path.expanduser('~/agri_ws')
TRUTH = os.path.expanduser('~/agri_ws/src/agri_robot/worlds/plants_truth.csv')
GREEN, RED, GRAY = (0, 200, 0), (0, 0, 255), (150, 150, 150)   # BGR
FONT = cv2.FONT_HERSHEY_SIMPLEX


def locate(box, img_w, img_h, pose):
    """Position (x, y) dans 'odom' d'une plante vue dans la boîte (x, y, w, h), ou None."""
    x, y, w, h = box
    if x <= 2 or y <= 2 or x + w >= img_w - 2 or y + h >= img_h - 2:
        return None                                # boîte coupée par le bord
    if not 0.75 <= w / h <= 1.6:
        return None                                # forme anormale (couronnes collées)
    f = (img_w / 2) / math.tan(FOV / 2)            # focale en pixels
    z = f * CROWN_D / h                            # profondeur (m) : la hauteur reste fiable sur les côtés
    if not MIN_RANGE <= z <= MAX_RANGE:
        return None
    cx = x + w / 2
    xr = z                                         # devant le robot
    yr = -z * (cx - img_w / 2) / f                 # gauche du robot (+)
    px, py, yaw = pose
    return (px + xr * math.cos(yaw) - yr * math.sin(yaw),
            py + xr * math.sin(yaw) + yr * math.cos(yaw))


def put(img, text, org, color=(255, 255, 255), scale=0.5):
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), 3)       # contour noir
    cv2.putText(img, text, org, FONT, scale, color, 1)


class FieldScout(Node):
    def __init__(self):
        super().__init__('field_scout')
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/mission/status', self.on_status, 10)
        self.pub = self.create_publisher(MarkerArray, '/plant_map', 10)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(15.0, lambda: self.save_report(verbose=False))
        self.bridge = CvBridge()
        self.pose = None
        self.report_done = False
        self.plants = []        # chaque plante : {'x', 'y', 'n', 'saine', 'malade'}
        cv2.namedWindow('AgriScout')
        self.get_logger().info("AgriScout demarre. Touche 'r' dans la fenetre = rapport.")

    # ------------------------------------------------------------ capteurs
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_status(self, msg):
        # Le pilote automatique annonce 'done' en fin de champ : rapport automatique
        if msg.data == 'done' and not self.report_done:
            self.report_done = True
            self.get_logger().info('Mission terminee : generation du rapport.')
            self.save_report(verbose=True)

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge : {e}')
            return
        cv2.imshow('AgriScout', self.process(frame))
        if cv2.waitKey(1) & 0xFF == ord('r'):
            self.save_report(verbose=True)

    # ------------------------------------------------------------ traitement
    def process(self, frame):
        """Détecte, localise, mémorise, et renvoie l'image annotée."""
        h_img, w_img = frame.shape[:2]
        f = (w_img / 2) / math.tan(FOV / 2)
        out = frame.copy()
        if self.pose is not None:
            for etat, x, y, w, h in detect(frame):
                pos = locate((x, y, w, h), w_img, h_img, self.pose)
                if pos is None:
                    cv2.rectangle(out, (x, y), (x + w, y + h), GRAY, 1)
                    continue
                self.add_observation(pos, etat)
                color = GREEN if etat == 'saine' else RED
                cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
                put(out, f'{etat} {f * CROWN_D / h:.1f}m', (x, max(y - 6, 14)), color)
        n_sick = sum(1 for p in self.plants if p['malade'] > p['saine'])
        put(out, f'Carte : {len(self.plants)} plantes | {n_sick} malades', (10, 20), scale=0.6)
        if self.pose is not None:
            put(out, f'robot x={self.pose[0]:.1f} y={self.pose[1]:.1f}', (10, 42))
        put(out, "gris = ignoree | 'r' = rapport", (10, h_img - 10), (230, 230, 230), 0.45)
        return out

    def add_observation(self, pos, etat):
        for p in self.plants:
            if math.hypot(p['x'] - pos[0], p['y'] - pos[1]) < MERGE_DIST:
                p['x'] = (p['x'] * p['n'] + pos[0]) / (p['n'] + 1)   # moyenne glissante
                p['y'] = (p['y'] * p['n'] + pos[1]) / (p['n'] + 1)
                p['n'] += 1
                p[etat] += 1
                return
        self.plants.append({'x': pos[0], 'y': pos[1], 'n': 1,
                            'saine': int(etat == 'saine'), 'malade': int(etat == 'malade')})

    # ------------------------------------------------------------ RViz
    def publish_map(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, p in enumerate(self.plants):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'odom', now
            m.ns, m.id = 'plants', i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p['x'], p['y'], 0.25
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            sick = p['malade'] > p['saine']
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.0, 0.0, 1.0) if sick else (0.0, 0.8, 0.0, 1.0)
            arr.markers.append(m)
        self.pub.publish(arr)

    # ------------------------------------------------------------ rapport
    def save_report(self, verbose=True):
        """Écrit rapport_agriscout.csv/.txt ; compare à la vérité terrain si elle existe."""
        truth = []
        if os.path.exists(TRUTH):
            with open(TRUTH, newline='', encoding='utf-8') as f:
                truth = [(r['id'], float(r['x']), float(r['y']), r['etat']) for r in csv.DictReader(f)]
        det = [(f'D{i + 1:02d}', p['x'], p['y'], 'malade' if p['malade'] > p['saine'] else 'saine', p['n'])
               for i, p in enumerate(self.plants)]

        pairs = sorted((math.hypot(t[1] - d[1], t[2] - d[2]), i, j)
                       for i, t in enumerate(truth) for j, d in enumerate(det))
        used_t, used_d, match = set(), set(), {}
        for dist, i, j in pairs:
            if dist < MATCH_DIST and i not in used_t and j not in used_d:
                used_t.add(i); used_d.add(j); match[j] = i

        os.makedirs(OUT_DIR, exist_ok=True)
        with open(os.path.join(OUT_DIR, 'rapport_agriscout.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['id', 'x', 'y', 'etat_detecte', 'observations', 'vraie_plante', 'vrai_etat', 'verdict'])
            for j, d in enumerate(det):
                if not truth:
                    verdict, tid, tet = '', '', ''
                elif j in match:
                    t = truth[match[j]]
                    tid, tet = t[0], t[3]
                    verdict = 'OK' if t[3] == d[3] else 'ERREUR'
                else:
                    tid, tet, verdict = '', '', 'FAUSSE_DETECTION'
                w.writerow([d[0], round(d[1], 2), round(d[2], 2), d[3], d[4], tid, tet, verdict])

        n_sick = sum(1 for d in det if d[3] == 'malade')
        lines = [f'RAPPORT AGRISCOUT - {datetime.now():%Y-%m-%d %H:%M:%S}',
                 f'Plantes detectees : {len(det)} dont {n_sick} malades', '']
        for j, d in enumerate(det):
            tag = ''
            if truth:
                tag = ('  -> ' + truth[match[j]][0] + (' OK' if truth[match[j]][3] == d[3] else ' ERREUR')) \
                    if j in match else '  -> fausse detection'
            lines.append(f'  {d[0]}  x={d[1]:5.2f} y={d[2]:5.2f}  {d[3]:6s}  (vue {d[4]} fois){tag}')
        if truth:
            ok = sum(1 for j, i in match.items() if truth[i][3] == det[j][3])
            tp = sum(1 for j, i in match.items() if truth[i][3] == 'malade' and det[j][3] == 'malade')
            fp = sum(1 for j, i in match.items() if truth[i][3] == 'saine' and det[j][3] == 'malade')
            n_true_sick_seen = sum(1 for i in match.values() if truth[i][3] == 'malade')
            n_true_sick = sum(1 for t in truth if t[3] == 'malade')
            lines += ['', f'VERITE TERRAIN : {len(truth)} plantes dont {n_true_sick} malades',
                      f'Couverture : {len(match)}/{len(truth)} plantes vues ({100 * len(match) / len(truth):.0f} %)']
            if match:
                lines.append(f'Exactitude saine/malade (sur les plantes vues) : {100 * ok / len(match):.0f} %')
            if n_true_sick_seen:
                lines.append(f'Malades retrouvees (rappel) : {tp}/{n_true_sick_seen} vues ({100 * tp / n_true_sick_seen:.0f} %)')
            if tp + fp:
                lines.append(f'Alertes "malade" justes (precision) : {100 * tp / (tp + fp):.0f} %')
            lines.append(f'Fausses detections : {len(det) - len(match)}')
            missed = [truth[i] for i in range(len(truth)) if i not in used_t]
            lines.append('Jamais vues : ' + (', '.join(f'{t[0]}({t[3]})' for t in missed) or 'aucune'))
        text = '\n'.join(lines)
        with open(os.path.join(OUT_DIR, 'rapport_agriscout.txt'), 'w', encoding='utf-8') as f:
            f.write(text + '\n')
        if verbose:
            print('\n' + text + f'\n\nFichiers : {OUT_DIR}/rapport_agriscout.csv et .txt\n')


def main(args=None):
    rclpy.init(args=args)
    node = FieldScout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_report(verbose=True)            # rapport final (Ctrl+C)
        cv2.destroyAllWindows()
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
