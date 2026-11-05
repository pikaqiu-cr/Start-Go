#!/usr/bin/env python3
"""
rle_to_paths.py — .rle -> contours ordonnés -> commandes U/G/D

Deux modes :
  outline (défaut) : image PLEINE -> ne dessine que la frontière (extérieure + trous)
                     via cv2.findContours.  C'est le mode pour une forme pleine.
  line             : dessin au trait -> squelettise (Zhang-Suen) puis suit le graphe
                     de pixels.  À utiliser seulement si le RLE est déjà un trait.

Principe :
  1. décode le RLE en bitmap
  2. extrait les contours ordonnés (outline) OU le squelette (line)
  3. simplifie chaque polyligne (Ramer-Douglas-Peucker)
  4. envoie  U / G x y / D ... / U  au robot (une polyligne = un trait continu)

Dépendances : numpy, pyserial ; opencv (uniquement pour --mode outline)
Usage :
  python rle_to_paths.py image.rle --port COM3                 # contours d'une forme pleine
  python rle_to_paths.py image.rle --port COM3 --width 200 --speed 80 --flip
  python rle_to_paths.py image.rle --dry-run                   # imprime les commandes
  python rle_to_paths.py trait.rle --port COM3 --mode line     # dessin au trait
"""

import argparse
import sys
import time

import numpy as np

NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
# ordre de parcours : on privilégie les voisins cardinaux (évite de couper les coins)
NB_ORDER = [(0, -1), (0, 1), (-1, 0), (1, 0),
            (-1, -1), (-1, 1), (1, -1), (1, 1)]


# ----------------------------------------------------------------------
# RLE
# ----------------------------------------------------------------------
def decode_rle(text):
    """'w:h:couleur:runs...' -> np.uint8 (h, w), 1 = noir."""
    text = "".join(text.split())
    if not text:
        raise ValueError("fichier RLE vide")

    f = text.split(":")
    try:
        w, h, col = int(f[0]), int(f[1]), int(f[2])
        runs = [int(x) for x in f[3:]]
    except (ValueError, IndexError):
        raise ValueError("RLE invalide : format attendu w:h:couleur:runs...")

    if w < 1 or h < 1:
        raise ValueError(f"dimensions invalides : {w}x{h}")
    if sum(runs) != w * h:
        raise ValueError(f"somme des runs = {sum(runs)} != w*h = {w*h}")

    img = np.zeros((h, w), np.uint8)
    idx = 0
    c = 1 if col else 0
    for r in runs:
        if r < 0:
            raise ValueError("run négatif")
        if c:
            img.flat[idx:idx + r] = 1
        idx += r
        c ^= 1
    return img


# ----------------------------------------------------------------------
# Squelettisation (Zhang-Suen vectorisé)
# ----------------------------------------------------------------------
def thin_zhang_suen(img):
    img = (img > 0).astype(np.uint8)
    prev = img.copy()

    while True:
        for step in (0, 1):
            p = np.pad(img, 1, mode="constant")
            P2 = p[:-2, 1:-1]
            P3 = p[:-2, 2:]
            P4 = p[1:-1, 2:]
            P5 = p[2:, 2:]
            P6 = p[2:, 1:-1]
            P7 = p[2:, :-2]
            P8 = p[1:-1, :-2]
            P9 = p[:-2, :-2]

            B = P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9

            seq = [P2, P3, P4, P5, P6, P7, P8, P9, P2]
            A = np.zeros_like(img)
            for i in range(8):
                A += ((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.uint8)

            m = (img == 1) & (B >= 2) & (B <= 6) & (A == 1)
            if step == 0:
                m &= ((P2 & P4 & P6) == 0)
                m &= ((P4 & P6 & P8) == 0)
            else:
                m &= ((P2 & P4 & P8) == 0)
                m &= ((P2 & P6 & P8) == 0)

            img[m] = 0

        if np.array_equal(img, prev):
            break
        prev = img.copy()

    return img


# ----------------------------------------------------------------------
# Mode outline : frontières d'une forme pleine (+ trous)
# ----------------------------------------------------------------------
def contours_outline(img):
    """img 1=noir -> polylignes (x, y) des frontières extérieures et des trous."""
    try:
        import cv2
    except ImportError:
        raise RuntimeError(
            "mode outline : installer OpenCV -> pip install opencv-python-headless")

    mask = (img > 0).astype(np.uint8) * 255

    # RETR_CCOMP : contours extérieurs + contours des trous
    result = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    cnts = result[0] if len(result) == 2 else result[1]   # compat OpenCV 4/5

    paths = []
    for c in cnts:
        if len(c) < 3:
            continue
        pts = c.reshape(-1, 2)
        path = [(int(x), int(y)) for x, y in pts]   # (x, y)
        path.append(path[0])                        # referme le contour
        paths.append(path)

    return paths


# ----------------------------------------------------------------------
# Suivi du graphe de pixels -> polylignes ordonnées
# ----------------------------------------------------------------------
def trace_paths(skel):
    pts = set(map(tuple, np.argwhere(skel)))
    if not pts:
        return []

    deg = {}
    for (y, x) in pts:
        deg[(y, x)] = sum(((y + dy, x + dx) in pts) for dy, dx in NB8)

    # priorités : bouts (degré 1), puis jonctions (>=3), puis le reste (boucles)
    starts = [p for p in pts if deg[p] == 1]
    starts += [p for p in pts if deg[p] >= 3]
    starts += list(pts)

    unseen = set(pts)
    paths = []

    for s in starts:
        if s not in unseen:
            continue

        path = [s]
        unseen.discard(s)
        cur = s

        while True:
            nxt = None
            for dy, dx in NB_ORDER:
                q = (cur[0] + dy, cur[1] + dx)
                if q in unseen:
                    nxt = q
                    break
            if nxt is None:
                break
            path.append(nxt)
            unseen.discard(nxt)
            cur = nxt

        # boucle fermée : le dernier pixel touche le premier -> on referme
        if len(path) > 2 and (abs(path[-1][0] - path[0][0]) <= 1
                              and abs(path[-1][1] - path[0][1]) <= 1):
            path.append(path[0])

        if len(path) >= 2:
            paths.append(path)

    return paths


# ----------------------------------------------------------------------
# Simplification RDP
# ----------------------------------------------------------------------
def rdp(points, eps):
    if len(points) < 3:
        return list(points)

    pts = np.asarray(points, dtype=float)
    p0, p1 = pts[0], pts[-1]
    d = p1 - p0
    L = float(np.hypot(d[0], d[1]))

    if L < 1e-9:
        dist = np.linalg.norm(pts - p0, axis=1)
    else:
        dist = np.abs(d[0] * (pts[:, 1] - p0[1])
                      - d[1] * (pts[:, 0] - p0[0])) / L

    i = int(np.argmax(dist))
    if dist[i] > eps:
        left = rdp(points[:i + 1], eps)
        right = rdp(points[i:], eps)
        return left[:-1] + right
    return [points[0], points[-1]]


# ----------------------------------------------------------------------
# Robot
# ----------------------------------------------------------------------
class Robot:
    def __init__(self, port, baud=115200):
        import serial
        self.s = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        self.s.reset_input_buffer()

    def cmd(self, c, timeout=300):
        self.s.write((c + "\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.s.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("OK"):
                return line
            if line.startswith("ERR"):
                raise RuntimeError(f"{c!r} -> {line}")
            # INFO / POS / READY... ignorés
        raise TimeoutError(f"pas de réponse à {c!r}")

    def handshake(self, timeout=600):
        self.s.reset_input_buffer()
        self.s.write(b"P\n")
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.s.readline().decode(errors="ignore").strip()
            if line:
                print("  <-", line)
                if line.startswith("PONG"):
                    return
        raise TimeoutError("pas de PONG (mauvais port ? robot occupé ?)")

    def close(self):
        self.s.close()


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rle", help="fichier .rle (dessin au trait)")
    ap.add_argument("--port", default=None, help="COM3, /dev/ttyUSB0, ...")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--width", type=float, default=250.0, help="largeur max (mm)")
    ap.add_argument("--height", type=float, default=250.0, help="hauteur max (mm)")
    ap.add_argument("--margin", type=float, default=10.0, help="marge (mm)")
    ap.add_argument("--speed", type=float, default=80.0, help="vitesse (mm/s)")
    ap.add_argument("--eps", type=float, default=0.8, help="simplification RDP (px)")
    ap.add_argument("--flip", action="store_true", help="inverser l'axe Y")
    ap.add_argument("--mode", choices=["outline", "line"], default="outline",
                    help="outline = forme pleine (frontière) ; line = dessin au trait")
    ap.add_argument("--no-thinning", action="store_true",
                    help="mode line : image déjà 1 px de large")
    ap.add_argument("--max-travel", type=float, default=600.0,
                    help="course max du robot (mm)")
    ap.add_argument("--dry-run", action="store_true",
                    help="imprimer les commandes sans envoyer")
    args = ap.parse_args()

    sys.setrecursionlimit(100000)

    # 1) décodage
    with open(args.rle, encoding="utf-8-sig") as fh:
        img = decode_rle(fh.read())
    h, w = img.shape
    print(f"RLE: {w}x{h} px, {int(img.sum())} pixels noirs")

    # 2) + 3) contours ordonnés, toujours en (x, y) = (colonne, ligne)
    if args.mode == "outline":
        paths = contours_outline(img)
        print(f"mode outline : {len(paths)} frontière(s) (extérieur + trous)")
    else:
        skel = img if args.no_thinning else thin_zhang_suen(img)
        print(f"mode line : squelette = {int(skel.sum())} pixels")
        raw = trace_paths(skel)                       # (ligne, colonne)
        paths = [[(x, y) for (y, x) in p] for p in raw]   # -> (x, y)
        print(f"mode line : {len(paths)} polyligne(s)")

    # 4) échelle + simplification
    scale = min(args.width / w, args.height / h)
    print(f"échelle: {scale:.3f} mm/px -> "
          f"{w * scale:.1f} x {h * scale:.1f} mm (+ marge {args.margin})")

    simplified = []
    total_pts = 0
    for p in paths:
        q = rdp(p, args.eps)
        if len(q) >= 2:
            simplified.append(q)
            total_pts += len(q)
    print(f"points après simplification: {total_pts}")

    # 5) conversion en mm
    def to_mm(p):
        x, y = p
        yy = (h - 1 - y) if args.flip else y
        return (args.margin + x * scale, args.margin + yy * scale)

    all_pts = [to_mm(p) for path in simplified for p in path]
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    print(f"zone: X {xmin:.1f}..{xmax:.1f} mm, Y {ymin:.1f}..{ymax:.1f} mm")

    limit = args.max_travel
    if xmax > limit or ymax > limit:
        sys.exit(f"ERREUR: dépasse la course {limit} mm. "
                 f"Réduire --width/--height ou déplacer l'origine.")

    # 6) envoi
    robot = None
    if not args.dry_run:
        if not args.port:
            sys.exit("ERREUR: --port requis (ou utiliser --dry-run)")
        robot = Robot(args.port, args.baud)
        robot.handshake()
        robot.cmd(f"V {args.speed:.1f}")

    n_cmd = 0
    for i, path in enumerate(simplified):
        print(f"\rcontour {i + 1}/{len(simplified)}", end="", flush=True)

        def emit(c):
            nonlocal n_cmd
            if robot:
                robot.cmd(c)
            else:
                if n_cmd < 25:
                    print("\n  >", c)
            n_cmd += 1

        emit("U")
        pm = [to_mm(p) for p in path]
        emit(f"G {pm[0][0]:.2f} {pm[0][1]:.2f}")
        emit("D")
        for x, y in pm[1:]:
            emit(f"G {x:.2f} {y:.2f}")
        emit("U")

    if robot:
        robot.close()
    print(f"\nterminé: {len(simplified)} contours, {n_cmd} commandes")


if __name__ == "__main__":
    main()
