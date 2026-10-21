"""Prototype jetable : drone Skydio X2 (MuJoCo Menagerie) dans l'environnement « murs troués ».

Mesure ce que coûterait une migration vers MuJoCo :
- export des collisionneurs existants (Box, Sphere, VerticalCylinder) en géométries MJCF ;
- contrôleur en cascade vitesse -> attitude -> poussées des 4 moteurs ;
- coût d'un pas de physique et du rendu hors écran de 2 et 12 caméras.

Usage : python proto_mujoco/proto_x2.py --menagerie <dossier skydio_x2> [--render]
(MUJOCO_GL=egl ou osmesa pour le rendu sans fenêtre.)

Repères : notre simulateur est Y-haut, MuJoCo Z-haut. (x, y, z)_nous -> (x, -z, y)_mujoco.
"""

import argparse
import math
import os
import sys
import time
from unittest.mock import MagicMock

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from collision import Box, Sphere, VerticalCylinder  # noqa: E402
from walls_env import WallMaze  # noqa: E402

G = 9.81


def to_mj(p):
    return np.array([p[0], -p[2], p[1]])


def colliders_to_mjcf(colliders):
    """Géométries statiques MJCF à partir des collisionneurs du simulateur."""
    out = []
    for c in colliders:
        if isinstance(c, Box):
            h = (c.half[0], c.half[2], c.half[1])
            out.append(f'<geom type="box" pos="{" ".join(map(str, to_mj(c.center)))}" '
                       f'size="{h[0]} {h[1]} {h[2]}" rgba=".75 .7 .6 1"/>')
        elif isinstance(c, Sphere):
            out.append(f'<geom type="sphere" pos="{" ".join(map(str, to_mj(c.center)))}" size="{c.radius}"/>')
        elif isinstance(c, VerticalCylinder):
            cx, cz = c.center_xz
            pos = to_mj((cx, (c.y_min + c.y_max) / 2, cz))
            out.append(f'<geom type="cylinder" pos="{" ".join(map(str, pos))}" '
                       f'size="{c.radius} {(c.y_max - c.y_min) / 2}"/>')
    return "\n".join(out)


def stereo_cameras(n_pairs, baseline=0.18, fovy=60.0):
    """n_pairs paires stéréo réparties tous les 360/n_pairs degrés autour du drone."""
    cams = []
    for k in range(n_pairs):
        a = math.radians(360.0 * k / n_pairs)
        fwd = np.array([math.cos(a), math.sin(a), 0.0])     # avant de la paire (repère drone)
        left = np.array([-math.sin(a), math.cos(a), 0.0])
        for side, name in ((0.5, "L"), (-0.5, "R")):
            p = 0.12 * fwd + side * baseline * left + np.array([0, 0, 0.05])
            x_axis = -left                                    # axe x caméra = droite image
            cams.append(f'<camera name="cam{k}{name}" pos="{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" '
                        f'xyaxes="{x_axis[0]:.4f} {x_axis[1]:.4f} 0 0 0 1" fovy="{fovy}"/>')
    return "\n".join(cams)


def build_model(menagerie, maze, n_pairs, colliders=None):
    with open(os.path.join(menagerie, "x2.xml")) as f:
        x2 = f.read()
    x2 = x2.replace('assetdir="assets"', f'assetdir="{os.path.join(menagerie, "assets")}"')
    sx, sy, sz = to_mj(maze.spawn[0])
    x2 = x2.replace('<body name="x2" pos="0 0 0.1" childclass="x2">',
                    f'<body name="x2" pos="{sx} {sy} {sz}" childclass="x2">\n{stereo_cameras(n_pairs)}')
    x2 = x2.replace("</worldbody>", f"""
    <light pos="60 0 30" dir="0 0 -1" directional="true"/>
    <geom type="plane" size="200 200 .1" rgba=".6 .6 .6 1"/>
    {colliders_to_mjcf(maze.colliders if colliders is None else colliders)}
  </worldbody>""")
    x2 = x2.replace("<keyframe>", '<visual><global offwidth="640" offheight="480"/></visual>\n  <keyframe>')
    x2 = x2.replace('qpos="0 0 .3 1 0 0 0"', f'qpos="{sx} {sy} {sz} 0.7071 0 0 0.7071"')  # lacet 90° : face à -Z chez nous
    return mujoco.MjModel.from_xml_string(x2)


class CascadeController:
    """Vitesse monde + lacet désirés -> poussées des 4 rotors (contrôle géométrique sur SO(3))."""

    def __init__(self, model):
        self.m = model
        self.bid = model.body("x2").id
        self.mass = float(model.body_subtreemass[self.bid])
        # Inertie dans le repère corps (les axes principaux sont tournés de body_iquat)
        Ri = np.zeros(9)
        mujoco.mju_quat2Mat(Ri, model.body_iquat[self.bid])
        Ri = Ri.reshape(3, 3)
        self.J = Ri @ np.diag(model.body_inertia[self.bid]) @ Ri.T
        com = model.body_ipos[self.bid]
        # Matrice d'allocation : [T, tx, ty, tz] = A @ f
        A = np.zeros((4, 4))
        for i in range(4):
            r = model.site_pos[model.site(f"thrust{i + 1}").id] - com
            A[:, i] = [1.0, r[1], -r[0], model.actuator_gear[i, 5]]
        self.A_inv = np.linalg.inv(A)
        self.kv = 4.0          # gain vitesse (1/s) -> temps de réponse ~ 0.25 s
        self.kR = 60.0 * self.J  # gains attitude, mis à l'échelle de l'inertie
        self.kW = 12.0 * self.J
        self.a_max = 10.0
        self.tilt_max = math.radians(35.0)

    def __call__(self, data, v_des, yaw_des):
        R = data.xmat[self.bid].reshape(3, 3)
        v = data.qvel[0:3]
        w = data.qvel[3:6]  # vitesse angulaire, repère corps (freejoint)
        a = np.clip(self.kv * (v_des - v), -self.a_max, self.a_max)
        f_des = self.mass * (a + np.array([0, 0, G]))
        # Limite d'inclinaison
        horiz = np.linalg.norm(f_des[:2])
        lim = f_des[2] * math.tan(self.tilt_max)
        if horiz > lim:
            f_des[:2] *= lim / horiz
        b3 = f_des / np.linalg.norm(f_des)
        c = np.array([math.cos(yaw_des), math.sin(yaw_des), 0.0])
        b2 = np.cross(b3, c)
        b2 /= np.linalg.norm(b2)
        Rd = np.column_stack([np.cross(b2, b3), b2, b3])
        E = 0.5 * (Rd.T @ R - R.T @ Rd)
        eR = np.array([E[2, 1], E[0, 2], E[1, 0]])
        tau = -self.kR @ eR - self.kW @ w + np.cross(w, self.J @ w)
        T = float(f_des @ R[:, 2])
        f = self.A_inv @ np.array([T, *tau])
        return np.clip(f, *self.m.actuator_ctrlrange[0])


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def run(args):
    maze = WallMaze(MagicMock(), seed=args.seed)
    # 1) Réponse à un échelon de vitesse en terrain dégagé : 3 m/s vers l'avant
    model = build_model(args.menagerie, maze, args.pairs, colliders=[])
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    ctrl = CascadeController(model)
    print(f"masse {ctrl.mass:.3f} kg, dt {model.opt.timestep}s")
    yaw0 = math.pi / 2
    log = []
    t_step = 0.0
    n = int(4.0 / model.opt.timestep)
    for k in range(n):
        t = k * model.opt.timestep
        v_des = np.array([0.0, 3.0, 0.0]) if t > 1.0 else np.zeros(3)
        data.ctrl[:] = ctrl(data, v_des, yaw0)
        t0 = time.perf_counter()
        mujoco.mj_step(model, data)
        t_step += time.perf_counter() - t0
        R = data.xmat[ctrl.bid].reshape(3, 3)
        tilt = math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))
        log.append((t, data.qvel[1], data.qpos[2], tilt, data.ncon))
    log = np.array(log)
    after = log[log[:, 0] > 1.0]
    t63 = after[after[:, 1] > 0.63 * 3.0][0, 0] - 1.0
    t90 = after[after[:, 1] > 0.9 * 3.0][0, 0] - 1.0
    print(f"échelon 3 m/s : t63 {t63:.2f}s, t90 {t90:.2f}s, inclinaison max {after[:, 3].max():.1f}°, "
          f"dérive altitude {np.ptp(after[:, 2]) * 100:.1f} cm")
    print(f"pas physique : {t_step / n * 1e6:.1f} µs (+ contrôleur Python)")

    # 2) Murs troués : vol droit à 3 m/s jusqu'au premier mur (choc réel)
    model = build_model(args.menagerie, maze, args.pairs)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    ctrl = CascadeController(model)
    print(f"murs : {model.ngeom} géométries, {model.ncam} caméras")
    first_contact = None
    for k in range(int(6.0 / model.opt.timestep)):
        data.ctrl[:] = ctrl(data, np.array([0.0, 3.0, 0.0]), yaw0)
        mujoco.mj_step(model, data)
        if data.ncon and first_contact is None:
            first_contact = (k * model.opt.timestep, data.qpos[:3].round(2))
    R = data.xmat[ctrl.bid].reshape(3, 3)
    print(f"premier contact {first_contact}, état final {data.qpos[:3].round(2)}, "
          f"inclinaison {math.degrees(math.acos(np.clip(R[2, 2], -1, 1))):.0f}°")

    # 3) Pas complet avec contrôleur Python
    t0 = time.perf_counter()
    for _ in range(1000):
        data.ctrl[:] = ctrl(data, np.zeros(3), yaw0)
        mujoco.mj_step(model, data)
    print(f"pas physique + contrôleur : {(time.perf_counter() - t0) * 1000 / 1000:.3f} ms")

    if args.render:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 480, 640)
        for n_cam in (2, 2 * args.pairs):
            t0 = time.perf_counter()
            frames = 20
            for _ in range(frames):
                for c in range(n_cam):
                    renderer.update_scene(data, camera=c)
                    rgb = renderer.render()
            dt = (time.perf_counter() - t0) / frames
            print(f"rendu {n_cam} caméras 640x480 : {dt * 1000:.1f} ms/frame ({1 / dt:.1f} fps)")
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=0)
        depth = renderer.render()
        print(f"profondeur : {depth.dtype} {depth.shape}, min {depth.min():.2f} m")
        renderer.disable_depth_rendering()
        try:
            from PIL import Image
            for c, name in ((0, "L"), (1, "R")):
                renderer.update_scene(data, camera=c)
                Image.fromarray(renderer.render()).save(os.path.join(args.out, f"mj_cam{name}.png"))
        except ImportError:
            pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--menagerie", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--pairs", type=int, default=6)
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--out", default=".")
    run(ap.parse_args())
