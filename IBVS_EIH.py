import numpy as np
import matplotlib.pyplot as plt

# =============== MATRICE DE ROTATION ================
def rotation_matrix_from_euler(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    return Rz @ Ry @ Rx

# =================== CAMERA ========================
class PerspectiveCamera:
    def __init__(self, au=800.0, av=800.0, u0=320.0, v0=240.0):
        self.K = np.array([[au, 0, u0],
                           [0, av, v0],
                           [0, 0, 1]])
        self.position = np.zeros(3, dtype=float)
        self.orientation = np.eye(3, dtype=float)

    def set_pose(self, position, euler_angles):
        self.position = np.array(position, dtype=float)
        self.orientation = rotation_matrix_from_euler(*euler_angles)

    def apply_velocity(self, v, w, dt=0.05):
        wx, wy, wz = w
        wRc = rotation_matrix_from_euler(wx*dt, wy*dt, wz*dt)
        self.orientation = wRc.T @ self.orientation

        wMc = self.orientation.T @ np.array(v)
        self.position += wMc * dt

    def perspective_projection(self, wP):
        u = []
        for P in wP:
            wP_ = np.array(P)
            cP = self.orientation @ (wP_ - self.position)
            if cP[2] <= 0:
                u.append(None)
                continue
            u_ = self.K @ (cP / cP[2])
            u.append([u_[0], u_[1]])
        return u

# ===================== SCENE =========================
## Points 3D
oX = np.array([
    [0.5, 0.5, 5.0],
    [0.5, -0.5, 5.0],
    [-0.5, -0.5, 5.0],
    [-0.5, 0.5, 5.0]
], dtype=float)
## Caméra
cam = PerspectiveCamera()
cam.set_pose([0, 0, 0], [0, 0, 0])
# ====================================================
u_star = cam.perspective_projection(oX)
cam.set_pose([0, 0, 0], [0, np.pi/8, 0])

# ================= FIGURES ==========================
fig = plt.figure(figsize=(15, 10))
plot_3d = fig.add_subplot(2, 3, 1, projection='3d')
plot_img = fig.add_subplot(2, 3, 2)
## Scène 3D
# Points 3D
plot_3d.scatter(oX[:, 0], oX[:, 1], oX[:, 2], color='red')
plot_3d.set_xlim(-4, 4)
plot_3d.set_ylim(-4, 4)
plot_3d.set_zlim(-2, 6)
plot_3d.set_aspect('equal', adjustable='box')
plot_3d.set_title("Scène 3D")
#Camera
frustum = np.array([[0, 0, 0], [0.4, 0.4, 1], [-0.4, 0.4, 1], [-0.4, -0.4, 1], [0.4, -0.4, 1]])
cam_plot = []
## Image 2D
plot_img.set_xlim(0, 640)
plot_img.set_ylim(480, 0)
plot_img.set_aspect('equal', adjustable='box')
plot_img.set_title("Image 2D")
u_plot = plot_img.scatter([], [], color='red')
u_star_plot = plot_img.scatter([], [], color='green')

# ====================================================

au, av = cam.K[0,0], cam.K[1,1]
u0, v0 = cam.K[0,2], cam.K[1,2]

e_plot = fig.add_subplot(2,3,4)
v_plot = fig.add_subplot(2,3,5)
w_plot = fig.add_subplot(2,3,6)

es = []
vs = [[] for _ in range(3)]
ws = [[] for _ in range(3)]
colors = ["red", "green", "blue"]
# ===================== MAIN =========================
for i in range(100):

    # Projection perspective des points 3D
    U = cam.perspective_projection(oX)

    # TODO: IBVS basé points
    e = np.array(U) - np.array(u_star)
    e = e.reshape((8,1))
    
    Lu = []
    for u, v in U:
        Z = 5
        x = (u - u0) / au
        y = (v - v0) / av

        Lu_i = np.array([
            [-au/Z, 0, au*x/Z, au*x*y, -(1+x**2)*au, au*y],
            [0, -av/Z, av*y/Z, (1+y**2)*av, -av*x*y, -av*x]
        ])
        Lu.append(Lu_i)

    Lu = np.vstack(Lu)
    Lu_pinv = np.linalg.pinv(Lu)
    lamda = 0.1
    vc = -lamda * (Lu_pinv @ e)

    # Envoi de vitesses linéaires et anulaires à la caméra
    cam.apply_velocity(vc.T[0][:3], vc.T[0][3:], dt=1)


    # ================= PLOTS ==========================
    # Plot des points 2D (projection des points 3D)
    u_plot.set_offsets(U)
    u_star_plot.set_offsets(u_star)
    # Plot de la camera en 3D
    for line in cam_plot:
        line.remove()
    cam_plot = []
    frustum_world = []
    for p in frustum:
        wP = cam.orientation @ p + cam.position
        frustum_world.append(wP)
    frustum_world = np.array(frustum_world)
    for i, j in [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]:
        line = plot_3d.plot([frustum_world[i, 0], frustum_world[j, 0]], [frustum_world[i, 1], frustum_world[j, 1]], [frustum_world[i, 2], frustum_world[j, 2]], color='blue')[0]
        cam_plot.append(line)

    for i, v_i in enumerate(vc[:3]):
        vs[i].append(v_i)
        v_plot.plot(vs[i], color=colors[i])
    v_plot.set_title("Linear")
    for i, w_i in enumerate(vc[3:]):
        ws[i].append(w_i)
        w_plot.plot(ws[i], color=colors[i])
    w_plot.set_title("Angular")
    es.append(np.linalg.norm(e))
    e_plot.plot(es, color='red')
    e_plot.set_title("Error")
    plt.draw()
    plt.pause(0.1)
# ====================================================
