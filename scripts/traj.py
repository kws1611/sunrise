from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from math import pi, sin, cos

x_num = 50
y_num = 50

x = np.linspace(0, 100, x_num)
y = np.linspace(0, 100, y_num)
z = np.zeros((x_num, y_num))


target = np.array((80, 80))
obs_center = np.array((50,50))
obs_list = []

r = 10
d1, d2 = (20, r + 10)
XY_max_vel = 12

k1 = XY_max_vel / d1
k2 = 2*XY_max_vel * (r**2) * r*d2/(d2 - r)

print(k1, k2)

for i in range(x_num):
    for j in range(y_num):
        cur = np.array((x[i], y[j]))

        d_goal = np.linalg.norm((target - cur))
        d_obs = np.linalg.norm((obs_center - cur))

        k11, k12 = (1, 0.5)
        if d_goal <= d1:
            z[j, i] = 0.5 * (k11*(target[0] - cur[0])**2 + k12*(target[1] - cur[1])**2)
            # z[j, i] = 0.5*k1*d_goal**2
        else:
            z[j, i] = (d1*(k11*(target[0] - cur[0])**2
            # z[j, i] = k1*d1*d_goal - 0.5*k1*d1**2

        if d_obs <= d2:
            z[j, i] += 0.5*k2 * (1/(d_obs) - 1/(d2))**2
        
        z[j, i] = min(1000, z[j, i])

xx,yy = np.meshgrid(x,y)

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(xx,yy,z)

fig = plt.figure(2)
plt.contourf(xx, yy, z, levels=40)

p_x = []
p_y = []
pre_p = np.array((1.0, 1.0))
p = np.array((0.0, 0.0))
while True:
    if np.linalg.norm(pre_p - p) < 0.001:
        break

    p_x.append(np.copy(p[0]))
    p_y.append(np.copy(p[1]))
    dt = 0.1

    d_goal = np.linalg.norm((target - p))
    d_obs = np.linalg.norm((obs_center - p))

    if d_goal <= d1:
        grad = k1 * (p - target)
    else:
        grad = k1*d1/d_goal * (p - target)

    if d_obs <= d2:
        grad += k2*(1/(1+d2) - 1/(d_obs))/(((d_obs)**2)*d_obs)*(p - obs_center)

    pre_p = np.copy(p)

    p += -grad*dt

plt.scatter(p_x, p_y)

plt.show()