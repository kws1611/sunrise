from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

x_num = 50
y_num = 50

x = np.linspace(0, 100, x_num)
y = np.linspace(0, 100, y_num)
z = np.zeros((x_num, y_num))


target = (100, 100)
obs = (50, 25.5)
d_range = 20

for i in range(x_num):
    for j in range(y_num):
        d = ((obs[0] - x[i])**2 + (obs[1] - y[j])**2)**0.5
        
        if d < d_range:
            z[j, i] = (((target[0] - x[i])**2 + (target[1] - y[j])**2)/2)**0.5 + (((1/d - 1/d_range)**2)/2)**0.5
            a = (((target[0] - x[i])**2 + (target[1] - y[j])**2)/2)**0.5
            b = (((1/d - 1/d_range)**2)/2)**0.5
            c = x[i]
            k = y[j]

        else:
            z[j, i] = (((target[0] - x[i])**2 + (target[1] - y[j])**2)/2)**0.5

xx,yy = np.meshgrid(x,y)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(xx,yy,z)
plt.show()
