from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
import numpy as np

world_x = 1000
world_y = 1000

probs = np.zeros((world_x, world_y))

chair_x = 10
chair_y = 10

mean = [0, 0]
covariance_mat = [[5, 0], [0, 9]]

x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)

X, Y = np.meshgrid(x, y)
pos = np.dstack((X, Y))
rv = multivariate_normal(mean, covariance_mat, 5000)
Z = rv.pdf(pos)

ax = plt.axes() #projection='3d'
ax.plot_surface(X, Y, Z, rstride=1, cstride=1,
                cmap='viridis', edgecolor='none')

plt.show()

