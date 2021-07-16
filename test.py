import numpy as np
import matplotlib.pyplot as plt

x = []; y = []
for i in range(100):
    x.append(np.sin(np.pi/48*i))
    y.append(2-2*np.cos(np.pi/48*i))

plt.plot(x, y)
plt.show()
