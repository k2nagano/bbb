import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
ax = fig.add_subplot(111)
t = []
x = []

def animate(i, t, x):
    tt = time.time()
    xx = np.sin(tt)
    t.append(tt)
    x.append(xx)
    t = t[-100:]
    x = x[-100:]
    ax.clear()
    ax.plot(t, x)

ani = animation.FuncAnimation(fig, animate, fargs=(t, x), interval=500)
plt.show()
