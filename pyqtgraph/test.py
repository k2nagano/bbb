import sys
import time
import threading
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

data = np.zeros(100)
curve = None

def graph():
    global curve
    w = pg.GraphicsWindow()
    w.setWindowTitle('test')
    plt = w.addPlot()
    # plt.setYRange(0, 1)
    curve = plt.plot(pen=(0, 0, 255))

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


threading.Thread(target=graph).start()
input()
for i in range(100):
    print('@@@')
    data = np.delete(data, 0)
    data = np.append(data, np.random.rand())
    curve.setData(data)
    time.sleep(1)