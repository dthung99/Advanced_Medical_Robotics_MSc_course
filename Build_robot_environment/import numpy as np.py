import numpy as np
import matplotlib.pyplot as plt
import threading
from queue import Queue
import time

class test():
    def __init__(self) -> None:
        data_thread = threading.Thread(target=self.generate_data)
        self.fig, self.ax = plt.subplots()
        x = np.random.rand()
        y = np.random.rand()
        self.ax.plot(x,y,"o")
        data_thread.start()
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        # self.fig.show()
        # time.sleep(2)
    def generate_data(self):
        while True:
            # Generate new data points
            x = np.random.rand()
            y = np.random.rand()
            self.ax.plot(x,y)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(1)
test()
plt.show()