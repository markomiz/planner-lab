import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv
from matplotlib import animation
from matplotlib.animation import FuncAnimation
from itertools import count

x1 = []
y1 = []
x2 = []
y2 = []
x3 = []
y3 = []
node_x = []
node_y = []
testnode_x = []
testnode_y = []

with open ('path0.csv', 'r') as f:
  plots = csv.reader(f, delimiter = ',')
  for row in plots:
    x1.append(float(row[0]))
    y1.append(float(row[1]))

with open ('path1.csv', 'r') as f:
  plots = csv.reader(f, delimiter = ',')
  for row in plots:
    x2.append(float(row[0]))
    y2.append(float(row[1]))

with open ('path2.csv', 'r') as f:
  plots = csv.reader(f, delimiter = ',')
  for row in plots:
    x3.append(float(row[0]))
    y3.append(float(row[1]))

with open ('nodes.txt', 'r') as f:
  plots = csv.reader(f, delimiter = ';')
  for row in plots:
    node_x.append(float(row[0]))
    node_y.append(float(row[1]))

ax1 = []
ay1 = []
ax2 = []
ay2 = []
ax3 = []
ay3 = []

fig, ax = plt.subplots()

ax.set(xlim=(-7, 7), xticks=np.arange(-7, 7),
       ylim=(-7, 7), yticks=np.arange(-7, 7))
plt.grid()
plt.scatter(node_x, node_y,  c = 'tab:gray', linewidth = 0.001)
rect1 = patches.Rectangle((-1,-5),2,7,linewidth=5,edgecolor='b',facecolor='none')
rect2 = patches.Rectangle((-2,3),4,2,linewidth=5,edgecolor='b',facecolor='none')
# Add the patch to the Axes
ax.add_patch(rect1)
ax.add_patch(rect2)

line1, = ax.plot([], [], linewidth=2.5, c='r')
line2, = ax.plot([], [], linewidth=2.5, c='g')
line3, = ax.plot([], [], linewidth=2.5, c='y')
def init(): 
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    return line1, line2, line3
   
def animate(i):
    ax1.append(x1[i])
    ax2.append(x2[i])
    ax3.append(x3[i])
    ay1.append(y1[i])
    ay2.append(y2[i])
    ay3.append(y3[i])
   
    line1.set_data(ax1, ay1)
    line2.set_data(ax2, ay2)
    line3.set_data(ax3, ay3)
    return line1, line2, line3

anim = FuncAnimation(fig, animate, init_func = init, frames = 200, interval = 20, blit = True)
plt.show()