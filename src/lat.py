#!/usr/bin/python3
# coding:utf-8 
import pandas as pd
import matplotlib.pyplot as plt

# track = pd.read_csv("./src/doodbot/MPCdata/track.csv", sep=',')
goal = pd.read_csv("./src/doodbot/MPCdata/goal.csv", sep=',')


# plt.plot(track['x'], track['y'],color='b')
plt.plot(goal['x'], goal['y'],linewidth=2,color='r')

# plt.xlim(80, 120)
# plt.ylim(-30, 30)
plt.gca().set_aspect('equal', adjustable='box')
# plt.savefig('./src/doodbot/MPCdata/visualize/cost_0-100.pdf', bbox_inches='tight', format='pdf')
plt.savefig('./src/doodbot/MPCdata/visualize/fastest.pdf', format='pdf')
plt.show()