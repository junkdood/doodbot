#!/usr/bin/python3
# coding:utf-8 
import pandas as pd
import matplotlib.pyplot as plt

track = pd.read_csv("./src/doodbot/MPCdata/track.csv", sep=',')
goal = pd.read_csv("./src/doodbot/MPCdata/goal.csv", sep=',')

plt.plot(goal['x'], goal['y'],linewidth=2,color='r')
plt.plot(track['x'], track['y'],color='b')


plt.show()