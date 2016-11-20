#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import sys

FFT_SIZE = 131072
i = 0

x = []
y = []

for r in range(FFT_SIZE):
    x.append(r)
    y.append(r)
   
   
plt.show()
 
axes = plt.gca()
axes.set_xlim(0, FFT_SIZE)
axes.set_ylim(0, 64)
line1, = axes.plot(x, y, 'r-')
plt.setp(line1, linewidth=1, color='r')


shown = 0
for line in sys.stdin:
	y[i] = line
	i = i + 1
	if i == FFT_SIZE:
		line1.set_xdata(x)
		line1.set_ydata(y)
		plt.draw()
		plt.pause(1e-17)
		i = 0
		
		
		
plt.show()
