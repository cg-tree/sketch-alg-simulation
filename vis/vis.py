import matplotlib.pyplot as plt
import numpy as np

criticalpoints = []
cpi = []
contours = []
levels = []
with open('sketch_plot.txt') as f:
  lines = f.readlines()
  
  write = ""
  for l in lines:
    e = l.split(' ')
    if len(e) == 2:
      if str(e[0]) == "ContourLevel":
        write = e[0]
        contours.append([])
        levels.append( float(e[1]) )
      elif str(e[0]) == "CriticalPoint":
        write = e[0]
        cpi.append( int(e[1]) )
      elif "Pen" in str(e[0]):
        write = e[0]

      elif write == "CriticalPoint":
        criticalpoints.append( (float(e[0]),float(e[1])) )

      elif write == "ContourLevel":
        contours[-1].append( (float(e[0]),float(e[1])) )

fig, ax = plt.subplots()
z = []
for l in range(len(levels)):
  x = []
  y = []
  for p in contours[l]:
    x.append(p[0])
    y.append(p[1])
    z.append(levels[l])
  ax.plot(x,y,label="level "+str(levels[l]))

cx = []
cy = []
for i in range(len(criticalpoints)):
  cx.append(criticalpoints[i][0])
  cy.append(criticalpoints[i][1])

ax.plot(cx,cy,label='CriticalPoints')



ax.set_title('contours and critical points')
ax.legend()
plt.show()
