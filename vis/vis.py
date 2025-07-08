import numpy as np
import matplotlib.pyplot as plt

criticalpoints = [[],[]]
cpi = []
contours = [([],[])]
levels = []
movedrone0 = [[],[]]
movedrone1 = [[],[]]

drones = []
movedrone = dict()
md1 = dict()
md0 = dict()

plt.ion()
fig, ax = plt.subplots()

cp, = ax.plot(criticalpoints[0],criticalpoints[1],linestyle='',marker='x',label='CriticalPoints')
#cp = ax.scatter(criticalpoints[0],criticalpoints[1],label='CriticalPoints')
#md1, = ax.plot(movedrone1[0],movedrone1[1],linestyle='-.',label="MoveDrone1")
#ct, = ax.plot([],[])


with open('sketch_plot.txt') as f:
  lines = f.readlines()
  
  write = ""
  for l in lines:
    e = l.split(' ')
    if len(e) == 2:
      if str(e[0]) == "ContourLevel":

        write = e[0]
        contours.append(([],[]))
        levels.append( float(e[1]) )
        cont, = ax.plot(contours[-1][0],contours[-1][1])

      elif str(e[0]) == "CriticalPoint":
        write = e[0]
        cpi.append( int(e[1]) )

      elif str(e[0]) == "MoveDrone0":
        write = e[0]
        drone = e[1]

      elif str(e[0]) == "MoveDrone1":
        write = e[0]
        drone = e[1]
        print(e[1])
        if drone not in drones:
          drones.append(drone)
          movedrone[drone] =([],[])
          md1[drone], = ax.plot(movedrone[drone][0],movedrone[drone][1])

      elif "Pen" in str(e[0]):
        write = e[0]

      elif write == "CriticalPoint":
        criticalpoints[0].append( float(e[0]) )
        criticalpoints[1].append( float(e[1]) )
        
        color = cp.get_color()
        plt.delaxes(cp)

        cp, = ax.plot(criticalpoints[0],criticalpoints[1],color=color,ls='',marker='x',lw=4,label='CriticalPoints')
        #cp = ax.scatter(criticalpoints[0],criticalpoints[1],color=color,lw=4,label='CriticalPoints')

      elif write == "ContourLevel":
        contours[-1][0].append( float(e[0]) )
        contours[-1][1].append( float(e[1]) )

        color = cont.get_color()
        plt.delaxes(cont)
        
        cont, = ax.plot(contours[-1][0],contours[-1][1],color=color,lw=0.5)

      elif write == "MoveDrone0":
        movedrone[drone][0].append( float(e[0]) )
        movedrone[drone][1].append( float(e[1]) )

        color = md0[drone].get_color()

        plt.delaxes(md0[drone])

        md0[drone], = ax.plot(movedrone[drone][0],movedrone[drone][1],color=color,linestyle='-.',lw=3,label="MoveDrone1 "+drone)
      
      elif write == "MoveDrone1":
        movedrone[drone][0].append( float(e[0]) )
        movedrone[drone][1].append( float(e[1]) )

        color = md1[drone].get_color()

        plt.delaxes(md1[drone])

        md1[drone], = ax.plot(movedrone[drone][0],movedrone[drone][1],color=color,linestyle='-.',lw=3,label="MoveDrone1 "+drone)
    
      else:
        print(write)

    #plt.gca().cla()
    #ax.plot(criticalpoints[0],criticalpoints[1],linestyle=':',label='CriticalPoints')
    #ax.plot(movedrone1[0],movedrone1[1],linestyle='-.',label="MoveDrone 1")
    if write != "ContourLevel":
      ax.set_title('contours and critical points')
      ax.legend()
      plt.draw()
      plt.pause(0.1)
'''
z = []
for l in range(len(levels)):
  x = []
  y = []
  for p in contours[l]:
    x.append(p[0])
    y.append(p[1])
    z.append(levels[l])
  #ax.plot(x,y,label="level "+str(levels[l]))
  ax.plot(x,y)

cx = []
cy = []
for i in range(len(criticalpoints)):
  cx.append(criticalpoints[i][0])
  cy.append(criticalpoints[i][1])

cp, = ax.plot(cx,cy,linestyle=':',label='CriticalPoints')

if len(movedrone0):
  d0x = []
  d0y = []
  for i in range(len(movedrone0)):
    d0x.append(movedrone0[i][0])
    d0y.append(movedrone0[i][1])
  ax.scatter(d0x,d0y,linestyle='-.',label="MoveDrone 0")

d1x = []
d1y = []
for i in range(len(movedrone1)):
  d1x.append(movedrone1[i][0])
  d1y.append(movedrone1[i][1])
md1, = ax.plot(d1x,d1y,linestyle='-.',label="MoveDrone 1")
print(movedrone1)

ax.set_title('contours and critical points')
ax.legend()
'''
plt.show(block=1)
