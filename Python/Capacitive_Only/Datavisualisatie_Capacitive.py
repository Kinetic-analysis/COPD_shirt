import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

fig = plt.figure()
ax2 = fig.add_subplot(1,1,1)


def animate(i):
    graph_data = open('data.csv','r').read()
    lines = graph_data.split('\n')
    Tijd2s = []
    CapWaarde2s = []
    
    firstline = True
    for line in lines:
        if len(line) > 3:
            if firstline:
                firstline = False
            else:
                Tijd2, CapWaarde2, Dummy = line.split(',')
                Tijd2s.append(int(Tijd2))
                CapWaarde2s.append(int(CapWaarde2))           

    ax2.clear()
    ax2.plot(Tijd2s, CapWaarde2s, label='Capacitieve Stretch Sensor 2')
    ax2.set_xlim(left=max(0, int(Tijd2)-50000000), right=int(Tijd2))
    ax2.grid(True)
    ax2.set_xlabel('Tijd (us)', fontsize=10)
    ax2.set_ylabel('Lengte (mm)', fontsize=10)
    ax2.set_title('Capacitieve Stretch Sensor 2', fontsize=10)

ani = animation.FuncAnimation(fig, animate, interval=500)


plt.show()
            