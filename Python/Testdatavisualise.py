import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

fig = plt.figure()
ax1 = fig.add_subplot(2,2,1)
ax2 = fig.add_subplot(2,2,2)
ax3 = fig.add_subplot(2,2,3)
ax4 = fig.add_subplot(2,2,4)

fig2 = plt.figure()
ax5 = fig2.add_subplot(2,3,1)
ax6 = fig2.add_subplot(2,3,2)
ax7 = fig2.add_subplot(2,3,3)
ax8 = fig2.add_subplot(2,3,4)
ax9 = fig2.add_subplot(2,3,5)
ax10 = fig2.add_subplot(2,3,6)

def animate(i):
    graph_data = open('data.csv','r').read()
    lines = graph_data.split('\n')
    Tijd1s = []
    CapWaarde1s = []
    Tijd2s = []
    CapWaarde2s = []
    Tijd3s = []
    ResWaarde1s = []
    Tijd4s = []
    ResWaarde2s = []
    Tijd5s = []
    AcceleroXs = []
    AcceleroYs = []
    AcceleroZs = []
    GyroXs = []
    GyroYs = []
    GyroZs = []
    
    firstline = True
    for line in lines:
        if len(line) > 3:
            if firstline:
                firstline = False
            else:
                Tijd1, CapWaarde1, Tijd2, CapWaarde2, Tijd3, ResWaarde1, Tijd4, ResWaarde2, Tijd5, AcceleroX, AcceleroY, AcceleroZ, GyroX, GyroY, GyroZ, Dummy = line.split(',')
                Tijd1s.append(int(Tijd1))
                CapWaarde1s.append(int(CapWaarde1))
                Tijd2s.append(int(Tijd2))
                CapWaarde2s.append(int(CapWaarde2))
                Tijd3s.append(int(Tijd3))
                ResWaarde1s.append(int(ResWaarde1))
                Tijd4s.append(int(Tijd4))
                ResWaarde2s.append(int(ResWaarde2))
                Tijd5s.append(int(Tijd5))
                AcceleroXs.append(int(AcceleroX))
                AcceleroYs.append(int(AcceleroY))
                AcceleroZs.append(int(AcceleroZ))
                GyroXs.append(int(GyroX))
                GyroYs.append(int(GyroY))
                GyroZs.append(int(GyroZ))
                
    ax1.clear()
    ax1.plot(Tijd1s, CapWaarde1s, label='Capacitieve Stretch Sensor 1')
    ax1.set_xlim(left=max(0, int(Tijd1)-50000000), right=int(Tijd1))
    ax1.grid(True)
    ax1.set_xlabel('Tijd (us)', fontsize=10)
    ax1.set_ylabel('Lengte (mm)', fontsize=10)
    ax1.set_title('Capacitieve Stretch Sensor 1', fontsize=10)
#     ax1.set_xticklabels(Tijd1s, fontsize=8)
#     ax1.set_yticklabels(CapWaarde1s, fontsize=8)
    
    ax2.clear()
    ax2.plot(Tijd2s, CapWaarde2s, label='Capacitieve Stretch Sensor 2')
    ax2.set_xlim(left=max(0, int(Tijd2)-50000000), right=int(Tijd1))
    ax2.grid(True)
    ax2.set_xlabel('Tijd (us)', fontsize=10)
    ax2.set_ylabel('Lengte (mm)', fontsize=10)
    ax2.set_title('Capacitieve Stretch Sensor 2', fontsize=10)
#     ax2.set_xticklabels(Tijd1s, fontsize=8)
#     ax2.set_yticklabels(CapWaarde1s, fontsize=8)
    
    ax3.clear()
    ax3.plot(Tijd3s, ResWaarde1s, label='Resistieve Stretch Sensor 1')
    ax3.set_xlim(left=max(0, int(Tijd3)-50000000), right=int(Tijd3))
    ax3.grid(True)
    ax3.set_xlabel('Tijd (us)', fontsize=10)
    ax3.set_ylabel('Lengte (mm)', fontsize=10)
    ax3.set_title('Resistieve Stretch Sensor 1', fontsize=10)
#     ax3.set_xticklabels(Tijd1s, fontsize=8)
#     ax3.set_yticklabels(CapWaarde1s, fontsize=8)
    
    ax4.clear()
    ax4.plot(Tijd4s, ResWaarde2s, label='Resistieve Stretch Sensor 2')
    ax4.set_xlim(left=max(0, int(Tijd4)-50000000), right=int(Tijd4))
    ax4.grid(True)
    ax4.set_xlabel('Tijd (us)', fontsize=10)
    ax4.set_ylabel('Lengte (mm)', fontsize=10)
    ax4.set_title('Resistieve Stretch Sensor 2', fontsize=10)
#     ax4.set_xticklabels(Tijd1s, fontsize=8)
#     ax4.set_yticklabels(CapWaarde1s, fontsize=8)
    
def animate2(i):
    T = 0
    graph_data = open('data.csv','r').read()
    lines = graph_data.split('\n')
    Tijd1s = []
    CapWaarde1s = []
    Tijd2s = []
    CapWaarde2s = []
    Tijd3s = []
    ResWaarde1s = []
    Tijd4s = []
    ResWaarde2s = []
    Tijd5s = []
    AcceleroXs = []
    AcceleroYs = []
    AcceleroZs = []
    GyroXs = []
    GyroYs = []
    GyroZs = []
    
    firstline = True
    for line in lines:
        if len(line) > 3:
            if firstline:
                firstline = False
            else:
                Tijd1, CapWaarde1, Tijd2, CapWaarde2, Tijd3, ResWaarde1, Tijd4, ResWaarde2, Tijd5, AcceleroX, AcceleroY, AcceleroZ, GyroX, GyroY, GyroZ, Dummy = line.split(',')
                Tijd1s.append(int(Tijd1))
                CapWaarde1s.append(int(CapWaarde1))
                Tijd2s.append(int(Tijd2))
                CapWaarde2s.append(int(CapWaarde2))
                Tijd3s.append(int(Tijd3))
                ResWaarde1s.append(int(ResWaarde1))
                Tijd4s.append(int(Tijd4))
                ResWaarde2s.append(int(ResWaarde2))
                Tijd5s.append(int(Tijd5))
                AcceleroXs.append(int(AcceleroX))
                AcceleroYs.append(int(AcceleroY))
                AcceleroZs.append(int(AcceleroZ))
                GyroXs.append(int(GyroX))
                GyroYs.append(int(GyroY))
                GyroZs.append(int(GyroZ))
                
    ax5.clear()
    ax5.plot(Tijd5s, AcceleroXs, label='Accelerometer X-axis')
#    ax5.set_xlim(left=max(0, int(Tijd5)-50000000), right=int(Tijd5))
    ax5.grid(True)
    ax5.set_xlabel('Tijd (us)', fontsize=10)
    ax5.set_ylabel('Versnelling (mg)', fontsize=10)
    ax5.set_title('Accelerometer X-axis', fontsize=10)
#     ax5.set_xticklabels(Tijd5s, fontsize=8)
#     ax5.set_yticklabels(AcceleroXs, fontsize=8)
    
    ax6.clear()
    ax6.plot(Tijd5s, AcceleroYs, label='Accelerometer Y-axis')
#    ax6.set_xlim(left=max(0, int(Tijd5)-50000000), right=int(Tijd5))
    ax6.grid(True)
    ax6.set_xlabel('Tijd (us)', fontsize=10)
    ax6.set_ylabel('Versnelling (mg)', fontsize=10)
    ax6.set_title('Accelerometer Y-axis', fontsize=10)
#     ax6.set_xticklabels(Tijd5s, fontsize=8)
#     ax6.set_yticklabels(AcceleroYs, fontsize=8)
    
    ax7.clear()
    ax7.plot(Tijd5s, AcceleroZs, label='Accelerometer Z-axis')
#    ax7.set_xlim(left=max(0, int(Tijd5)-50000000), right=int(Tijd5))
    ax7.grid(True)
    ax7.set_xlabel('Tijd (us)', fontsize=10)
    ax7.set_ylabel('Versnelling (mg)', fontsize=10)
    ax7.set_title('Accelerometer Z-axis', fontsize=10)
#     ax7.set_xticklabels(Tijd5s, fontsize=8)
#     ax7.set_yticklabels(AcceleroZs, fontsize=8)
    
    ax8.clear()
    ax8.plot(Tijd5s, GyroXs, label='Gyroscoop X-axis')
#    ax8.set_xlim(left=max(0, int(Tijd5)-50000000), right=int(Tijd5))
    ax8.grid(True)
    ax8.set_xlabel('Tijd (us)', fontsize=10)
    ax8.set_ylabel('Rotatie (dps)', fontsize=10)
    ax8.set_title('Gyroscoop X-axis', fontsize=10)
#     ax8.set_xticklabels(Tijd5s, fontsize=8)
#     ax8.set_yticklabels(GyroXs, fontsize=8)
    
    ax9.clear()
    ax9.plot(Tijd5s, GyroYs, label='Gyroscoop Y-axis')
#    ax9.set_xlim(left=max(0, int(Tijd5)-50000000), right=int(Tijd5))
    ax9.grid(True)
    ax9.set_xlabel('Tijd (us)', fontsize=10)
    ax9.set_ylabel('Rotatie (dps)', fontsize=10)
    ax9.set_title('Gyroscoop Y-axis', fontsize=10)
#     ax9.set_xticklabels(Tijd5s, fontsize=8)
#     ax9.set_yticklabels(GyroYs, fontsize=8)
    
    ax10.clear()
    ax10.plot(Tijd5s, GyroZs, label='Gyroscoop Z-axis')
#    ax10.set_xlim(left=max(0, int(Tijd5)-50000000), right=int(Tijd5))
    ax10.grid(True)
    ax10.set_xlabel('Tijd (us)', fontsize=10)
    ax10.set_ylabel('Rotatie (dps)', fontsize=10)
    ax10.set_title('Gyroscoop Z-axis', fontsize=10)
#     ax10.set_xticklabels(Tijd5s, fontsize=8)
#     ax10.set_yticklabels(GyroZs, fontsize=8)
    
ani = animation.FuncAnimation(fig, animate, interval=500)
#ani2 = animation.FuncAnimation(fig2, animate2, interval=500)

plt.show()
            