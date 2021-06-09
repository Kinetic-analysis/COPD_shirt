import pandas as pd
from matplotlib import pyplot as plt

plt.style.use('seaborn')

data = pd.read_csv('data.csv')
Tijd1 = data['Tijd1']
CapWaarde1 = data['CapWaarde1']
Tijd2 = data['Tijd2']
CapWaarde2 = data['CapWaarde2']
Tijd3 = data['Tijd3']
ResWaarde1 = data['ResWaarde1']
Tijd4 = data['Tijd4']
ResWaarde2 = data['ResWaarde2']


fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, ncols=1, sharex=True)

ax1.plot(Tijd1, CapWaarde1, label='Capacitive Stretch Sensor 1')
ax2.plot(Tijd2, CapWaarde2, label='Capacitive Stretch Sensor 2')
ax3.plot(Tijd3, ResWaarde1, label='Resistive Stretch Sensor 1')
ax4.plot(Tijd4, ResWaarde2, label='Resistive Stretch Sensor 2')

ax1.legend()
ax1.set_title('Capacitieve Stretch Sensor 1')
ax1.set_ylabel('Rek (cm)')

ax2.legend()
ax2.set_title('Capacitieve Stretch Sensor 2')
ax2.set_ylabel('Rek (cm)')

ax3.legend()
ax3.set_title('Resistieve Stretch Sensor 1')
ax3.set_ylabel('Rek (cm)')

ax4.legend()
ax4.set_title('Resistieve Stretch Sensor 2')
ax4.set_xlabel('Tijd (us)')
ax4.set_ylabel('Rek (cm)')

plt.tight_layout()

plt.show()