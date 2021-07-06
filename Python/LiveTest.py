import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from drawnow import *
import numpy    #Import numpy
import time

plt.ion()

def makeFig():
    plt.figure(1)                   #Maakt figuur 1
    plt.subplot(3,4,1)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.title('Respiratie')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd1, CapWaarde1, 'r-',label='Capacitieve Stretch Sensor 1') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda

    plt.subplot(3,4,2)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.title('Weerstation')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd2, CapWaarde2, 'g-',label='Capacitieve Stretch Sensor 2') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,3)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.title('Weerstation')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd3, ResWaarde1, 'b-',label='Resistieve Stretch Sensor 1') #Plot de temperatuur
    plt.xlim(1,5)
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,4)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.title('Weerstation')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd4, ResWaarde2, 'y-',label='Resistieve Stretch Sensor 2') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,5)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Versnelling (mg)')
    plt.plot(Tijd5, AcceleroX, 'r-',label='Accelerometer X-axis') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,6)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Versnelling (mg)')
    plt.plot(Tijd5, AcceleroY, 'r-',label='Accelerometer Y-axis') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,7)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Versnelling (mg)')
    plt.plot(Tijd5, AcceleroZ, 'r-',label='Accelerometer Z-axis') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,8)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rotaties (dps)')
    plt.plot(Tijd5, GyroX, 'r-',label='Gyroscoop X-axis') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,9)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rotaties (dps)')
    plt.plot(Tijd5, GyroY, 'r-',label='Gyroscoop Y-axis') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(3,4,10)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.xlim(1, (396898661))
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rotaties (dps)')
    plt.plot(Tijd5, GyroZ, 'r-',label='Gyroscoop Z-axis') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda


    

    
while True:
    data = pd.read_csv('data.csv')
    Tijd1 = data['Tijd1']
    Tijd2 = data['Tijd2']
    Tijd3 = data['Tijd3']
    Tijd4 = data['Tijd4']
    CapWaarde1 = data['CapWaarde1']
    CapWaarde2 = data['CapWaarde2']
    ResWaarde1 = data['ResWaarde1']
    ResWaarde2 = data['ResWaarde2']
    Tijd5 = data['Tijd5']
    AcceleroX = data['AcceleroX']
    AcceleroY = data['AcceleroY']
    AcceleroZ = data['AcceleroZ']
    GyroX = data['GyroX']
    GyroY = data['GyroY']
    GyroZ = data['GyroZ']
    drawnow(makeFig)