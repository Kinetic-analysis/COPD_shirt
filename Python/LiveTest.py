import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from drawnow import *
import numpy    #Import numpy

#plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()

plt.ion()

def makeFig():
    plt.figure(1)                   #Maakt figuur 1
    plt.subplot(411)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    plt.title('Respiratie')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd1, CapWaarde1, 'r-',label='Capacitieve Stretch Sensor 1') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda

    plt.subplot(412)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.title('Weerstation')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd2, CapWaarde2, 'g-',label='Capacitieve Stretch Sensor 2') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(413)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.title('Weerstation')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd3, ResWaarde1, 'b-',label='Resistieve Stretch Sensor 1') #Plot de temperatuur
    plt.legend(loc='upper left', prop={'size': 8})    #Plot de legenda
    
    plt.subplot(414)                #1e grafiek van de 3
    plt.xlabel(' ')             
    #plt.ylim(25,40)                 #Ymax en Ymin
    #plt.title('Weerstation')        #Geeft titel weer
    plt.grid(True)                  #Zet grid aan
    plt.ylabel('Rek (cm)')
    plt.plot(Tijd4, ResWaarde2, 'y-',label='Resistieve Stretch Sensor 2') #Plot de temperatuur
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
    Test = 1000
    drawnow(makeFig)