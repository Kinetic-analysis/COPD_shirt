# importing the required module
import matplotlib.pyplot as plt
  
# x axis values
x = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
# corresponding y axis values
y = [1,2,3,4,5,4,3,2,1,2,3,4,5,4,3,2,1,2,3,4]
  
# plotting the points 
plt.plot(x, y)
  
# naming the x axis
plt.xlabel('Time')
# naming the y axis
plt.ylabel('Respiration')
  
# giving a title to my graph
plt.title('Capacitive Stretch Sensor')
  
# function to show the plot
plt.show()