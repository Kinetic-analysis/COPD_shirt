from datetime import datetime
import string
import random
import time

# datetime object containing current date and time
now = datetime.now()

# dd/mm/YY H:M:S
dt_String = now.strftime("%d/%m/%Y %H:%M:%S")
file1 = open("COPD_Data.txt","a")
file1.write(dt_String)
file1.write("\n")
file1.close()

i = 1
while i > 0:
    tijd_String = ''.join(random.choices(string.digits, k = 10))  
    cap_String = ''.join(random.choices(string.digits, k = 10))
    res_String = ''.join(random.choices(string.digits, k = 10))
    
    file1 = open("COPD_Data.txt","a")
    file1.write("Tijd van meting: ")
    file1.write(tijd_String)
    file1.write("\t")
    file1.write("Capcitieve Stretch sensor: ")
    file1.write(cap_String)
    file1.write("\t")
    file1.write("Resistieve Stretch sensor: ")
    file1.write(res_String)
    file1.write("\n")
    file1.close()
    time.sleep(1)