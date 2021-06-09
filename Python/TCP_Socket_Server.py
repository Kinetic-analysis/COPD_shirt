import socket
import csv
import random
import time

HOST = '192.168.192.42'  # Standard loopback interface address (localhost)
PORT = 2000        # Port to listen on (non-privileged ports are > 1023)

Dummy = 2000

fieldnames = ["Tijd1", "CapWaarde1", "Tijd2", "CapWaarde2", "Tijd3", "ResWaarde1", "Tijd4", "ResWaarde2", "Dummy"]

with open('data.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(4096)
            dataIN = repr(data)
            Tijd1, CapWaarde1, Tijd2, CapWaarde2, Tijd3, ResWaarde1, Tijd4, ResWaarde2 = dataIN.split(',')
            Tijd1 = Tijd1[1:]
            Tijd1 = Tijd1[1:]
            ResWaarde2 = ResWaarde2[:-1]
            if not data:
                break
            conn.sendall(data)
            
            with open('data.csv', 'a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                info = {
                    "Tijd1": Tijd1,
                    "CapWaarde1": CapWaarde1,
                    "Tijd2": Tijd2,
                    "CapWaarde2": CapWaarde2,
                    "Tijd3": Tijd3,
                    "ResWaarde1": ResWaarde1,
                    "Tijd4": Tijd4,
                    "ResWaarde2": ResWaarde2,
                    "Dummy": Dummy
                }
                
                csv_writer.writerow(info)
                print(Tijd1, CapWaarde1, Tijd2, CapWaarde2, Tijd3, ResWaarde1, Tijd4, ResWaarde2)