import socket

HOST = '192.168.192.42' # accept any IP-adres
PORT = 2000 # Port to listen on
bufferSize  = 1024
msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind((HOST, PORT))
    while True:
        bytesAddressPair = s.recvfrom(bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        #clientIP  = "Client IP Address:{}".format(address)
        
        print(clientMsg)
        #print(clientIP)

        # Sending a reply to client
        #s.sendto(bytesToSend, address)
