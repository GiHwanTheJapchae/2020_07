import socket

HOST = '127.0.0.1'
PORT = 8080

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client_socket.connect((HOST, PORT))

client_socket.sendall('receiving..'.encode())

data = client_socket.recv(1024)

while():

print('received', repr(data.decode()))

locations = data.split(',')

#0부터 짝수는 위도 홀수는 경도 locations를 나누기
for i in range(len(locations)):
    if i%2 == 0:
        latitude = locations[i]
    else:
        longtitude = locations[i]


client_socket.close()
