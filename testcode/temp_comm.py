import socket

class comm:
    #client_socket 클래스내 함수간 공유 가능한지 알아보기
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # client_socket 변수 생성

    def __init__(self, host, port):

        self.host = host#호스트 주소
        self.port = port#호스트 포트
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((self.host, self.port))
        client_socket.sendall('connected'.encode())#연결 확인 신호
    
    def conn_getd(self):

        data = client_socket.rect(1024)#data에 client_socket ㄷ이터 저장
        print('received', repr(data.decode()))

        locations = data.split(',')#locations에 데이터 ','을 기준으로 나누어 리스트화

        return locations

    def conn_ed(self):
        client_socket.close()



"""
HOST = '127.0.0.1'
PORT = 8080

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client_socket.connect((HOST, PORT))

client_socket.sendall('receiving..'.encode())

data = client_socket.recv(1024)
print('received', repr(data.decode()))

locations = data.split(',')

#0부터 짝수는 위도 홀수는 경도 locations를 나누기
for i in range(len(locations)):
    if i % 2 == 0:
        latitude = locations[i]
    else:
        longtitude = locations[i]


client_socket.close()
"""
