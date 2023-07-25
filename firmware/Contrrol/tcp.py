from datetime import datetime
import time
from socket import *
import sys

# Адрес центральной платы
host = '192.168.0.14'
host2 = '192.168.0.1'
port = 81
addr = (host, port)
addr2 = (host2, port)

tcp_socket = socket(AF_INET, SOCK_STREAM)
tcp_socket.connect(addr)

tcp_socket2 = socket(AF_INET, SOCK_STREAM)
tcp_socket2.connect(addr2)

# data = input('write to server: ')
# if not data:
#    tcp_socket.close()
#    sys.exit(1)

# encode - перекодирует введенные данные в байты, decode - обратно
# data = str.encode(data)

# Флаги для формирования сообщения
comandFlag = "C"
addresFlag = "A"
dataFlag = "D"
NameFlag = "N"
endFlag = "x"

# data = int(data)
# собираем сообщение
data = comandFlag
data += "1"
data += addresFlag
data += "16"
data += dataFlag
data += "8"
data += NameFlag
data += "0"
data += endFlag

# Channel off
data2 = comandFlag
data2 += "4"
data2 += addresFlag
data2 += "0"
data2 += dataFlag
data2 += "0"
data2 += NameFlag
data2 += "0"
data2 += endFlag

while 1:
    var = 0
    while var < 24:
        # PWM set
        data1 = comandFlag
        data1 += "5"
        data1 += addresFlag
        data1 += "0"
        data1 += dataFlag
        data1 += "1000"
        data1 += NameFlag
        data1 += str(var)
        data1 += endFlag

        # Channel on
        data2 = comandFlag
        data2 += "4"
        data2 += addresFlag
        data2 += "0"
        data2 += dataFlag
        data2 += "1"
        data2 += NameFlag
        data2 += str(var)
        data2 += endFlag

        tcp_socket.send(data1.encode())
        data_r = tcp_socket.recv(1024)
        print('read:')
        print(data_r)

        time.sleep(0.2)

        tcp_socket.send(data2.encode())
        data_r = tcp_socket.recv(1024)
        print('read:')
        print(data_r)

        time.sleep(0.2)

        var += 1


    var = 0
    while var < 24:
        # Channel off
        data2 = comandFlag
        data2 += "4"
        data2 += addresFlag
        data2 += "0"
        data2 += dataFlag
        data2 += "0"
        data2 += NameFlag
        data2 += str(var)
        data2 += endFlag

        tcp_socket.send(data2.encode())
        data_r = tcp_socket.recv(1024)
        print('read:')
        print(data_r)

        time.sleep(0.2)

        var += 1

    time.sleep(1)

tcp_socket.close()
