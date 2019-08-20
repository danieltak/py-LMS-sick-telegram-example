# -*- coding: utf-8 -*-
from __future__ import print_function

# This Code Base
# http://ros-robot.blogspot.jp/2011/08/pclapi-point-cloud-library-pcl-pcl-api.html
import sys
import numpy as np
import pcl
import random
import struct
import binascii
import socket
import pcl.pcl_visualization
import time
import csv
import re

start = time.time()

def comm_dists(ip, port):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the port where the server is listening
    server_address = (ip, port)
    print(sys.stderr, 'connecting to %s port %s' % server_address)
    sock.connect(server_address)

    try:

        # Send data
        message = b'sRN LMDscandata'
        message = b'\x02' + message + b'\x03'
        print(sys.stderr, 'sending "%s"' % message)
        sock.sendall(message)

        # Look for the response
        amount_received = 0
        amount_expected = len(message)

        while amount_received < amount_expected:
            data = sock.recv(5000)
            amount_received += len(data)
            print(sys.stderr, 'received "%s"' % data)

    finally:
        print(sys.stderr, 'closing socket')
        sock.close()
    return data

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def hex2sint(value):
    value = int(value, 16)
    return value - 0x100000000 if value > 0x7FFFFFFF else value

def dados_cart(data):
    dados_bin = data
    data = data.decode()
    inicio = data.find('DIST1')
    dados = data[inicio::].split()
    qtd = 0
    dists = []

    for n, dado in enumerate(dados):
        if n == 1:
            fator_escala = struct.unpack('>f', binascii.unhexlify(dado))[0]
            # print('Scale factor according to IEEE754: ', fator_escala)
        elif n == 2:
            escala_offset = struct.unpack('>f', binascii.unhexlify(dado))[0]
            # print('Scale factor offset acco. to IEEE754: ', escala_offset)
        elif n == 3:
            ang_inicio = hex2sint(dado)
            # print(ang_inicio)
        elif n == 4:
            graus = hex2sint(dado)
            # print(graus)
        elif n == 5:
            qtd = hex2sint(dado)
            print('Quantidade de medidas: ', qtd)
        if n in range(6, qtd + 6):
            dists.append(int(dado, 16))

    # List de distâncias, com o fator de escala
    # print(dists)
    dists = [x * fator_escala for x in dists]
    # print(dists)

    # print('Hex: ', [hex(i) for i in dados_bin])

    theta = []
    for x in range(0, qtd):
        theta.append((ang_inicio + (graus * x)) / 10000)
    # print('Theta graus: ', theta)

    theta = []
    for x in range(0, qtd):
        theta.append(np.deg2rad((ang_inicio + (graus * x)) / 10000))
    # print('Theta rad: ', theta)

    x = []
    y = []
    for i in range(0, qtd):
        a, b = pol2cart(dists[i], theta[i])
        x.append(a)
        y.append(b)

    return x, y, qtd


i=0
dados_csv = []

while i < int(c):

    # Simulação dados
    data = comm_dists('192.168.1.151', 2112)
    data = data.decode()
    data = data.replace('\x02','')
    data = data.replace('\x03', '')
    data = data.split()
    # data = re.sub(r'[^\x00-\x7f]', r'', data)
    print(data)
    # x, y, qtd = dados_cart(data)
    dados_csv.append(data)
    end = time.time()
    print(end - start, 'segundos')
    i=i+1

with open('nuvemPontos_' + time.strftime("%Y-%m-%d_%H-%M-%S") + '.csv', 'w') as f:
    w = csv.writer(f, delimiter=";", dialect="excel", lineterminator='\n')
    w.writerows(dados_csv)

end = time.time()
print(end - start, 'segundos')
