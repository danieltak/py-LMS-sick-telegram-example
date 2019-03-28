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
    print(data)
    inicio = data.find('DIST1')
    dados = data[inicio::].split()
    qtd = 0
    dists = []

    for n, dado in enumerate(dados):
        if n == 1:
            fator_escala = struct.unpack('>f', binascii.unhexlify(dado))[0]
            print('Scale factor according to IEEE754: ', fator_escala)
        elif n == 2:
            escala_offset = struct.unpack('>f', binascii.unhexlify(dado))[0]
            print('Scale factor offset acco. to IEEE754: ', escala_offset)
        elif n == 3:
            ang_inicio = hex2sint(dado)
            print(ang_inicio)
        elif n == 4:
            graus = hex2sint(dado)
            print(graus)
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



# Simulação dados
data = comm_dists('192.168.1.151', 2112)
x, y, qtd = dados_cart(data)

end = time.time()
print(end - start, 'segundos')

# pcl::PointCloud<pcl::PointXYZRGB> cloud;
cloud = pcl.PointCloud_PointXYZRGB()

# Fill in the cloud data
# cloud.width  = 15;
# cloud.height = 10;
# cloud.points.resize (cloud.width * cloud.height)
# cloud.resize (np.array([15, 10], dtype=np.float))
# points = np.zeros((10, 15, 4), dtype=np.float32)
points = np.zeros((qtd, 4), dtype=np.float32)
RAND_MAX = 1.0
# Generate the data
for i in range(0, qtd):
    # set Point Plane
    points[i][0] = x[i]
    points[i][1] = y[i]
    points[i][2] = 0
    points[i][3] = 255 << 16 | 255 << 8 | 255

# Set a few outliers
# points[0][2] = 2.0;
# points[3][2] = -2.0;
# points[6][2] = 4.0;

# for i in range(0, 150):
#     print (points[i][0], points[i][1], points[i][2], points[i][3])

cloud.from_array(points)
print(cloud)

# Create the segmentation object
# pcl::SACSegmentation<pcl::PointXYZRGB> seg
seg = cloud.make_segmenter()
# Optional
seg.set_optimize_coefficients (True)
# Mandatory
seg.set_model_type (pcl.SACMODEL_PLANE)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_distance_threshold (0.1)

# pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients)
# pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
inliers, model = seg.segment()

# if inliers.size
# 	return
# end

print (model)
# std::cerr << "Model coefficients: " << coefficients->values[0] << " "
# << coefficients->values[1] << " "
# << coefficients->values[2] << " "
# << coefficients->values[3] << std::endl;
#
# std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
# for (size_t i = 0; i < inliers->indices.size (); ++i)
# {
#   std::cerr << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
#   << cloud.points[inliers->indices[i]].y << " "
#   << cloud.points[inliers->indices[i]].z << std::endl;
#   cloud.points[inliers->indices[i]].r = 255;
#   cloud.points[inliers->indices[i]].g = 0;
#   cloud.points[inliers->indices[i]].b = 0;
# }
for i in inliers:
    points[i][3] = 255 << 16 | 255 << 8 | 0

cloud.from_array(points)

#
# pcl::visualization::CloudViewer viewer("Cloud Viewer");
# viewer.showCloud(cloud.makeShared());
# while (!viewer.wasStopped ())
visual = pcl.pcl_visualization.CloudViewing()
visual.ShowColorCloud(cloud)

end = time.time()
print(end - start, 'segundos')

v = True
while v:
    v=not(visual.WasStopped())


