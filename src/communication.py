import socket
import numpy as np
import sys
import struct


class HuskeyConnect(object):

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1)
        ip_address = '192.168.0.25'
        server_address = (ip_address, 1895)
        print >>sys.stderr, 'Connecting To %s Port %s' % server_address
        self.sock.connect(server_address)
        self.degree_to_rad = np.pi / 180

    def getStates(self):
        # get position
        check = struct.unpack('<B', self.sock.recv(1))[0]
        if check is not 2:
            print 'Warning: Bad Qualisys Packet'
            pose_msg_x = float('nan')
        recieved_data = self.sock.recv(4096)
        if len(recieved_data) < 12:
            print 'bad 2'
            pose_msg_x = float('nan')
        pose_msg_x = struct.unpack('<f', recieved_data[:4])[0]
        pose_msg_y = struct.unpack('<f', recieved_data[4:8])[0]
        pose_msg_theta = struct.unpack('<f', recieved_data[12:16])[0] * self.degree_to_rad
        return pose_msg_x / 1000, pose_msg_y / 1000, pose_msg_theta

    def close(self):
        self.sock.close()
