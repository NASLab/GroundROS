import socket
import numpy as np
import sys
import struct
from time import sleep


class NaslabNetwork(object):

    def __init__(self, ip_address='192.168.0.25'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1)
        server_address = (ip_address, 1895)
        print >>sys.stderr, 'Connecting To %s Port %s' % server_address
        self.sock.connect(server_address)
        self.degree_to_rad = np.pi / 180

    def getStates(self, num):
        # get position
        check = struct.unpack('<B', self.sock.recv(1))[0]
        if check is not 2:
            print 'Warning: Bad Qualisys Packet'
            pose_msg_x = float('nan')
        recieved_data = self.sock.recv(4096)
        if len(recieved_data) < 12:
            print 'bad 2'
            pose_msg_x = float('nan')
        num_byte = num * 24
        pose_msg_x = struct.unpack('<f', recieved_data[num_byte:num_byte + 4])[0]
        pose_msg_y = struct.unpack('<f', recieved_data[num_byte + 4:num_byte + 8])[0]
        pose_msg_theta = struct.unpack('<f', recieved_data[num_byte + 12:num_byte + 16])[0] 
        # print num, pose_msg_theta
        return pose_msg_x, pose_msg_y, pose_msg_theta

    def close(self):
        self.sock.close()


class LabNavigation(object):

    def __init__(self, ip_address='192.168.0.25'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(10)
        server_address = (ip_address, 1895)
        print >>sys.stderr, 'Connecting To %s Port %s' % server_address
        try:
            self.sock.connect(server_address)
        except socket.error:
            print 'Connection failed. Check server.'
            raise

    def getStates(self, num):
        # get position
        self.sock.send(str(num))
        packed_buffer = self.sock.recv(25)
        try:
            agent_id = struct.unpack('<B', packed_buffer[:1])[0]
        except struct.error:
            print 'Connection terminated by server.'
            raise
        x = struct.unpack('<f', packed_buffer[1:5])[0]
        y = struct.unpack('<f', packed_buffer[5:9])[0]
        z = struct.unpack('<f', packed_buffer[9:13])[0]
        yaw = struct.unpack('<f', packed_buffer[13:17])[0]
        pitch = struct.unpack('<f', packed_buffer[17:21])[0]
        roll = struct.unpack('<f', packed_buffer[21:25])[0]
        return agent_id, x, y, z, yaw, pitch, roll

    def close(self):
        self.sock.close()
