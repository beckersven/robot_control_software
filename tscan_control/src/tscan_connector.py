#! /usr/bin/python
import socket
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import struct
from std_srvs.srv import SetBool, SetBoolResponse
from threading import Lock
import time
import random
import signal
from itertools import izip
class PointCloudManager:
    def __init__(self, tscan_pc_ip="129.13.234.155", command_port=1999, point_data_port=2000):

        self.server_socket_1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_1.bind(("129.13.234.154", command_port))
        self.server_socket_1.listen(1)
        print("Windows client can now establish connection 1/2...")
        self.command_socket, client_config = self.server_socket_1.accept()
        random.random()
        if not client_config[0] == tscan_pc_ip:
            raise Exception("Wrong PC connected (not TSCAN-PC at {})! Aborting...".format(tscan_pc_ip))
        print("Established connection 1/2 with TSCAN PC!")


        self.server_socket_2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket_2.bind(("129.13.234.154", point_data_port))
        self.server_socket_2.listen(1)
        print("Windows client can now establish connection 2/2...")
        self.point_reception_socket, client_config = self.server_socket_2.accept()
        if not client_config[0] == tscan_pc_ip:
            raise Exception("Wrong PC connected (not TSCAN-PC at {})! Aborting...".format(tscan_pc_ip))
        print("Established connection 2/2 with TSCAN PC!")
        
        rospy.init_node("tscan_connector")
        self.point_cloud = PointCloud()
        self.point_cloud.header.frame_id = "zeiss_ttrack"
        self.point_cloud_pub = rospy.Publisher(name="/point_cloud", data_class=PointCloud, queue_size=5, latch=True)
        self.is_scanning = False
        self.command_mutex = Lock()

        self.activation_service = rospy.Service(name="/enable_tscan", service_class=SetBool, handler=self.activate_tscan_callback)    
        
        signal.signal(signal.SIGINT, self.shutdown_handler)
        data_overflow = ""
        while True:
            try:
                new_data = self.point_reception_socket.recv(1048576)
            
            # Common error that can be ignored
            except socket.error as (code, msg):
                if code is socket.EINTR:
                    break
            
            # Empty message indicates that the client (Windows) closed his sockets
            if new_data is None:
                break
            
            # Only process fully transmitted points and store the rest in the overflow:
            # 12 characters in socket-stream = 12 byte
            # 1 point = 3 float (single) values = 3 * 32 bit = 3 * 4 byte = 12 byte
            data = data_overflow + new_data
            to_process = data[0:len(data) - len(data) % 12]
            data_overflow = data[len(data) - len(data) % 12:]
            
            new_points = struct.unpack("<{}f".format(int(len(to_process) / 4)), to_process)
            
            print("Received {} points".format(len(to_process) / 12))
            for x,y,z in izip(*[iter(new_points)]*3):
                self.point_cloud.points.append(Point32(x * 1e-3,y * 1e-3,z * 1e-3))
            self.point_cloud.header.stamp = rospy.Time.now()
            self.point_cloud_pub.publish(self.point_cloud)
            
        


    def activate_tscan_callback(self, request):
        with self.command_mutex:
            if request.data and not self.is_scanning:
                self.command_socket.sendall(b"TURN ON ")
            elif not request.data and  self.is_scanning:
                self.command_socket.sendall(b"TURN OFF")
            else:
                return SetBoolResponse(success=False)
            response = self.command_socket.recv(2)
            self.is_scanning = not self.is_scanning
            return SetBoolResponse(success=response == "OK")

    def shutdown_handler(self, sig, frame):
        print("Requesting shutdown...")
        with self.command_mutex:
            if self.is_scanning:
                self.command_socket.sendall(b"TURN OFF")
                if not self.command_socket.recv(2) == "OK":
                    print("Shutdown failed")
            self.command_socket.sendall(b"SHUTDOWN")
            if not self.command_socket.recv(2) == "OK":
                print("Shutdown failed")
        print("SHUTDOWN succeeded")



if __name__=="__main__":
    PC = PointCloudManager()
