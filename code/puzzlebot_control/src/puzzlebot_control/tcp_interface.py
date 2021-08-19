import numpy as np
import rospy
import sys
import time
import socket
import logging
import threading
from Queue import Queue
from collections import deque

class ClientThread(threading.Thread):
    def __init__(self, clientAddress, clientsocket, data_q):
        threading.Thread.__init__(self)
        self.dq = deque(maxlen=1)
        self.data_q = data_q
        self.csocket = clientsocket
        self.csocket.settimeout(0.5)
        self.clientAddress = clientAddress

        self.dt = 0
        self.prev_time = time.time()
        print ("New connection added: ", clientAddress)

    def run(self):
        print ("Connection from : ", self.clientAddress)
        while True:
            try:
                if len(self.dq) == 0: 
                    continue
                val = self.dq.pop()
                self.csocket.send(val)
                rospy.loginfo("sent msg to %s:%d, %s" % (self.clientAddress[0], self.clientAddress[1], val))
            except:
                pass

            try:
                data = self.csocket.recv(9)
                if len(data) == 0:
                    continue
                rospy.logwarn("From %s, %d : %s" % (self.clientAddress[0], self.clientAddress[1], data))
                self.data_q.append(data)

                curr_time = time.time()
                self.dt = curr_time - self.prev_time
                self.prev_time = curr_time
            except:
                rospy.logerr("no msg from %s:%d" % self.clientAddress)

class TCPBridge:
    def __init__(self, N):
        self.N = N
        self.TCP_IP = '0.0.0.0'
        self.TCP_PORT = 8080 
        self.BUFFER_SIZE = 9

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((self.TCP_IP, self.TCP_PORT))
        self.threads = []
        self.robot_ips = []
        self.data_qs = []
        self.dts = []
    
    def start_listen(self):
        for i in range(self.N):

            rospy.loginfo("waiting for %d th robot" % i)
            self.s.listen(2)
            clientsock, clientAddress = self.s.accept()
            data_q = deque(maxlen=1)
            self.data_qs.append(data_q)
            newthread = ClientThread(clientAddress, clientsock, data_q)
            newthread.start()
            self.threads.append(newthread)
            rospy.loginfo("%d robot connected" % len(self.threads))
            self.dts.append(0)

            # process robot ips
            try:
                ip = int(clientAddress[0].split('.')[-1])
                self.robot_ips.append(ip)
            except:
                pass

        time.sleep(2)

    def send(self, tw):
        # tw is 2-by-N
        for i in range(self.N):
            [v, w] = tw[:, i]
            msg = "%d,%d" % (v, w) # left vel, right vel

            self.threads[i].dq.append(msg)
            self.dts[i] = self.threads[i].dt

    def end(self):
        print 'Connection closed.'
        self.s.close()

