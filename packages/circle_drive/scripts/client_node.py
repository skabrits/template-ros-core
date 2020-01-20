#!/usr/bin/env python
import socket

import rospy
from std_msgs.msg import String


rospy.init_node('client_node')

pub = rospy.Publisher('/coord',String,queue_size=1)
# sleep(1)


class Client:
    def __init__(self, server):
        self.server = server
        self.start_con()

    def start_con(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.server[0], self.server[1]))

        while True:
            try:
                # message = input('Message: ')
                # if message == 'quit':
                #     print("Closed manually\n")
                #     print('Close the connection')
                #     self.writer.close()
                #     await self.writer.wait_closed()
                #     break
                rospy.loginfo("Connected!!!")
                #message = "Hello World!!!!"
                #self.writer.write(bytes((message+"\n"), "Utf-8"))
                #await self.writer.drain()
                #data = await self.reader.read(100)
                data = sock.recv(1024)
                rospy.loginfo(data.decode())
                command = str(data.decode())[:-1].split(" ")

                if len(command) > 0:
                    if command[0] == "///":
                        msg=String()
                        msg.data = command[1] + " " + command[2]
                        pub.publish(msg)
				
				

            except Exception as ex:
                rospy.loginfo("Closed due to the error\n", ex)
                rospy.loginfo('Close the connection')
                self.s.close()
                break

c = Client(('172.31.1.147', 8080))
