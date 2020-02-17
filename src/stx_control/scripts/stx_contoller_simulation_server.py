#!/usr/bin/env python

import socket
import signal
import sys
import rospy

ERR_CODE = 0


def signal_handler(signal, frame):
    # close the socket here
    conn.close()
    sys.exit(0)


TCP_IP = '127.0.0.1'
TCP_PORT = 6001
BUFFER_SIZE = 1024


def analise(msg):
    rospy.loginfo(msg)
    msg_type = data.split('(')[0]
    msg_id = data.split('(')[1].split(',')[0]
    rospy.loginfo("msg type: {}".format(msg_type))
    rospy.loginfo("msg id: {}".format(msg_id))

    mc_immediate_response = "{} {}\r\n".format(msg_id, ERR_CODE)
    mc_differed_response = "!{} {}\r\n".format(msg_id, ERR_CODE)

    rospy.loginfo("Sending ack response:  {}".format(mc_immediate_response))
    conn.send(mc_immediate_response)

    if msg_type == 'grMove5dof':
        # uncomment next line in order to simulate MC's different behavior of differed msg and immediate msg.
        # rospy.sleep(0.1)
        rospy.loginfo("Sending differed response:  {}".format(mc_differed_response))
        conn.send(mc_differed_response)


if __name__ == '__main__':
    rospy.init_node('simulation_mc')
    signal.signal(signal.SIGINT, signal_handler)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)

    conn, addr = s.accept()
    rospy.loginfo("Connection address:{}".format(addr))

    while 1:
        data = conn.recv(BUFFER_SIZE)
        if not data:
            break
        analise(data)

    conn.close()
