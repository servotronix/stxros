#!/usr/bin/env python
# coding=utf-8
import sys
import signal
from stxlib.tcp_move.basic_client import BasicClient


MC_IP = '90.0.0.1'


def signal_handler(signal, frame):
    tn.disableGroup(65)
    sys.exit(0)


if __name__ == '__main__':
    try:
        signal.signal(signal.SIGINT, signal_handler)

        tn = BasicClient(MC_IP)
        tn.connect()

        while True:
            print "============ Press `1` to begin setting up the robot ..."
            print "============ Press `0` when done with the setting ..."
            inp = raw_input()
            if inp == '1':
                tn.enableGroup(65)
                break

        while True:

            print "============ Enter joint number..."
            joint_num = raw_input()

            if joint_num == '0':
                tn.disconnect()
                break

            print "============ Enter movement degree..."
            deg = raw_input()
            neg_deg = '-' + deg

            if deg == '0':
                break

            if int(deg, 10) > 100:
                print "degree is to big, try again"
                continue

            print "=========== Enter movement speed..."
            speed = raw_input()

            if speed == '0':
                break

            if int(speed, 10) > 100:
                print "speed is to fast, try again"
                continue

            while True:
                print "\n======the robot will move '{}°' degrees with joint number '{}' in a speed of {}. (y/n)?=====".format(
                    deg, joint_num, speed)
                ans = raw_input()

                if ans == 'n':
                    print "\nEnter new command \n"
                    break

                if ans == 'y':
                    print "press '+' to move to {} degrees or '-' to move -{} degrees".format(deg, deg)
                    print "to change joints/speed/degree press Enter"
                    print "when joint is home pres 'H'\n"

                    while True:
                        direction = raw_input()
                        if direction == '':
                            break

                        if direction == 'H':
                            tn.setAxisHome(joint_num)

                            break

                        if direction == '+':
                            tn.moveAxisRealtive(joint_num, deg, speed)
                            print "\nrobot is moving!"

                        else:
                            if direction == '-':
                                tn.moveAxisRealtive(joint_num, neg_deg, speed)
                                print "\nrobot is moving!"
                            else:
                                print "input should be '+' or '-' (press 'Enter' to change joint movement settings)"


                # if ans == '0':
                #     tn.disconnect()
                #     break

                break

    except KeyboardInterrupt:
        pass



