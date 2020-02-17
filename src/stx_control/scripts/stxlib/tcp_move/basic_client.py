import telnetlib
import logging


class BasicClient:
    msgCounter = 1

    def __init__(self, ip):
        self.ip = ip
        self.client = None

    def getMsgCounter(self):
        return self.msgCounter

    def connect(self):
        self.client = telnetlib.Telnet(self.ip, 6001)

    def disconnect(self):
        self.client = telnetlib.Telnet(self.ip, 6001)
        self.client.close()

    def home(self):
        logging.info('homing...')

    def enableAxis(self, axisId):
        msg = "AxEnable({},{},{})".format(self.msgCounter, axisId, 1)
        self.write(msg)

    def disableAxis(self, axisId):
        msg = "AxEnable({},{},{})".format(self.msgCounter, axisId, 0)
        self.write(msg)

    def enableGroup(self, groupId):
        msg = "grEnable({},{},{})".format(self.msgCounter, groupId, 1)
        self.write(msg)

    def disableGroup(self, groupId):
        msg = "grEnable({},{},{})".format(self.msgCounter, groupId, 0)
        self.write(msg)

    def gripperOpen(self):
        msg = "gripperToggle({},{})".format(self.msgCounter, 1)
        self.write(msg)

    def gripperClose(self):
        msg = "gripperToggle({},{})".format(self.msgCounter, 0)
        self.write(msg)

    def grJoints(self, groupId, j1, j2, j3, j4, j5, vCruise):
        msg = "grMove5dof({}, {}, {}, {}, {}, {}, {}, {})".format(self.msgCounter, groupId, j1, j2, j3, j4, j5, vCruise)
        self.write(msg)

    def moveAxisRealtive(self, jointNum, deg, vCruise):
        msg = "axMoveRelative({}, {}, {}, {})".format(self.msgCounter, jointNum, deg, vCruise)
        self.write(msg)

    def setAxisHome(self, jointNum):
        msg = "axMoveHome({}, {})".format(self.msgCounter, jointNum)
        self.write(msg)

    def recv(self):
        retVal = self.client.read_until(b"\n").decode('ascii')
        return retVal

    def write(self, str):
        self.msgCounter += 1
        self.client.write((str + "\n").encode('ascii'))
        logging.info("BASIC Client sent: %s", str)
        # retval = self.client.read_until(b"\n").decode('ascii')
        # logging.info("Basic Client receive: %s", retval)
        # return retval
