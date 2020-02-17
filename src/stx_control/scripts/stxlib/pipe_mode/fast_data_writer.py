import socket
import struct
import logging

#udp
class PipeModeSender:

    def __init__(self, ip):
        self.ip = ip

    def generic_send(self, values, is_doubles=True, start_index=1):

        int32CS16 = 0                   # checksum
        int32nVar = len(values)        # means we send to doubles
        int32StartVar = start_index               # index of 1st variable is 1
        int32DataType = 0 if is_doubles else 1               # 0 - doubles, 1 - ints
        packStr = 'Iiii'
        for value in values:
            if (is_doubles):
                packStr += 'd'
            else:
                packStr += 'i'
        msg = struct.pack(packStr, int32CS16, int32nVar, int32StartVar, int32DataType, *values)
        # logging.info('msg length is: %d', len(msg))      # should print 16 for header + 8 for each double
        bytesSum = 1                            # chksm function is 1+sum of all bytes in msg
        for byte in msg:
            bytesSum += ord(byte)
        int32CS16 = bytesSum
        msg = struct.pack(packStr, int32CS16, int32nVar, int32StartVar, int32DataType, *values)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(msg, (self.ip, 60001))

    def send_doubles(self, doubles):
        self.generic_send(doubles)
        logging.info("PIPE_MODE doubles sent: [%s]", ", ".join(map(str, doubles)))

    def send_ints(self, ints):
        self.generic_send(ints, False)
        logging.info("PIPE_MODE ints sent: [%s]", ", ".join(map(str, ints)))

    # ======== HIGHER LEVEL INTERFACES ===========

    def send_positions(self, positions):
        self.send_doubles(positions)

    def send_velocities(self, velocities):
        self.send_doubles(velocities)

    def send_positions_and_velocities(self, positions, velocities):
        if len(positions) != len(velocities):
            logging.error('positions & velocities are different lengths')
        else:
            posStr = " ".join(map(str, positions))
            velStr = " ".join(map(str, velocities))
            logging.info('sending positions\t[%s] & velocities\t[%s]', posStr, velStr)

            doubles = []
            for i in range(0, len(positions)):
                doubles.append(positions[i])
                doubles.append(velocities[i])
            self.send_doubles(doubles)

