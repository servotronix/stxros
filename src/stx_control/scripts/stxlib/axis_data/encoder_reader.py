import socket
import struct
import logging


class AxisData:
    def __init__(self, data):
        self.__data = data

    def get_pCmd(self):
        return self.__data[4]

    def get_pFb(self):
        return self.__data[5]

    def get_vCmd(self):
        return self.__data[9]

    def get_vFb(self):
        return self.__data[10]

    def get_tFb(self):
        return self.__data[13]


class AxisDataReader:
    def __init__(self, ip):
        self.ip = ip
        self.HEADER_FORMAT = ('i' * 8) + ('i' * 8) + ('d' * 8)
        self.HEADER_SIZE = struct.calcsize(self.HEADER_FORMAT)

        self.AXIS_FORMAT = ('i' * 4) + ('d' * 10)
        self.AXIS_SIZE = struct.calcsize(self.AXIS_FORMAT)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def read(self, numOfAxes):
        """

        :type numOfAxes: int
        """
        word = ('\x01' * numOfAxes) + '\x00' * (64 - numOfAxes)
        sock = self.sock
        sock.sendto(word.encode('utf-8'), (self.ip, 60000))

        # todo: handle timeout
        # print('before')
        sock.settimeout(1.0)
        try:
            response, senderInfo = sock.recvfrom(1024)
            # print(''.join(format(x, '02x') for x in response))
            # print(response.format(x, '02x') for x in response)
        except socket.error:
            print('WARNING: UDP didnt provide response')
            return self.read(numOfAxes)
        # print('after')
        data = struct.unpack(self.HEADER_FORMAT, response[:self.HEADER_SIZE])

        axes_data = []  # type: list[AxisData]
        for i in range(0, numOfAxes):
            start_index = self.HEADER_SIZE + (i * self.AXIS_SIZE)
            end_index = start_index + self.AXIS_SIZE
            data = struct.unpack(self.AXIS_FORMAT, response[start_index:end_index])
            axes_data.append(AxisData(data))

        return axes_data

    def print_read(self, numOfAxes):
        data = self.read(numOfAxes)
        logging.info("========= Axis Data ==========")
        for i in range(0, numOfAxes):
            logging.info("[%d]: pCmd:%.2f, pFb:%.2f, vCmd:%.2f, vFb:%.2f, tFb:%.2f", i + 1, data[i].get_pCmd(),
                         data[i].get_pFb(), data[i].get_vCmd(), data[i].get_vFb(), data[i].get_tFb())
