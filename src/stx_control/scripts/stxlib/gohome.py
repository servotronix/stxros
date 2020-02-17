
import logging
from axis_data.encoder_reader import AxisDataReader
from pipe_mode.fast_data_writer import PipeModeSender
from tcp_move.basic_client import BasicClient

import time
MC_IP = "192.168.56.101"

logging.basicConfig(
    format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')

writer = PipeModeSender(MC_IP)
reader = AxisDataReader(MC_IP)
telnet = BasicClient(MC_IP)

telnet.connect()


telnet.enable_range(1, 5)

reader.print_read(6)

axes_data = reader.read(6)
BASIC_VELOCITY = 10
velocities = []
for i in range(0, len(axes_data)):
    if axes_data[i].get_pFb() > 0:
        velocities.append(-BASIC_VELOCITY)
    else:
        velocities.append(BASIC_VELOCITY)

writer.send_positions_and_velocities([0]*len(axes_data),  velocities)

time.sleep(1)
reader.print_read(6)

telnet.disable_range(1, 5)

telnet.disconnect()
